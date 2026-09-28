// Copyright 2026. All rights reserved.

#include "motion_controller.h"
#include <algorithm>
#include <cmath>

#include "sdkconfig.h"

#if defined(CONFIG_LEAP_HEADING_HOLD)
static constexpr bool kHeadingHoldEnabled = true;
#else
static constexpr bool kHeadingHoldEnabled = false;
#endif
// 航向修正在左右轮上产生的目标转速差，不超过当前平均目标转速的该比例，
// 避免在起步斜坡的低速段把某一侧目标推到反方向。
static constexpr float kHeadingMaxTrimRatio = 0.3f;

// 左右轮"每个编码器脉冲对应的实际行程"之比（右/左），用于修正两侧减速比或
// 轮径的差异。按均值不变拆成两侧系数，只修正左右差，不改变整体距离标定。
// 标定方法见 doc/heading_hold.md。
static constexpr float kRightLeftTravelRatio =
    static_cast<float>(CONFIG_LEAP_RIGHT_LEFT_TRAVEL_RATIO_X10000) / 10000.0f;
static constexpr float kWheelTravelScale[2] = {
    2.0f / (1.0f + kRightLeftTravelRatio),
    2.0f * kRightLeftTravelRatio / (1.0f + kRightLeftTravelRatio)};

static const float kWheelDiameter = 65.0f;
static const float kTrackWidth = 131.7f;
static const float kLy = kTrackWidth / 2.0f;
static const float kRpmToMms = (M_PI * kWheelDiameter) / 60.0f;
static const float kMmsToRpm = 1.0f / kRpmToMms;

// Match the gains used by the repeatable startup runs in build/log. NVS may
// override them at runtime, but a freshly flashed board must start from the
// same known baseline.
static const float kKp = 1.2f, kKi = 0.5f, kKd = 0.0f, kMaxPwm = 255.0f;
static const float kMaxPosVel = 250.0f;
static const float kMaxPosYaw = 3.0f;
static const int kRelativeTurnSettleMaxCycles = 5;
static const float kRelativeTurnSettleVxMms = 10.0f;

static float MoveTowards(float current, float target, float max_step)
{
  // Force one exact zero-target control cycle on a direction reversal. This
  // gives the incremental wheel PI a deterministic reset point instead of
  // stepping directly from a small positive target to a negative one.
  if (current * target < 0.0f && std::abs(current) <= max_step)
  {
    return 0.0f;
  }
  return target > current ? std::min(current + max_step, target) : std::max(current - max_step, target);
}

static bool IsNearZero(float value)
{
  return std::abs(value) < 0.001f;
}

float MotionController::NormalizeAngle(float angle)
{
  while (angle > M_PI)
    angle -= 2.0f * M_PI;
  while (angle < -M_PI)
    angle += 2.0f * M_PI;
  return angle;
}

MotionController::MotionController(
    At8236Motor &left_motor, At8236Motor &right_motor,
    QuadratureEncoder &left_encoder, QuadratureEncoder &right_encoder)
    : motors_{&left_motor, &right_motor},
      encs_{&left_encoder, &right_encoder},
      pid_vel_{
          PidController(PidMode::kIncremental, kKp, kKi, kKd, -kMaxPwm, kMaxPwm),
          PidController(PidMode::kIncremental, kKp, kKi, kKd, -kMaxPwm, kMaxPwm)},
      wheel_pair_controller_(WheelPairControllerConfig{}),
      pid_pos_x_(PidMode::kPositional, 2.1f, 0.0f, 0.5f, -kMaxPosVel, kMaxPosVel),
      pid_pos_y_(PidMode::kPositional, 2.1f, 0.0f, 0.5f, -kMaxPosVel, kMaxPosVel),
      pid_pos_yaw_(PidMode::kPositional, 2.5f, 0.0f, 0.5f, -kMaxPosYaw, kMaxPosYaw) {}

void MotionController::SetMaxAcceleration(float accel_mms2)
{
  if (std::isfinite(accel_mms2) && accel_mms2 > 0.0f)
  {
    max_accel_rpm_s_ = accel_mms2 * kMmsToRpm;
  }
}

void MotionController::MoveToPosition(float target_x, float target_y, float target_yaw)
{
  if (!motion_enabled_)
  {
    Stop();
    return;
  }
  straight_control_enabled_ = false;
  wheel_pair_controller_.Reset();
  for (int i = 0; i < kNumWheels; ++i)
  {
    pid_vel_[i].Reset();
  }
  target_pos_x_ = target_x;
  target_pos_y_ = target_y;
  target_pos_yaw_ = target_yaw;
  control_mode_ = ControlMode::kPosition;
}

void MotionController::MoveRelative(float distance_mm, float angle_deg)
{
  if (!motion_enabled_)
  {
    Stop();
    return;
  }
  straight_control_enabled_ = false;
  wheel_pair_controller_.Reset();
  seq_stable_count_ = 0;
  control_mode_ = ControlMode::kRelativeSeq;
  seq_target_dist_ = distance_mm;
  seq_target_angle_ = angle_deg * (M_PI / 180.0f);

  seq_start_x_ = pos_x_;
  seq_start_y_ = pos_y_;
  seq_start_imu_yaw_ = imu_yaw_rel_;

  pid_pos_x_.Reset();
  pid_pos_yaw_.Reset();
  for (int i = 0; i < kNumWheels; ++i)
  {
    pid_vel_[i].Reset();
    target_vel_[i] = 0.0f;
  }

  if (std::abs(distance_mm) > 1.0f)
    seq_state_ = SeqState::kForward;
  else if (std::abs(angle_deg) > 0.5f)
    seq_state_ = SeqState::kTurn;
  else
    seq_state_ = SeqState::kIdle;
}

void MotionController::GetOdometry(float *x, float *y, float *yaw) const
{
  if (x)
    *x = pos_x_;
  if (y)
    *y = pos_y_;
  if (yaw)
    *yaw = odom_yaw_;
}

void MotionController::GetOdometryQuaternion(
    float *qw, float *qx, float *qy, float *qz) const
{
  const float half_yaw = odom_yaw_ * 0.5f;

  if (qw)
    *qw = std::cos(half_yaw);
  if (qx)
    *qx = 0.0f;
  if (qy)
    *qy = 0.0f;
  if (qz)
    *qz = std::sin(half_yaw);
}

float MotionController::GetTotalDistance() const
{
  return total_distance_;
}

void MotionController::ResetOdometry()
{
  for (int i = 0; i < kNumWheels; ++i)
  {
    encs_[i]->ResetCount();
    last_count_[i] = 0;
    filtered_rpm_[i] = 0.0f;
    pid_vel_[i].Reset();
    final_target_vel_[i] = 0.0f;
    target_vel_[i] = 0.0f;
  }
  wheel_pair_controller_.Reset();
  heading_hold_.Reset();
  last_sync_error_rpm_ = 0.0f;
  last_sync_correction_pwm_ = 0.0f;
  wheel_sync_active_ = false;
  startup_sync_active_ = false;
  wheel_startup_fault_ = false;
  straight_control_enabled_ = false;
  need_reset_yaw_offset_ = true;
  total_distance_ = 0.0f; // 重置总路程
  pos_x_ = pos_y_ = odom_yaw_ = imu_yaw_rel_ = 0.0f;
  target_pos_x_ = target_pos_y_ = target_pos_yaw_ = 0.0f;
  pid_pos_x_.Reset();
  pid_pos_y_.Reset();
  pid_pos_yaw_.Reset();
}

void MotionController::CalculateKinematics(float linear_x, float linear_y, float angular_z)
{
  (void)linear_y;
  final_target_vel_[0] = (linear_x - angular_z * kLy) * kMmsToRpm;
  final_target_vel_[1] = (linear_x + angular_z * kLy) * kMmsToRpm;
}

void MotionController::Drive(float linear_x, float linear_y, float angular_z)
{
  if (!motion_enabled_)
  {
    Stop();
    return;
  }
  if (!std::isfinite(linear_x) ||
      !std::isfinite(linear_y) ||
      !std::isfinite(angular_z))
  {
    Stop();
    return;
  }
  if (IsNearZero(linear_x) && IsNearZero(linear_y) && IsNearZero(angular_z))
  {
    Stop();
    return;
  }

  const float previous_final_target[kNumWheels] = {
      final_target_vel_[0], final_target_vel_[1]};
  control_mode_ = ControlMode::kVelocity;
  straight_control_enabled_ = !IsNearZero(linear_x) && IsNearZero(angular_z);
  CalculateKinematics(linear_x, linear_y, angular_z);
  if (!std::isfinite(final_target_vel_[0]) ||
      !std::isfinite(final_target_vel_[1]))
  {
    Stop();
    return;
  }
  for (int i = 0; i < kNumWheels; ++i)
  {
    if (previous_final_target[i] * final_target_vel_[i] < 0.0f)
    {
      pid_vel_[i].Reset();
      wheel_pair_controller_.Reset();
    }
  }
}

void MotionController::SetMotionEnabled(bool enabled)
{
  if (!enabled)
  {
    motion_enabled_ = false;
    Stop();
    return;
  }
  if (!motion_enabled_)
  {
    // Clear any target that may have been written while disabled before the
    // executor permission is restored. A new command is always required.
    Stop();
    motion_enabled_ = true;
  }
}

void MotionController::SetBatteryVoltage(float voltage_v)
{
  if (voltage_v > 0.0f)
  {
    battery_voltage_v_ = voltage_v;
  }
}

float MotionController::CalculateBasePwm(
    int wheel_index, float target_vel, float pid_output) const
{
  // 左右轮分别独立的速度-PWM前馈模型
  // 左轮悬空实验：
  //  根据当前悬空实验数据拟合得到：
  //   14.69 RPM -> ~130.5 PWM
  //   29.38 RPM -> ~142.5 PWM
  //   44.07 RPM -> ~155.0 PWM
  //   58.76 RPM -> ~165.5 PWM
  // PWM_FF=119+0.80*|RPM|
  //
  //
  //  右轮悬空正向实验：
  //  14.69 RPM -> ~142.5 PWM
  //  29.38 RPM -> ~150.5 PWM
  //  44.07 RPM -> ~160.0 PWM
  //  58.76 RPM -> ~169.5 PWM
  //
  //  四点线性拟合：
  //  PWM_ff = 133 + 0.62 * |RPM|

  const float kFfoffset = (wheel_index == 0) ? 119.0f : 133.0f;
  const float kFfSlope = (wheel_index == 0) ? 0.80f : 0.62f;

  // 没有目标速度时，电机不输出PWM
  if (std::abs(target_vel) <= 0.1f)
  {
    return 0;
  }
  // 根据目标转速计算前馈PWM大小
  float ff_pwm = kFfoffset + kFfSlope * std::abs(target_vel);
  // 根据目标转速决定方向
  if (target_vel < 0.0f)
  {
    ff_pwm = -ff_pwm;
  }
  // 前馈提供主要控制量，pi负责小范围误差修正
  float out = ff_pwm + pid_output;

  return std::clamp(out, -kMaxPwm, kMaxPwm);
}

void MotionController::Update(float dt, float current_imu_yaw_rad)
{
  if (dt <= 0.0f)
    return;

  // 1. 读取并计算转速
  const float kAlpha = 0.35f;
  const float kPulsesPerRev = 4680.0f;
  const int sign[kNumWheels] = {-1, 1}; // 右侧轮反向读取

  for (int i = 0; i < kNumWheels; ++i)
  {
    int32_t current_count = encs_[i]->GetCount() * sign[i];
    float pps = (current_count - last_count_[i]) / dt;
    last_count_[i] = current_count;
    float raw_rpm = (pps / kPulsesPerRev) * 60.0f * kWheelTravelScale[i];
    filtered_rpm_[i] = kAlpha * raw_rpm + (1.0f - kAlpha) * filtered_rpm_[i];
  }

  // 2. 里程计更新
  float local_vx, local_vy, local_vw;
  GetVelocity(&local_vx, &local_vy, &local_vw);

  // 累加计算小车真实走过的总路程 (用于精确定长循迹)
  float step_dist = std::sqrt(local_vx * local_vx + local_vy * local_vy) * dt;
  total_distance_ += step_dist;

  if (need_reset_yaw_offset_)
  {
    yaw_offset_ = current_imu_yaw_rad;
    need_reset_yaw_offset_ = false;
  }
  imu_yaw_rel_ = NormalizeAngle(current_imu_yaw_rad - yaw_offset_);
  odom_yaw_ += local_vw * dt;
  odom_yaw_ = NormalizeAngle(odom_yaw_);
  pos_x_ += (local_vx * std::cos(odom_yaw_) - local_vy * std::sin(odom_yaw_)) * dt;
  pos_y_ += (local_vx * std::sin(odom_yaw_) + local_vy * std::cos(odom_yaw_)) * dt;
  if (!motion_enabled_)
  {
    // Encoder and odometry state above remain live while output is inhibited.
    Stop();
    return;
  }
  // 3. 执行外环控制逻辑
  if (control_mode_ == ControlMode::kPosition)
  {
    float dx = target_pos_x_ - pos_x_;
    float dy = target_pos_y_ - pos_y_;
    float distance = std::sqrt(dx * dx + dy * dy);

    float cmd_vx = 0.0f, vw_cmd = 0.0f;
    if (distance > 15.0f)
    {
      float angle_error = NormalizeAngle(std::atan2(dy, dx) - odom_yaw_);
      vw_cmd = pid_pos_yaw_.Calculate(angle_error, 0.0f, dt);
      float v_cmd_raw = pid_pos_x_.Calculate(distance, 0.0f, dt);
      cmd_vx = std::cos(angle_error) > 0.0f ? v_cmd_raw * std::cos(angle_error) : 0.0f;
    }
    else
    {
      float final_angle_error = NormalizeAngle(target_pos_yaw_ - odom_yaw_);
      vw_cmd = pid_pos_yaw_.Calculate(final_angle_error, 0.0f, dt);
    }
    CalculateKinematics(cmd_vx, 0.0f, vw_cmd);
  }
  else if (control_mode_ == ControlMode::kRelativeSeq)
  {
    float cmd_vx = 0.0f, vw_cmd = 0.0f;

    if (seq_state_ == SeqState::kForward)
    {
      float dx = pos_x_ - seq_start_x_, dy = pos_y_ - seq_start_y_;
      float moved_dist = std::sqrt(dx * dx + dy * dy) * (seq_target_dist_ < 0 ? -1 : 1);

      if (std::abs(seq_target_dist_ - moved_dist) < 10.0f)
      {
        for (int i = 0; i < kNumWheels; ++i)
        {
          pid_vel_[i].Reset();
          target_vel_[i] = 0.0f;
        }
        if (std::abs(seq_target_angle_) > 0.01f)
        {
          seq_state_ = SeqState::kSettle;
          settle_count_ = 0;
        }
        else
          seq_state_ = SeqState::kIdle;
      }
      else
      {
        cmd_vx = pid_pos_x_.Calculate(seq_target_dist_, moved_dist, dt);
      }
    }
    else if (seq_state_ == SeqState::kSettle)
    {
      if (std::abs(local_vx) < kRelativeTurnSettleVxMms ||
          ++settle_count_ > kRelativeTurnSettleMaxCycles)
      {
        seq_start_imu_yaw_ = imu_yaw_rel_;
        pid_pos_yaw_.Reset();
        seq_stable_count_ = 0;
        seq_state_ = SeqState::kTurn;
      }
    }
    else if (seq_state_ == SeqState::kTurn)
    {
      float moved_angle = NormalizeAngle(imu_yaw_rel_ - seq_start_imu_yaw_);
      float err = NormalizeAngle(seq_target_angle_ - moved_angle);

      if (std::abs(err) < 0.05f && std::abs(local_vw) < 0.1f)
      {
        if (++seq_stable_count_ > 10)
        {
          seq_state_ = SeqState::kIdle;
        }
        vw_cmd = 0.0f;
      }
      else
      {
        seq_stable_count_ = 0;
        vw_cmd = pid_pos_yaw_.Calculate(seq_target_angle_, moved_angle, dt);

        const float kMinVw = 0.15f;
        if (vw_cmd > 0.01f && vw_cmd < kMinVw)
          vw_cmd = kMinVw;
        if (vw_cmd < -0.01f && vw_cmd > -kMinVw)
          vw_cmd = -kMinVw;
      }
    }
    CalculateKinematics(cmd_vx, 0.0f, vw_cmd);
  }

  // 4. 下发电机控制指令与PID计算
  float max_rpm_step = max_accel_rpm_s_ * dt;
  bool velocity_stop_command = (control_mode_ == ControlMode::kVelocity);
  for (int i = 0; i < kNumWheels && velocity_stop_command; ++i)
  {
    velocity_stop_command = IsNearZero(final_target_vel_[i]);
  }

  if (velocity_stop_command)
  {
    for (int i = 0; i < kNumWheels; ++i)
    {
      target_vel_[i] = 0.0f;
      final_target_vel_[i] = 0.0f;

      last_raw_pwm_[i] = 0.0f;
      last_final_pwm_[i] = 0.0f;

      pid_vel_[i].Reset();
      motors_[i]->Brake();
    }
    wheel_pair_controller_.Reset();
    heading_hold_.Reset();
    last_sync_error_rpm_ = 0.0f;
    last_sync_correction_pwm_ = 0.0f;
    wheel_sync_active_ = false;
    startup_sync_active_ = false;
    return;
  }

  // 对目标轮速进行加速度限制
  for (int i = 0; i < kNumWheels; ++i)
  {
    // 普通速度控制模式：
    // 对/cmd_vel的目标轮速进行加速度控制，
    // 避免目标转速发生大幅阶跃
    if (control_mode_ == ControlMode::kVelocity)
    {
      target_vel_[i] = MoveTowards(
          target_vel_[i],
          final_target_vel_[i],
          max_rpm_step);
    }
    // 相对运动模式保持原有逻辑不变
    else if (control_mode_ == ControlMode::kRelativeSeq)
    {
      if (std::abs(final_target_vel_[i]) > std::abs(target_vel_[i]))
      {
        target_vel_[i] = MoveTowards(
            target_vel_[i],
            final_target_vel_[i],
            max_rpm_step);
      }
      else
      {
        target_vel_[i] = final_target_vel_[i];
      }
    }
    else
    {
      target_vel_[i] = final_target_vel_[i];
    }
  }

  // 直行航向保持：把 IMU 航向误差换算成左右轮等大反向的目标转速修正。
  // 最终目标 final_target_vel_ 不变，因此轮对控制器仍按"直行"判定资格，
  // 同步误差则基于修正后的目标计算，两者方向一致、不会互相抵消。
  const bool heading_hold_engaged =
      kHeadingHoldEnabled &&
      control_mode_ == ControlMode::kVelocity &&
      straight_control_enabled_;
  const float heading_wz = heading_hold_.Update(heading_hold_engaged, imu_yaw_rel_, dt);
  float control_target[kNumWheels] = {target_vel_[0], target_vel_[1]};
  if (heading_wz != 0.0f && target_vel_[0] * target_vel_[1] > 0.0f)
  {
    const float mean_abs_rpm = 0.5f * (std::abs(target_vel_[0]) + std::abs(target_vel_[1]));
    const float trim_limit = kHeadingMaxTrimRatio * mean_abs_rpm;
    const float trim_rpm = std::clamp(heading_wz * kLy * kMmsToRpm, -trim_limit, trim_limit);
    control_target[0] -= trim_rpm;
    control_target[1] += trim_rpm;
  }

  // Compute both independent wheel-loop outputs before applying the paired
  // startup/synchronisation correction. This keeps the two PWM updates close
  // together and lets the pair controller add equal-and-opposite trim.
  float raw_pwm[kNumWheels] = {};
  float base_pwm[kNumWheels] = {};
  for (int i = 0; i < kNumWheels; ++i)
  {
    if (std::abs(control_target[i]) < 0.1f)
    {
      pid_vel_[i].Reset();
    }
    raw_pwm[i] = pid_vel_[i].Calculate(control_target[i], filtered_rpm_[i], dt);
    base_pwm[i] = CalculateBasePwm(i, control_target[i], raw_pwm[i]);
  }

  const bool wheel_sync_available =
      control_mode_ == ControlMode::kVelocity &&
      straight_control_enabled_ &&
      encs_[0]->IsInitialized() &&
      encs_[1]->IsInitialized();
  const WheelPairControlOutput pair_output = wheel_pair_controller_.Update(
      control_target[0],
      control_target[1],
      final_target_vel_[0],
      final_target_vel_[1],
      filtered_rpm_[0],
      filtered_rpm_[1],
      base_pwm[0],
      base_pwm[1],
      dt,
      wheel_sync_available);

  wheel_startup_fault_ = wheel_startup_fault_ || pair_output.startup_fault;
  const int16_t final_pwm[kNumWheels] = {
      (wheel_startup_fault_ || !motion_enabled_)
          ? static_cast<int16_t>(0)
          : static_cast<int16_t>(std::lround(pair_output.left_pwm)),
      (wheel_startup_fault_ || !motion_enabled_)
          ? static_cast<int16_t>(0)
          : static_cast<int16_t>(std::lround(pair_output.right_pwm))};
  last_sync_error_rpm_ = pair_output.sync_error_rpm;
  last_sync_correction_pwm_ = pair_output.sync_correction_pwm;
  wheel_sync_active_ = pair_output.sync_active;
  startup_sync_active_ = pair_output.startup_active;

  for (int i = 0; i < kNumWheels; ++i)
  {
    last_raw_pwm_[i] = raw_pwm[i];
    last_final_pwm_[i] = static_cast<float>(final_pwm[i]);
  }
  if (wheel_startup_fault_ || !motion_enabled_)
  {
    motors_[0]->Brake();
    motors_[1]->Brake();
  }
  else
  {
    motors_[0]->SetSpeed(final_pwm[0]);
    motors_[1]->SetSpeed(final_pwm[1]);
  }
}

void MotionController::GetVelocity(float *linear_x, float *linear_y, float *angular_z) const
{
  if (!linear_x || !linear_y || !angular_z)
    return;

  float v[kNumWheels];
  for (int i = 0; i < kNumWheels; ++i)
  {
    v[i] = filtered_rpm_[i] * kRpmToMms;
  }

  *linear_x = (v[0] + v[1]) / 2.0f;
  *linear_y = 0.0f;
  *angular_z = (-v[0] + v[1]) / (2.0f * kLy);
}

void MotionController::Stop()
{
  control_mode_ = ControlMode::kVelocity;
  straight_control_enabled_ = false;
  wheel_pair_controller_.Reset();
  heading_hold_.Reset();
  last_sync_error_rpm_ = 0.0f;
  last_sync_correction_pwm_ = 0.0f;
  wheel_sync_active_ = false;
  startup_sync_active_ = false;
  wheel_startup_fault_ = false;

  for (int i = 0; i < kNumWheels; ++i)
  {
    target_vel_[i] = 0.0f;
    final_target_vel_[i] = 0.0f;

    last_raw_pwm_[i] = 0.0f;
    last_final_pwm_[i] = 0.0f;

    pid_vel_[i].Reset();
    motors_[i]->Brake();
  }
  pid_pos_x_.Reset();
  pid_pos_y_.Reset();
  pid_pos_yaw_.Reset();

  seq_state_ = SeqState::kIdle;
}

void MotionController::SetVelocityPidGains(float kp, float ki, float kd)
{
  for (int i = 0; i < kNumWheels; ++i)
  {
    pid_vel_[i].SetGains(kp, ki, kd);
  }
  wheel_pair_controller_.Reset();
}

void MotionController::SetPositionPidGains(float kp, float ki, float kd)
{
  pid_pos_x_.SetGains(kp, ki, kd);
  pid_pos_y_.SetGains(kp, ki, kd);
  pid_pos_yaw_.SetGains(kp, ki, kd);
}

void MotionController::SetMotorTargetVelocity(MotorID motor_id, float target_mms)
{
  if (!motion_enabled_)
  {
    Stop();
    return;
  }
  if (!std::isfinite(target_mms))
  {
    Stop();
    return;
  }
  control_mode_ = ControlMode::kVelocity;
  straight_control_enabled_ = false;
  wheel_pair_controller_.Reset();
  const int motor_index = static_cast<int>(motor_id);
  if (motor_index < 0 || motor_index >= kNumWheels)
  {
    return;
  }
  const float new_target_rpm = target_mms * kMmsToRpm;
  if (final_target_vel_[motor_index] * new_target_rpm < 0.0f)
  {
    pid_vel_[motor_index].Reset();
  }
  final_target_vel_[motor_index] = new_target_rpm;
  if (IsNearZero(target_mms))
  {
    target_vel_[motor_index] = 0.0f;

    last_raw_pwm_[motor_index] = 0.0f;
    last_final_pwm_[motor_index] = 0.0f;

    pid_vel_[motor_index].Reset();
    motors_[motor_index]->Brake();
  }
}

void MotionController::SetAllMotorTargetsVelocity(float left_mms, float right_mms)
{
  if (!motion_enabled_)
  {
    Stop();
    return;
  }
  if (!std::isfinite(left_mms) || !std::isfinite(right_mms))
  {
    Stop();
    return;
  }
  if (IsNearZero(left_mms) && IsNearZero(right_mms))
  {
    Stop();
    return;
  }

  control_mode_ = ControlMode::kVelocity;
  straight_control_enabled_ =
      left_mms * right_mms > 0.0f &&
      std::abs(left_mms - right_mms) < 1.0f;
  const float new_target_rpm[kNumWheels] = {
      left_mms * kMmsToRpm,
      right_mms * kMmsToRpm};
  if (!std::isfinite(new_target_rpm[0]) ||
      !std::isfinite(new_target_rpm[1]))
  {
    Stop();
    return;
  }
  for (int i = 0; i < kNumWheels; ++i)
  {
    if (final_target_vel_[i] * new_target_rpm[i] < 0.0f)
    {
      pid_vel_[i].Reset();
      wheel_pair_controller_.Reset();
    }
    final_target_vel_[i] = new_target_rpm[i];
  }
}

float MotionController::GetMotorVelocity(MotorID motor_id) const
{
  const int motor_index = static_cast<int>(motor_id);
  if (motor_index < 0 || motor_index >= kNumWheels)
  {
    return 0.0f;
  }
  return filtered_rpm_[motor_index] * kRpmToMms;
}

void MotionController::GetAllMotorVelocities(float *left_mms, float *right_mms) const
{
  if (left_mms)
    *left_mms = filtered_rpm_[0] * kRpmToMms;
  if (right_mms)
    *right_mms = filtered_rpm_[1] * kRpmToMms;
}
void MotionController::GetMotorControlState(
    MotorID motor_id,
    float *target_rpm,
    float *actual_rpm,
    float *raw_pwm,
    float *final_pwm) const
{
  const int motor_index = static_cast<int>(motor_id);
  if (motor_index < 0 || motor_index >= kNumWheels)
  {
    return;
  }
  if (target_rpm)
  {
    *target_rpm = target_vel_[motor_index];
  }
  if (actual_rpm)
  {
    *actual_rpm = filtered_rpm_[motor_index];
  }
  if (raw_pwm)
  {
    *raw_pwm = last_raw_pwm_[motor_index];
  }
  if (final_pwm)
  {
    *final_pwm = last_final_pwm_[motor_index];
  }
}

void MotionController::GetHeadingHoldState(
    bool *active, float *error_rad, float *correction_wz) const
{
  if (active)
  {
    *active = heading_hold_.Active();
  }
  if (error_rad)
  {
    *error_rad = heading_hold_.LastErrorRad();
  }
  if (correction_wz)
  {
    *correction_wz = heading_hold_.LastCorrection();
  }
}

void MotionController::GetWheelSyncState(
    float *sync_error_rpm,
    float *sync_correction_pwm,
    bool *sync_active,
    bool *startup_active,
    bool *startup_fault) const
{
  if (sync_error_rpm)
  {
    *sync_error_rpm = last_sync_error_rpm_;
  }
  if (sync_correction_pwm)
  {
    *sync_correction_pwm = last_sync_correction_pwm_;
  }
  if (sync_active)
  {
    *sync_active = wheel_sync_active_;
  }
  if (startup_active)
  {
    *startup_active = startup_sync_active_;
  }
  if (startup_fault)
  {
    *startup_fault = wheel_startup_fault_;
  }
}
