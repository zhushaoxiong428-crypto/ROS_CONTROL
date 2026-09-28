#pragma once

// 直行航向保持（纯逻辑，可在主机上单元测试）。
//
// 编码器同步只能让两轮"转得一样快"，看不到轮径差、打滑以及同步 PI 死区
// 内残留的速度差，这些都会让车在直行时持续偏航。航向保持在直行开始时锁存
// IMU 航向作为参考，之后用 PI 把航向误差换算成角速度修正量 wz（rad/s，
// 逆时针为正），由运动控制器转换为左右轮等大反向的目标转速修正。
struct HeadingHoldConfig
{
  float kp = 1.5f;          // (rad/s) / rad
  float ki = 0.5f;          // (rad/s) / (rad·s)
  float max_wz = 0.35f;     // 修正量上限，rad/s
  float max_integral = 0.5f; // 积分项上限，rad·s
};

class HeadingHold
{
public:
  explicit HeadingHold(const HeadingHoldConfig &config = HeadingHoldConfig{})
      : config_(config) {}

  void Reset()
  {
    active_ = false;
    reference_rad_ = 0.0f;
    integral_ = 0.0f;
    last_error_rad_ = 0.0f;
    last_correction_ = 0.0f;
  }

  // engaged：当前是否为直行命令。首次进入时锁存参考航向；退出时复位。
  // 返回角速度修正量 wz（rad/s）。
  float Update(bool engaged, float yaw_rad, float dt);

  bool Active() const { return active_; }
  float ReferenceRad() const { return reference_rad_; }
  float LastErrorRad() const { return last_error_rad_; }
  float LastCorrection() const { return last_correction_; }

  static float WrapAngle(float angle_rad);

private:
  HeadingHoldConfig config_;
  bool active_ = false;
  float reference_rad_ = 0.0f;
  float integral_ = 0.0f;
  float last_error_rad_ = 0.0f;
  float last_correction_ = 0.0f;
};
