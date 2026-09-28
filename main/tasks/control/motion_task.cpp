#include "system_globals.h"
#include "board.h"
#include "msg/motion_msg.h"
#include "msg/pid_msg.h"
#include "msg/imu_msg.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "msg/motor_debug_msg.h"
#include "msg/battery_msg.h"
#include "msg/power_safety_msg.h"
#include "control/battery_motion_interlock.h"
#include "control/motion_command_safety.h"
#include "control/emergency_stop_gate.h"

#include <cmath>

static const char *TAG = "MOTION_TASK";

// 控制周期用 vTaskDelayUntil 固定为 20 ms，与编码器测速、里程计积分和
// PID 中使用的 dt 一致。
static constexpr float kControlDt = 0.02f;
static constexpr TickType_t kControlPeriod = pdMS_TO_TICKS(20);
// 持续速度命令（mode 0）超过该时间未更新则停车。
static constexpr TickType_t kVelocityCmdTimeout = pdMS_TO_TICKS(500);

void motion_task(void *p)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    MotionMsg cmd_msg = {};
    MotionMsg incoming_msg = {};
    MotionMsg status_msg = {};
    PidMsg speed_msg;
    PidMsg position_msg;

    ImuMsg imu_msg = {};
    BatteryMsg battery_msg = {};
    BatteryMotionInterlock battery_interlock;

    int current_mode = 0;
    TickType_t last_velocity_cmd_tick = 0;
    bool velocity_cmd_seen = false;           // 是否收到过持续速度命令
    bool velocity_watchdog_triggered = false; // 看门狗是否已触发（只打印一次）
    uint16_t motor_debug_control_seq = 0;
    bool interlock_log_initialized = false;
    BatteryInterlockReason last_logged_interlock_reason = BatteryInterlockReason::kNoSample;
    bool last_logged_motion_allowed = false;
    bool reject_log_initialized = false;
    uint32_t last_reject_log_ms = 0;
    bool estop_logged_active = false;
    EmergencyStopGate estop_gate;

    auto reset_blocked_motion_state = [&]() {
        cmd_msg = {};
        current_mode = 0;
        velocity_cmd_seen = false;
        velocity_watchdog_triggered = false;
        g_motion_busy = false;
    };

    // 处理一条运动命令：先经过电池互锁，再分发到对应控制模式。
    auto handle_motion_command = [&](const MotionMsg &msg, uint32_t now_ms) {
        const BatteryCommandDecision decision =
            battery_interlock.EvaluateCommand(IsExplicitAllStopCommand(msg));
        if (decision == BatteryCommandDecision::kStopOnly ||
            decision == BatteryCommandDecision::kStopAndArm)
        {
            robot.SetMotionEnabled(decision == BatteryCommandDecision::kStopAndArm);
            robot.Stop();
            reset_blocked_motion_state();
            return;
        }
        if (decision == BatteryCommandDecision::kRejectMotion)
        {
            robot.SetMotionEnabled(false);
            reset_blocked_motion_state();
            if (!reject_log_initialized || now_ms - last_reject_log_ms >= 1000)
            {
                const BatteryMotionInterlockStatus status = battery_interlock.Status(now_ms);
                ESP_LOGW(
                    TAG,
                    "Motion command rejected by battery interlock: reason=%s voltage=%.3fV seq=%lu age=%lums",
                    BatteryMotionInterlock::ReasonName(status.reason),
                    status.voltage_v,
                    static_cast<unsigned long>(status.sample_seq),
                    static_cast<unsigned long>(status.sample_age_ms));
                reject_log_initialized = true;
                last_reject_log_ms = now_ms;
            }
            return;
        }

        robot.SetMotionEnabled(true);
        cmd_msg = msg;
        current_mode = cmd_msg.control_mode;
        switch (current_mode)
        {
        case 0:
            last_velocity_cmd_tick = xTaskGetTickCount();
            velocity_cmd_seen = true;
            velocity_watchdog_triggered = false;
            robot.Drive(cmd_msg.target_vx, cmd_msg.target_vy, cmd_msg.target_wz);
            break;
        case 1:
            robot.MoveToPosition(cmd_msg.target_x, cmd_msg.target_y, cmd_msg.target_yaw);
            break;
        case 2:
            g_motion_busy = true;
            robot.MoveRelative(cmd_msg.target_x, cmd_msg.target_yaw);
            break;
        case 3:
            robot.SetMotorTargetVelocity(static_cast<MotorID>(cmd_msg.motor_id), cmd_msg.target_motor_v);
            break;
        case 6:
            robot.SetAllMotorTargetsVelocity(cmd_msg.target_vx, cmd_msg.target_vy);
            break;
        default:
            ESP_LOGW(TAG, "Unsupported motion mode: %d; stopping", current_mode);
            robot.Stop();
            reset_blocked_motion_state();
            break;
        }
    };

    while (1)
    {
        xQueuePeek(q_imu_state, &imu_msg, 0);

        if (q_battery_state != nullptr &&
            xQueuePeek(q_battery_state, &battery_msg, 0) == pdTRUE)
        {
            battery_interlock.ObserveSample(
                battery_msg.sample_seq,
                battery_msg.sample_time_ms,
                battery_msg.valid,
                battery_msg.voltage_v);
            if (battery_msg.valid)
            {
                robot.SetBatteryVoltage(battery_msg.voltage_v);
            }
        }
        // Read "now" after peeking the cross-task sample. Otherwise the
        // battery task could stamp a new sample between these two operations,
        // making unsigned age arithmetic look almost 2^32 ms old.
        const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
        battery_interlock.Poll(now_ms);

        // 里程计复位由通信任务请求，只在本任务内执行，避免与 Update() 并发。
        if (motion_take_odometry_reset_request())
        {
            robot.ResetOdometry();
            ESP_LOGI(TAG, "Odometry reset");
        }

        // 1. 按到达顺序处理本周期内的所有运动命令。急停期间命令一律丢弃。
        const bool estop_active = motion_emergency_stop_active();
        estop_gate.Observe(estop_active);
        while (xQueueReceive(q_motion_cmd, &incoming_msg, 0) == pdTRUE)
        {
            const bool was_awaiting_zero = estop_gate.AwaitingZeroCommand();
            if (!estop_gate.Admit(IsExplicitAllStopCommand(incoming_msg)))
            {
                continue;
            }
            if (was_awaiting_zero)
            {
                ESP_LOGI(TAG, "Zero command received after emergency stop; motion re-armed");
            }
            handle_motion_command(incoming_msg, now_ms);
        }

        if (!battery_interlock.MotionAllowed())
        {
            robot.SetMotionEnabled(false);
            reset_blocked_motion_state();
        }

        // 2. 接收 PID 指令 (速度环)
        if (xQueueReceive(q_speedpid_cmd, &speed_msg, 0) == pdTRUE)
        {
            robot.SetVelocityPidGains(speed_msg.kp, speed_msg.ki, speed_msg.kd);
            g_speed_pid_state = speed_msg;
            ESP_LOGI(TAG, "Speed PID updated: kp=%.3f, ki=%.3f, kd=%.3f",
                     speed_msg.kp, speed_msg.ki, speed_msg.kd);
        }

        // 3. 接收 PID 指令 (位置环)
        if (xQueueReceive(q_position_pid_cmd, &position_msg, 0) == pdTRUE)
        {
            robot.SetPositionPidGains(position_msg.kp, position_msg.ki, position_msg.kd);
            g_position_pid_state = position_msg;
            ESP_LOGI(TAG, "Position PID updated: kp=%.3f, ki=%.3f, kd=%.3f",
                     position_msg.kp, position_msg.ki, position_msg.kd);
        }

        // 检查持续速度命令是否超时
        bool velocity_timeout = false;
        if (current_mode == 0 && velocity_cmd_seen)
        {
            velocity_timeout = (xTaskGetTickCount() - last_velocity_cmd_tick) > kVelocityCmdTimeout;
        }

        // 4. 执行物理控制循环。停车分支也要调用 Update()，
        //    保证编码器计数和里程计持续更新。
        const float imu_yaw_rad = imu_msg.yaw * (M_PI / 180.f);
        if (!battery_interlock.MotionAllowed())
        {
            robot.SetMotionEnabled(false);
            robot.Update(kControlDt, imu_yaw_rad);
            g_motion_busy = false;
        }
        else if (estop_active)
        {
            robot.Stop();
            reset_blocked_motion_state();
            robot.Update(kControlDt, imu_yaw_rad);
            if (!estop_logged_active)
            {
                ESP_LOGW(TAG, "Emergency stop active: motors braked, commands ignored");
            }
        }
        else if (velocity_timeout)
        {
            robot.Stop();
            robot.Update(kControlDt, imu_yaw_rad);
            if (!velocity_watchdog_triggered)
            {
                ESP_LOGW(TAG, "Velocity command timeout, robot stopped");
                velocity_watchdog_triggered = true;
            }
        }
        else
        {
            robot.Update(kControlDt, imu_yaw_rad);
            if (current_mode == 2 && !robot.IsBusy())
            {
                g_motion_busy = false;
            }
        }
        if (estop_logged_active && !estop_active)
        {
            ESP_LOGI(TAG, "Emergency stop released; send a zero velocity command to re-arm");
        }
        estop_logged_active = estop_active;

        // 5. 更新遥测状态并上报
        robot.GetVelocity(&status_msg.vx, &status_msg.vy, &status_msg.wz);
        robot.GetOdometry(&status_msg.x, &status_msg.y, &status_msg.yaw);
        robot.GetOdometryQuaternion(&status_msg.qw, &status_msg.qx, &status_msg.qy, &status_msg.qz);

        robot.GetAllMotorVelocities(&status_msg.vel_left, &status_msg.vel_right);
        MotorDebugMsg motor_debug = {};

        robot.GetMotorControlState(
            MotorID::kLeft,
            &motor_debug.left_target_rpm,
            &motor_debug.left_actual_rpm,
            &motor_debug.left_raw_pwm,
            &motor_debug.left_final_pwm);

        robot.GetMotorControlState(
            MotorID::kRight,
            &motor_debug.right_target_rpm,
            &motor_debug.right_actual_rpm,
            &motor_debug.right_raw_pwm,
            &motor_debug.right_final_pwm);

        robot.GetWheelSyncState(
            &motor_debug.sync_error_rpm,
            &motor_debug.sync_correction_pwm,
            &motor_debug.sync_active,
            &motor_debug.startup_active,
            &motor_debug.startup_fault);

        motor_debug.control_seq = ++motor_debug_control_seq;
        motor_debug.control_time_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);

        if (q_motor_debug_state != nullptr)
        {
            xQueueOverwrite(q_motor_debug_state, &motor_debug);
        }
        status_msg.control_mode = cmd_msg.control_mode;
        status_msg.source = cmd_msg.source;
        status_msg.target_vx = cmd_msg.target_vx;

        xQueueOverwrite(q_motion_state, &status_msg);

        const BatteryMotionInterlockStatus interlock_status =
            battery_interlock.Status(now_ms);
        if (q_power_safety_state != nullptr)
        {
            PowerSafetyMsg safety = {};
            safety.reason = static_cast<uint8_t>(interlock_status.reason);
            safety.last_trip_reason = static_cast<uint8_t>(interlock_status.last_trip_reason);
            safety.has_sample = interlock_status.has_sample;
            safety.adc_valid = interlock_status.adc_valid;
            safety.sample_fresh = interlock_status.sample_fresh;
            safety.voltage_ready = interlock_status.voltage_ready;
            safety.armed = interlock_status.armed;
            safety.motion_allowed = interlock_status.motion_allowed;
            safety.voltage_v = interlock_status.voltage_v;
            safety.recovery_sample_count = interlock_status.recovery_sample_count;
            safety.sample_seq = interlock_status.sample_seq;
            safety.sample_age_ms = interlock_status.sample_age_ms;
            safety.trip_count = interlock_status.trip_count;
            xQueueOverwrite(q_power_safety_state, &safety);
        }
        if (!interlock_log_initialized ||
            interlock_status.reason != last_logged_interlock_reason ||
            interlock_status.motion_allowed != last_logged_motion_allowed)
        {
            ESP_LOGI(
                TAG,
                "Battery motion interlock: reason=%s voltage=%.3fV adc_valid=%d fresh=%d ready=%d armed=%d allowed=%d recovery=%u/%u seq=%lu age=%lums trips=%lu",
                BatteryMotionInterlock::ReasonName(interlock_status.reason),
                interlock_status.voltage_v,
                interlock_status.adc_valid ? 1 : 0,
                interlock_status.sample_fresh ? 1 : 0,
                interlock_status.voltage_ready ? 1 : 0,
                interlock_status.armed ? 1 : 0,
                interlock_status.motion_allowed ? 1 : 0,
                static_cast<unsigned>(interlock_status.recovery_sample_count),
                static_cast<unsigned>(BatteryMotionInterlockConfig{}.recovery_samples),
                static_cast<unsigned long>(interlock_status.sample_seq),
                static_cast<unsigned long>(interlock_status.sample_age_ms),
                static_cast<unsigned long>(interlock_status.trip_count));
            interlock_log_initialized = true;
            last_logged_interlock_reason = interlock_status.reason;
            last_logged_motion_allowed = interlock_status.motion_allowed;
        }

        vTaskDelayUntil(&last_wake_time, kControlPeriod);
    }
}
