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

#include <cmath>

static const char *TAG = "MOTION_TASK";
// 这里的控制周期用的是相对延时，所以实际的控制周期=程序执行+20ms，与之后的编码器测速、里程计积分以及PID运算所用的固定20ms不一致。
// 现在要改成绝对延时，将实际的控制周期固定在20ms，还需测试实际控制周期是否接近20ms
void motion_task(void *p)
{
    const float dt = 0.02f;

    // 新加------------------
    const TickType_t constrol_period = pdMS_TO_TICKS(20);
    TickType_t last_wake_time = xTaskGetTickCount();
    // uint64_t last_cycle_us = 0;
    // uint64_t cycle_sum_us = 0;
    // uint64_t cycle_min_us = 0;
    // uint64_t cycle_max_us = 0;
    // uint64_t cycle_count = 0;
    //----------------------

    MotionMsg cmd_msg = {};
    MotionMsg incoming_msg = {};
    MotionMsg status_msg = {};
    PidMsg speed_msg;
    PidMsg position_msg;

    ImuMsg imu_msg = {};
    BatteryMsg battery_msg = {};
    BatteryMotionInterlock battery_interlock;

    int current_mode = 0;
    // 新增-------------
    const TickType_t velocity_cmd_timeout = pdMS_TO_TICKS(500); // 超时时间
    TickType_t last_velocity_cmd_tick = 0;                      // 最后一次收到速度命令的时间
    bool velocity_cmd_seen = false;                             // 系统启动后到底有没有真正收到过速度命令
    bool velocity_watchdog_triggered = false;                   // watchdog是否已经触发
    uint16_t motor_debug_control_seq = 0;
    bool interlock_log_initialized = false;
    BatteryInterlockReason last_logged_interlock_reason = BatteryInterlockReason::kNoSample;
    bool last_logged_motion_allowed = false;
    bool reject_log_initialized = false;
    uint32_t last_reject_log_ms = 0;
    //-----------------

    auto reset_blocked_motion_state = [&]() {
        cmd_msg = {};
        current_mode = 0;
        velocity_cmd_seen = false;
        velocity_watchdog_triggered = false;
        g_motion_busy = false;
    };

    while (1)
    {

        //
        // uint64_t now_us = esp_timer_get_time();
        // if (last_cycle_us != 0)
        // {
        //     uint64_t cycle_us = now_us - last_cycle_us;

        //     cycle_sum_us += cycle_us;
        //     if (cycle_count == 0 || cycle_us < cycle_min_us)
        //     {
        //         cycle_min_us = cycle_us;
        //     }
        //     if (cycle_us > cycle_max_us)
        //     {
        //         cycle_max_us = cycle_us;
        //     }
        //     cycle_count++;
        //     if (cycle_count >= 50)
        //     {
        //         float avg_ms =
        //             static_cast<float>(cycle_sum_us) /
        //             static_cast<float>(cycle_count) /
        //             1000.0f;
        //         float min_ms =
        //             static_cast<float>(cycle_min_us) /
        //             1000.0f;
        //         float max_ms =
        //             static_cast<float>(cycle_max_us) /
        //             1000.0f;
        //         ESP_LOGI(
        //             TAG,
        //             "Control period: avg=%.3f ms, min=%.3f ms, max=%.3f ms",
        //             avg_ms,
        //             min_ms,
        //             max_ms);
        //         cycle_sum_us = 0;
        //         cycle_min_us = 0;
        //         cycle_max_us = 0;
        //         cycle_count = 0;
        //     }
        // }
        // last_cycle_us = now_us;
        //

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

        // 1. 接收底层运动指令 (指令状态机)
        if (xQueueReceive(q_motion_cmd, &incoming_msg, 0) == pdTRUE)
        {
            const BatteryCommandDecision decision = battery_interlock.EvaluateCommand(
                IsExplicitAllStopCommand(incoming_msg));
            if (decision == BatteryCommandDecision::kStopOnly ||
                decision == BatteryCommandDecision::kStopAndArm)
            {
                robot.SetMotionEnabled(
                    decision == BatteryCommandDecision::kStopAndArm);
                robot.Stop();
                reset_blocked_motion_state();
            }
            else if (decision == BatteryCommandDecision::kRejectMotion)
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
            }
            else
            {
                robot.SetMotionEnabled(true);
                cmd_msg = incoming_msg;
                current_mode = cmd_msg.control_mode; // 记录最新模式

                // 处理离散单次触发模式
                if (current_mode == 0)
                {
                    last_velocity_cmd_tick = xTaskGetTickCount();
                    velocity_cmd_seen = true;
                    velocity_watchdog_triggered = false;
                    robot.Drive(cmd_msg.target_vx, cmd_msg.target_vy, cmd_msg.target_wz);
                }
                else if (current_mode == 1)
                {
                    robot.MoveToPosition(cmd_msg.target_x, cmd_msg.target_y, cmd_msg.target_yaw);
                }
                else if (current_mode == 2)
                {
                    g_motion_busy = true;
                    robot.MoveRelative(cmd_msg.target_x, cmd_msg.target_yaw);
                }
                else if (current_mode == 3)
                {
                    robot.SetMotorTargetVelocity((MotorID)cmd_msg.motor_id, cmd_msg.target_motor_v);
                }
                else if (current_mode == 6)
                {
                    robot.SetAllMotorTargetsVelocity(
                        cmd_msg.target_vx,
                        cmd_msg.target_vy);
                }
                else
                {
                    ESP_LOGW(TAG, "Unsupported motion mode: %d; stopping", current_mode);
                    robot.Stop();
                    reset_blocked_motion_state();
                }
            }
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
        if (xQueueReceive(q_postionpid_cmd, &position_msg, 0) == pdTRUE)
        {
            robot.SetPositionPidGains(position_msg.kp, position_msg.ki, position_msg.kd);
            g_position_pid_state = position_msg;
            ESP_LOGI(TAG, "Position PID updated: kp=%.3f, ki=%.3f, kd=%.3f",
                     position_msg.kp, position_msg.ki, position_msg.kd);
        }
        // 新增 检查持续速度命令是否超时
        bool velocity_timeout = false;
        if (current_mode == 0 && velocity_cmd_seen)
        {
            const TickType_t now_tick = xTaskGetTickCount();
            if ((now_tick - last_velocity_cmd_tick) > velocity_cmd_timeout)
            {
                velocity_timeout = true;
            }
        }

        // 4. 执行物理控制循环
        // if (g_emergency_stop)

        // {
        //     robot.Stop();
        //     g_motion_busy = false;
        // }
        // else if(velocity_timeout)//新增一个判断分支
        // {
        //     robot.Stop();
        //     if(!velocity_watchdog_triggered)
        //     {
        //         ESP_LOGW(TAG,"Velocity command timeout, robot stopped");
        //         velocity_watchdog_triggered=true;
        //     }
        // }
        // else
        // {
        //     float imu_yaw_rad = imu_msg.yaw * (M_PI / 180.0f);
        //     robot.Update(dt, imu_yaw_rad);
        //     if (current_mode == 2 && !robot.IsBusy())
        //     {
        //         g_motion_busy = false;
        //     }
        // }
        float imu_yaw_rad = imu_msg.yaw * (M_PI / 180.f);
        if (!battery_interlock.MotionAllowed())
        {
            robot.SetMotionEnabled(false);
            robot.Update(dt, imu_yaw_rad);
            g_motion_busy = false;
        }
        else if (g_emergency_stop)
        {
            robot.Stop();

            // 停车仍然更新编码器和里程状态
            robot.Update(dt, imu_yaw_rad);

            g_motion_busy = false;
        }
        else if (velocity_timeout)
        {
            robot.Stop();
            // watchdog 停车时仍继续更新编码器
            // 防止 last_count_长时间不更新
            robot.Update(dt, imu_yaw_rad);
            if (!velocity_watchdog_triggered)
            {
                ESP_LOGW(TAG, "Velocity command timeout, robot stopped");
                velocity_watchdog_triggered = true;
            }
        }
        else
        {
            robot.Update(dt, imu_yaw_rad);
            if (current_mode == 2 && !robot.IsBusy())
            {
                g_motion_busy = false;
            }
        }

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

        vTaskDelayUntil(&last_wake_time, constrol_period);
        // vTaskDelay(pdMS_TO_TICKS(20));
    }
}
