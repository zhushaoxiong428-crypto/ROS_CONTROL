#include "motion_config.h"
#include "system_globals.h"
#include "board.h" 
#include "msg/motion_msg.h"
#include "msg/pid_msg.h"
#include "msg/imu_msg.h"  
#include "command_watchdog.h"
#include "cycle_timing_stats.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <cmath>          

static const char *TAG = "MOTION_TASK";

static void log_timing(const CycleTimingStats::Summary& s) {
    // UART0 carries the XRCE-DDS stream in micro-ROS UART mode.
    if (system_globals().runtime.wifi_comm_mode == WifiCommMode::kMicroRosUart) {
        return;
    }
    ESP_LOGI(TAG, "control loop (%lu ms) n=%lu period mean=%.3f std=%.3f min=%.3f max=%.3f ms "
                  "|jitter|max=%.3f ms busy mean=%.3f max=%.3f ms overruns=%lu",
             static_cast<unsigned long>(leap_config::motion::kControlLoopPeriodMs),
             static_cast<unsigned long>(s.count),
             s.period_mean_us / 1000.0f, s.period_std_us / 1000.0f,
             s.period_min_us / 1000.0f, s.period_max_us / 1000.0f,
             s.max_abs_jitter_us / 1000.0f,
             s.busy_mean_us / 1000.0f, s.busy_max_us / 1000.0f,
             static_cast<unsigned long>(s.overruns));
}

void motion_task(void *p) {
    const int64_t nominal_period_us = leap_config::motion::kControlLoopPeriodMs * 1000;
    MotionMsg cmd_msg = {};
    MotionMsg incoming_msg = {};
    MotionMsg gamepad_cmd = {};
    MotionMsg active_cmd = {};
    MotionMsg status_msg = {};
    PidMsg speed_msg;
    PidMsg position_msg;
    
    ImuMsg imu_msg = {}; 

    int current_mode = 0;
    bool was_gamepad_override_active = false;
    CommandWatchdog nav_watchdog(leap_config::motion::kCommandTimeoutMs);
    CycleTimingStats timing(nominal_period_us, leap_config::motion::kControlLoopOverrunMarginUs);
    int64_t last_wake_us = esp_timer_get_time();
    TickType_t last_wake_tick = xTaskGetTickCount();
    // The first period and the one after each report (which includes the
    // UART log write) are not representative; leave them out.
    bool skip_timing_sample = true;

    auto apply_command = [&](const MotionMsg& command) {
        current_mode = command.control_mode;

        if (current_mode == 0) {
            robot.Drive(command.target_vx, command.target_vy, command.target_wz);
        } else if (current_mode == 1) {
            robot.MoveToPosition(command.target_x, command.target_y, command.target_yaw);
        } else if (current_mode == 2) {
            system_globals().flags.motion_busy = true;
            robot.MoveRelative(command.target_x, command.target_yaw);
        } else if (current_mode == 3) {
            robot.SetMotorTargetVelocity((MotorID)command.motor_id, command.target_motor_v);
        } else if (current_mode == 6) {
            robot.SetAllMotorTargetsVelocity(command.target_vx, command.target_vy);
        } else {
            ESP_LOGW(TAG, "Unsupported motion mode: %d", current_mode);
        }
    };
    
    while (1) {
        const int64_t wake_us = esp_timer_get_time();
        const int64_t period_us = wake_us - last_wake_us;
        last_wake_us = wake_us;
        // Wake-up latency makes real periods vary by about +/-1 ms around
        // 20 ms; velocity is encoder delta / dt, so use the measured period.
        const float dt = ClampLoopDtS(period_us, nominal_period_us);
        const uint32_t now_ms = pdTICKS_TO_MS(xTaskGetTickCount());
        xQueuePeek(system_globals().queues.imu_state, &imu_msg, 0);
                 
        // 1. 接收底层运动指令 (指令状态机)
        const bool nav_cmd_received = xQueueReceive(system_globals().queues.motion_cmd, &incoming_msg, 0) == pdTRUE;
        if (nav_cmd_received) {
            cmd_msg = incoming_msg;
            nav_watchdog.Feed(now_ms);
        }

        const bool gamepad_override_active = system_globals().flags.gamepad_override_active;
        if (gamepad_override_active &&
            xQueuePeek(system_globals().queues.gamepad_motion_cmd, &gamepad_cmd, 0) == pdTRUE) {
            active_cmd = gamepad_cmd;
            apply_command(active_cmd);
        } else if (nav_cmd_received || was_gamepad_override_active) {
            active_cmd = cmd_msg;
            apply_command(active_cmd);
        }
        was_gamepad_override_active = gamepad_override_active;

        // 指令超时：速度类指令的发送方停发后停车，并清掉旧指令，
        // 防止手柄退出接管时重新执行过期的速度。
        if (!gamepad_override_active && nav_watchdog.IsArmed() &&
            IsStreamingMotionMode(active_cmd.control_mode) &&
            !nav_watchdog.IsFresh(now_ms)) {
            ESP_LOGW(TAG, "No motion command for %lu ms, stopping",
                     static_cast<unsigned long>(leap_config::motion::kCommandTimeoutMs));
            nav_watchdog.Disarm();
            const uint8_t source = active_cmd.source;
            cmd_msg = MotionMsg{};
            cmd_msg.source = source;
            active_cmd = cmd_msg;
            apply_command(active_cmd);
        }

        // 2. 接收 PID 指令 (速度环)
        if (xQueueReceive(system_globals().queues.speedpid_cmd, &speed_msg, 0) == pdTRUE) {
            robot.SetVelocityPidGains(speed_msg.kp, speed_msg.ki, speed_msg.kd);
            system_globals().control.speed_pid = speed_msg;
            ESP_LOGI(TAG, "Speed PID updated: kp=%.3f, ki=%.3f, kd=%.3f",
                     speed_msg.kp, speed_msg.ki, speed_msg.kd);
        }
        
        // 3. 接收 PID 指令 (位置环)
        if (xQueueReceive(system_globals().queues.positionpid_cmd, &position_msg, 0) == pdTRUE) {
            robot.SetPositionPidGains(position_msg.kp, position_msg.ki, position_msg.kd);
            system_globals().control.position_pid = position_msg;
            ESP_LOGI(TAG, "Position PID updated: kp=%.3f, ki=%.3f, kd=%.3f",
                     position_msg.kp, position_msg.ki, position_msg.kd);
        }

        // 4. 执行物理控制循环
        if (system_globals().flags.emergency_stop) {
            robot.Stop();
            system_globals().flags.motion_busy = false;
        } else {
            float imu_yaw_rad = imu_msg.yaw * (M_PI / 180.0f); 
            robot.Update(dt, imu_yaw_rad);
            if (current_mode == 2 && !robot.IsBusy()) {
                system_globals().flags.motion_busy = false;
            }
        }
        
        // 5. 更新遥测状态并上报
        robot.GetVelocity(&status_msg.vx, &status_msg.vy, &status_msg.wz);
        robot.GetOdometry(&status_msg.x, &status_msg.y, &status_msg.yaw); 
        robot.GetOdometryQuaternion(&status_msg.qw,&status_msg.qx,&status_msg.qy,&status_msg.qz);

        robot.GetAllMotorVelocities(&status_msg.vel_left, &status_msg.vel_right);
        
        status_msg.control_mode = active_cmd.control_mode;
        status_msg.source = active_cmd.source;
        status_msg.target_vx = active_cmd.target_vx;
        
        xQueueOverwrite(system_globals().queues.motion_state, &status_msg);

        // 6. 控制周期统计
        if (!skip_timing_sample) {
            timing.Add(period_us, esp_timer_get_time() - wake_us);
        }
        skip_timing_sample = false;
        if (timing.count() >= leap_config::motion::kTimingReportCycles) {
            log_timing(timing.Get());
            timing.Reset();
            skip_timing_sample = true;
        }

        vTaskDelayUntil(&last_wake_tick, pdMS_TO_TICKS(leap_config::motion::kControlLoopPeriodMs));
    }
}
