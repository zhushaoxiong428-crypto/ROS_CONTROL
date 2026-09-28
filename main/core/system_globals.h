#pragma once

#include <stddef.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "msg/motion_msg.h"
#include "msg/pid_msg.h"

enum class WifiCommMode : uint8_t {
    kMavlinkUdp = 0,
    kMicroRos = 1,
    kMavlinkUart = 2,
};

struct MavlinkStatustextInfo {
    bool valid;
    uint8_t severity;
    uint32_t last_update_ms;
    char device_name[32];
    char ip[16];
    char text[51];
};

bool wifi_comm_mode_is_valid(WifiCommMode mode);
WifiCommMode wifi_comm_mode_next(WifiCommMode mode);
const char *wifi_comm_mode_to_runtime_value(WifiCommMode mode);
const char *wifi_comm_mode_to_display_name(WifiCommMode mode);

// ================= 跨任务运动接口 =================
// 所有通信任务都通过这些函数与 motion_task 交互，不直接调用 robot 对象，
// 以免与控制循环在另一核上并发修改控制器状态。

// 按顺序提交运动命令。队列满时丢弃最旧的一条，保证最新命令一定入队，
// 同时不会像单槽覆盖那样吞掉紧挨着的零速（互锁武装）命令。
bool motion_command_submit(const MotionMsg &cmd);

// 锁存急停：只有显式释放才会解除，普通运动命令不会清除它。
void motion_emergency_stop_set(bool active);
bool motion_emergency_stop_active();

// 请求 motion_task 在下一个控制周期内清零里程计。
void motion_request_odometry_reset();
bool motion_take_odometry_reset_request();

// ================= 全局标志位 =================
extern volatile bool g_motion_busy;
extern volatile uint32_t g_lidar_scan_sequence;
extern volatile WifiCommMode g_wifi_comm_mode;
extern bool g_microros_agent_auto_discovery;
extern char g_microros_agent_ip[16];
extern uint16_t g_microros_agent_port;
extern char g_device_name[32];
extern MavlinkStatustextInfo g_mavlink_statustext;

// ================= 消息队列句柄 =================
extern QueueHandle_t q_imu_state;
extern QueueHandle_t q_motion_state;
extern QueueHandle_t q_motor_debug_state;
extern QueueHandle_t q_ultrasonic_state;
extern QueueHandle_t q_lidar_state;   
extern QueueHandle_t q_gamepad_state;
extern QueueHandle_t q_temperature_state;
extern QueueHandle_t q_battery_state;
extern QueueHandle_t q_power_safety_state;


extern QueueHandle_t q_motion_cmd;
extern QueueHandle_t q_servo_cmd;
extern QueueHandle_t q_speedpid_cmd;
extern QueueHandle_t q_position_pid_cmd;

extern PidMsg g_speed_pid_state;
extern PidMsg g_position_pid_state;

// ================= 任务入口函数声明 =================
void motion_task(void *p);
void imu_task(void *p);
void ultrasonic_task(void *p);
void battery_task(void *p);
void peripheral_task(void *p);
void mavlink_udp_task(void *pvParameters);
void mavlink_uart_task(void *pvParameters);
void lidar_task(void *p);
void wifi_provision_task(void *pvParameters);
void gamepad_i2c_task(void *p);
void microros_task(void *p);
