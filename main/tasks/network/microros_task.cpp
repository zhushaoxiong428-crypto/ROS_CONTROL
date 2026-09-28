#include "system_globals.h"
#include "wifi_app.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "msg/battery_msg.h"
#include "msg/power_safety_msg.h"
#include "msg/imu_msg.h"
#include "msg/lidar_msg.h"
#include "msg/motion_msg.h"
#include "msg/motor_debug_msg.h"
#include "msg/ultrasonic_msg.h"
#include "pid_config.h"
#include "agent_ping_health.h"
#include "laser_scan_downsampler.h"
#include "publish_probe_stats.h"
#include "std_msgs/msg/bool.h"
#include "std_msgs/msg/float32_multi_array.h"

#include "geometry_msgs/msg/twist.h"
#include "leap_interfaces/srv/get_speed_pid.h"
#include "leap_interfaces/srv/set_speed_pid.h"
#include "nav_msgs/msg/odometry.h"
#include "rcl/rcl.h"
#include "rcl/node_options.h"
#include "rcl/time.h"
#include "rcl/timer.h"
#include "rcl/wait.h"
#include "rclc/executor.h"
#include "rmw/qos_profiles.h"
#include "rmw_microros/custom_transport.h"
#include "rmw_microros/ping.h"
#include "rmw_microros/rmw_microros.h"
#include "rmw_microros/time_sync.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include "rosidl_runtime_c/string_functions.h"
#include "sensor_msgs/msg/battery_state.h"
#include "sensor_msgs/msg/imu.h"
#include "sensor_msgs/msg/laser_scan.h"
#include "sensor_msgs/msg/range.h"
#include "uxr/client/profile/discovery/discovery.h"
#include "uxr/client/profile/transport/ip/ip.h"
#include "uxr/client/profile/transport/custom/custom_transport.h"

static const char *TAG = "MICROROS";
static constexpr size_t kLaserScanSourcePointCount = 360;
static constexpr size_t kLaserScanPointCount = CONFIG_MICROROS_SCAN_POINT_COUNT;
static_assert(kLaserScanPointCount > 0 &&
              kLaserScanPointCount <= kLaserScanSourcePointCount);
static constexpr double kDegToRad = 0.017453292519943295;
static constexpr double kFullCircleRad = 6.283185307179586;
static constexpr double kGravity = 9.80665;
static constexpr uint32_t kAgentCheckIntervalMs = 2000;
static constexpr TickType_t kAgentCheckIntervalTicks = pdMS_TO_TICKS(kAgentCheckIntervalMs);
// Runtime health checks execute in the same task as spin/publish because the
// XRCE session is not used concurrently. Bound each check well below the
// 500 ms velocity-command watchdog. Session recovery deliberately uses a
// longer confirmation window because it is not the motion safety mechanism.
static constexpr int kAgentPingTimeoutMs = 200;
static constexpr uint8_t kAgentPingAttempts = 1;
static constexpr uint32_t kAgentPingFailureWindowMs = 12000;
static constexpr uint32_t kAgentPingFailureWindowTicks =
    pdMS_TO_TICKS(kAgentPingFailureWindowMs);
static constexpr uint8_t kAgentPingMinimumFailures = 3;
static constexpr const char *kAgentDiscoveryMulticastIp = "239.255.0.2";
// Keep XRCE discovery away from DDS/RTPS port 7400. A dedicated port makes
// unicast fallback deterministic when multicast is blocked by a phone hotspot.
static constexpr uint16_t kAgentDiscoveryPort = 8889;
static constexpr uint32_t kAgentMulticastDiscoveryAttempts = 1;
static constexpr int kAgentMulticastDiscoveryPeriodMs = 500;
static constexpr uint32_t kAgentUnicastDiscoveryAttempts = 2;
static constexpr int kAgentUnicastDiscoveryPeriodMs = 1000;
static constexpr size_t kMaxAgentUnicastCandidates = 1024;

#if defined(CONFIG_MICROROS_TELEMETRY_BEST_EFFORT)
static constexpr const char *kTelemetryQosMode = "best_effort";
#else
static constexpr const char *kTelemetryQosMode = "default";
#endif

#if defined(CONFIG_MICROROS_DISABLE_SCAN_AB_TEST)
static constexpr const char *kScanPublishMode = "disabled_ab_test";
#else
static constexpr const char *kScanPublishMode = "enabled";
#endif

#ifndef CONFIG_MICRO_ROS_LOCAL_PORT
#define CONFIG_MICRO_ROS_LOCAL_PORT "8888"
#endif

struct UdpTransportContext
{
    int fd;
    sockaddr_in remote;
    uint16_t local_port;
};

struct AgentDiscoveryResult
{
    esp_netif_ip_info_t sta_ip_info;
    bool has_sta_ip_info;
    char ip[16];
    uint16_t port;
    bool found;
};

static UdpTransportContext s_udp_ctx = {};
static geometry_msgs__msg__Twist s_cmd_vel_msg = {};
static nav_msgs__msg__Odometry s_odom_msg = {};
static std_msgs__msg__Float32MultiArray s_motor_debug_msg = {};
static float s_motor_debug_data[19] = {};
static uint16_t s_motor_debug_publish_seq = 0;
static sensor_msgs__msg__Imu s_imu_msg = {};
static sensor_msgs__msg__LaserScan s_scan_msg = {};
static sensor_msgs__msg__BatteryState s_battery_msg = {};
static sensor_msgs__msg__Range s_ultrasonic_msg = {};
static rcl_publisher_t s_odom_publisher = {};
static rcl_publisher_t s_imu_publisher = {};
static rcl_publisher_t s_motor_debug_publisher = {};
static rcl_publisher_t s_scan_publisher = {};
static rcl_publisher_t s_battery_publisher = {};
static rcl_publisher_t s_ultrasonic_publisher = {};
static rcl_subscription_t s_cmd_vel_subscriber = {};
static rcl_subscription_t s_estop_subscriber = {};
static std_msgs__msg__Bool s_estop_msg = {};
static rcl_service_t s_set_speed_pid_service = {};
static rcl_service_t s_get_speed_pid_service = {};
static rclc_executor_t s_service_executor = {};
static leap_interfaces__srv__SetSpeedPid_Request s_set_speed_pid_req = {};
static leap_interfaces__srv__SetSpeedPid_Response s_set_speed_pid_res = {};
static leap_interfaces__srv__GetSpeedPid_Request s_get_speed_pid_req = {};
static leap_interfaces__srv__GetSpeedPid_Response s_get_speed_pid_res = {};
static rcl_init_options_t s_init_options = {};
static rcl_context_t s_context = {};
static rcl_allocator_t s_allocator = {};
static rcl_node_t s_node = {};
static rcl_timer_t s_publish_timer = {};
static rcl_wait_set_t s_wait_set = {};
static rcl_clock_t s_clock = {};
static bool s_ros_created = false;
static bool s_strings_initialized = false;
static bool s_scan_ranges_initialized = false;
static bool s_init_options_initialized = false;
static bool s_context_initialized = false;
static bool s_node_initialized = false;
static bool s_odom_publisher_initialized = false;
static bool s_imu_publisher_initialized = false;
static bool s_motor_debug_publisher_initialized = false;
static bool s_scan_publisher_initialized = false;
static bool s_battery_msg_initialized = false;
static bool s_battery_publisher_initialized = false;
static bool s_ultrasonic_publisher_initialized = false;
static bool s_cmd_vel_subscriber_initialized = false;
static bool s_estop_subscriber_initialized = false;
static bool s_set_speed_pid_service_initialized = false;
static bool s_get_speed_pid_service_initialized = false;
static bool s_service_executor_initialized = false;
static bool s_clock_initialized = false;
static bool s_timer_initialized = false;
static bool s_wait_set_initialized = false;
static uint32_t s_publish_tick = 0;
static TickType_t s_last_agent_check_tick = 0;
static bool s_ros_session_error = false;
static AgentPingHealth s_agent_ping_health = {};

enum class ProbeStage : uint8_t
{
    kOdom,
    kImu,
    kMotorDebug,
    kScan,
    kBattery,
    kUltrasonic,
    kPing,
    kSetupPing,
    kWait,
    kCommand,
    kTimer,
    kService,
    kSpin,
    kLoopGap,
    kEpoch,
    kReport,
    kCount,
};

static constexpr int64_t kProbeReportIntervalUs = 10000000;
static ProbeStats s_probe_stats[static_cast<size_t>(ProbeStage::kCount)] = {};
static int64_t s_probe_report_start_us = 0;
static int64_t s_last_active_loop_us = 0;
static uint32_t s_probe_session_generation = 0;

static ProbeStats &probe_stats(ProbeStage stage)
{
    return s_probe_stats[static_cast<size_t>(stage)];
}

static void observe_probe(ProbeStage stage, int64_t start_us, bool success = true)
{
    const int64_t end_us = esp_timer_get_time();
    const int64_t elapsed_us = end_us - start_us;
    if (elapsed_us >= 0)
    {
        probe_stats(stage).Observe(
            static_cast<uint64_t>(elapsed_us), success,
            static_cast<uint32_t>(end_us / 1000), s_motor_debug_publish_seq);
    }
}

struct ProbeScope
{
    explicit ProbeScope(ProbeStage probe_stage)
        : stage(probe_stage), start_us(esp_timer_get_time()) {}

    ~ProbeScope()
    {
        observe_probe(stage, start_us);
    }

    ProbeStage stage;
    int64_t start_us;
};

static void report_probe_if_due(bool force = false)
{
    const int64_t now_us = esp_timer_get_time();
    if (s_probe_report_start_us == 0)
    {
        s_probe_report_start_us = now_us;
        return;
    }
    if (!force && now_us - s_probe_report_start_us < kProbeReportIntervalUs)
    {
        return;
    }

    const int64_t report_start_us = now_us;
    const auto maximum = [](ProbeStage stage) -> unsigned long long {
        return static_cast<unsigned long long>(probe_stats(stage).max_us);
    };
    ESP_LOGI(TAG,
             "MICROROS_TIMING window_ms=%llu session=%u timer_count=%llu debug_count=%llu ping_count=%llu "
             "loop_max_us=%llu spin_max_us=%llu wait_max_us=%llu cmd_max_us=%llu timer_max_us=%llu "
             "svc_max_us=%llu epoch_max_us=%llu ping_max_us=%llu setup_ping_max_us=%llu report_max_us=%llu "
             "loop_gt50=%llu spin_gt50=%llu wait_gt50=%llu timer_gt50=%llu svc_gt50=%llu ping_gt50=%llu "
             "ping_fail=%llu ping_fail_streak=%u ping_fail_age_ms=%u",
             static_cast<unsigned long long>((now_us - s_probe_report_start_us) / 1000),
             static_cast<unsigned>(s_probe_session_generation),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kTimer).count),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kMotorDebug).count),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kPing).count),
             maximum(ProbeStage::kLoopGap), maximum(ProbeStage::kSpin),
             maximum(ProbeStage::kWait), maximum(ProbeStage::kCommand),
             maximum(ProbeStage::kTimer), maximum(ProbeStage::kService),
             maximum(ProbeStage::kEpoch), maximum(ProbeStage::kPing),
             maximum(ProbeStage::kSetupPing), maximum(ProbeStage::kReport),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kLoopGap).over_50ms),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kSpin).over_50ms),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kWait).over_50ms),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kTimer).over_50ms),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kService).over_50ms),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kPing).over_50ms),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kPing).fail),
             static_cast<unsigned>(s_agent_ping_health.consecutive_failures),
             static_cast<unsigned>(pdTICKS_TO_MS(s_agent_ping_health.FailureAgeTicks(
                 static_cast<uint32_t>(xTaskGetTickCount())))));
    ESP_LOGI(TAG,
             "MICROROS_TIMING_PUB n/max_us/gt50/fail "
             "odom=%llu/%llu/%llu/%llu imu=%llu/%llu/%llu/%llu debug=%llu/%llu/%llu/%llu "
             "scan=%llu/%llu/%llu/%llu battery=%llu/%llu/%llu/%llu ultra=%llu/%llu/%llu/%llu",
             static_cast<unsigned long long>(probe_stats(ProbeStage::kOdom).count),
             maximum(ProbeStage::kOdom),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kOdom).over_50ms),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kOdom).fail),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kImu).count),
             maximum(ProbeStage::kImu),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kImu).over_50ms),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kImu).fail),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kMotorDebug).count),
             maximum(ProbeStage::kMotorDebug),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kMotorDebug).over_50ms),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kMotorDebug).fail),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kScan).count),
             maximum(ProbeStage::kScan),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kScan).over_50ms),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kScan).fail),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kBattery).count),
             maximum(ProbeStage::kBattery),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kBattery).over_50ms),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kBattery).fail),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kUltrasonic).count),
             maximum(ProbeStage::kUltrasonic),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kUltrasonic).over_50ms),
             static_cast<unsigned long long>(probe_stats(ProbeStage::kUltrasonic).fail));

    static constexpr const char *kProbeNames[] = {
        "odom", "imu", "debug", "scan", "battery", "ultra", "ping", "setup_ping",
        "wait", "cmd", "timer", "svc", "spin", "loop", "epoch", "report",
    };
    static_assert(sizeof(kProbeNames) / sizeof(kProbeNames[0]) ==
                  static_cast<size_t>(ProbeStage::kCount));
    for (size_t index = 0; index < static_cast<size_t>(ProbeStage::kCount); ++index)
    {
        const ProbeStats &stats = s_probe_stats[index];
        if (stats.over_50ms > 0)
        {
            ESP_LOGI(TAG,
                     "MICROROS_TIMING_SLOW stage=%s count=%llu gt50=%llu max_us=%llu max_end_mcu_ms=%u debug_seq=%u fail=%llu",
                     kProbeNames[index],
                     static_cast<unsigned long long>(stats.count),
                     static_cast<unsigned long long>(stats.over_50ms),
                     static_cast<unsigned long long>(stats.max_us),
                     static_cast<unsigned>(stats.max_end_mcu_ms),
                     static_cast<unsigned>(stats.max_debug_publish_seq),
                     static_cast<unsigned long long>(stats.fail));
        }
    }

    for (ProbeStats &stats : s_probe_stats)
    {
        stats.Reset();
    }
    s_probe_report_start_us = esp_timer_get_time();
    observe_probe(ProbeStage::kReport, report_start_us);
}

#define RCCHECK(fn)                                                                  \
    do                                                                               \
    {                                                                                \
        rcl_ret_t rc = (fn);                                                         \
        if (rc != RCL_RET_OK)                                                        \
        {                                                                            \
            ESP_LOGW(TAG, "rcl status line %d: %d", __LINE__, static_cast<int>(rc)); \
            return false;                                                            \
        }                                                                            \
    } while (0)

#define RCSOFTCHECK_TIMED(stage, fn)                                                 \
    do                                                                               \
    {                                                                                \
        const int64_t probe_start_us = esp_timer_get_time();                         \
        rcl_ret_t rc = (fn);                                                         \
        observe_probe((stage), probe_start_us, rc == RCL_RET_OK);                    \
        if (rc != RCL_RET_OK)                                                        \
        {                                                                            \
            ESP_LOGW(TAG, "rcl status line %d: %d", __LINE__, static_cast<int>(rc)); \
        }                                                                            \
    } while (0)

#define RCPUBLISHCHECK(stage, fn)                                                            \
    do                                                                                       \
    {                                                                                        \
        const int64_t probe_start_us = esp_timer_get_time();                                 \
        rcl_ret_t rc = (fn);                                                                 \
        observe_probe((stage), probe_start_us, rc == RCL_RET_OK);                            \
        if (rc != RCL_RET_OK)                                                                \
        {                                                                                    \
            s_ros_session_error = true;                                                      \
            ESP_LOGW(TAG, "rcl publish failed line %d: %d", __LINE__, static_cast<int>(rc)); \
        }                                                                                    \
    } while (0)

extern "C" bool transport_open_udp(uxrCustomTransport *transport)
{
    auto *ctx = static_cast<UdpTransportContext *>(transport->args);
    ctx->fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (ctx->fd < 0)
    {
        return false;
    }

    sockaddr_in local = {};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(ctx->local_port);
    bind(ctx->fd, reinterpret_cast<sockaddr *>(&local), sizeof(local));
    return true;
}

extern "C" bool transport_close_udp(uxrCustomTransport *transport)
{
    auto *ctx = static_cast<UdpTransportContext *>(transport->args);
    if (ctx->fd >= 0)
    {
        close(ctx->fd);
        ctx->fd = -1;
    }
    return true;
}

extern "C" size_t transport_write_udp(
    uxrCustomTransport *transport,
    const uint8_t *buf,
    size_t len,
    uint8_t *)
{
    auto *ctx = static_cast<UdpTransportContext *>(transport->args);
    const int sent = sendto(
        ctx->fd,
        buf,
        len,
        0,
        reinterpret_cast<sockaddr *>(&ctx->remote),
        sizeof(ctx->remote));
    return sent > 0 ? static_cast<size_t>(sent) : 0;
}

extern "C" size_t transport_read_udp(
    uxrCustomTransport *transport,
    uint8_t *buf,
    size_t len,
    int timeout,
    uint8_t *)
{
    auto *ctx = static_cast<UdpTransportContext *>(transport->args);
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(ctx->fd, &readfds);

    timeval tv = {};
    tv.tv_sec = timeout / 1000;
    tv.tv_usec = (timeout % 1000) * 1000;
    const int ret = select(ctx->fd + 1, &readfds, nullptr, nullptr, &tv);
    if (ret <= 0)
    {
        return 0;
    }

    const int received = recv(ctx->fd, buf, len, 0);
    return received > 0 ? static_cast<size_t>(received) : 0;
}

static void cleanup_result(rcl_ret_t ret)
{
    if (ret != RCL_RET_OK)
    {
        ESP_LOGW(TAG, "cleanup status: %d", static_cast<int>(ret));
    }
}

static void reset_ros_handles()
{
    s_odom_publisher = rcl_get_zero_initialized_publisher();
    s_imu_publisher = rcl_get_zero_initialized_publisher();
    s_scan_publisher = rcl_get_zero_initialized_publisher();
    s_battery_publisher = rcl_get_zero_initialized_publisher();
    s_ultrasonic_publisher = rcl_get_zero_initialized_publisher();
    s_cmd_vel_subscriber = rcl_get_zero_initialized_subscription();
    s_estop_subscriber = rcl_get_zero_initialized_subscription();
    s_set_speed_pid_service = rcl_get_zero_initialized_service();
    s_get_speed_pid_service = rcl_get_zero_initialized_service();
    s_service_executor = rclc_executor_get_zero_initialized_executor();
    s_init_options = rcl_get_zero_initialized_init_options();
    s_context = rcl_get_zero_initialized_context();
    s_node = rcl_get_zero_initialized_node();
    s_publish_timer = rcl_get_zero_initialized_timer();
    s_wait_set = rcl_get_zero_initialized_wait_set();
    s_clock = {};
}

static void handle_cmd_vel(const geometry_msgs__msg__Twist *msg)
{
    if (msg == nullptr || q_motion_cmd == nullptr)
    {
        return;
    }

    MotionMsg cmd = {};
    cmd.source = MOTION_SRC_MICROROS;
    cmd.control_mode = 0;
    cmd.target_vx = static_cast<float>(msg->linear.x * 1000.0);
    cmd.target_vy = static_cast<float>(msg->linear.y * 1000.0);
    cmd.target_wz = static_cast<float>(msg->angular.z);
    (void)motion_command_submit(cmd);
}

// /emergency_stop: true 锁存急停，false 释放。释放后仍需新的运动命令才会动。
static void handle_emergency_stop(const std_msgs__msg__Bool *msg)
{
    if (msg == nullptr)
    {
        return;
    }
    const bool was_active = motion_emergency_stop_active();
    motion_emergency_stop_set(msg->data);
    if (was_active != msg->data)
    {
        ESP_LOGW(TAG, "Emergency stop %s via /emergency_stop", msg->data ? "LATCHED" : "released");
    }
}

static void set_speed_pid_response(
    leap_interfaces__srv__SetSpeedPid_Response *res,
    bool success,
    const PidMsg &pid_msg)
{
    if (res == nullptr)
    {
        return;
    }
    res->success = success;
    res->kp = pid_msg.kp;
    res->ki = pid_msg.ki;
    res->kd = pid_msg.kd;
}

static void get_speed_pid_response(
    leap_interfaces__srv__GetSpeedPid_Response *res,
    bool success,
    const PidMsg &pid_msg)
{
    if (res == nullptr)
    {
        return;
    }
    res->success = success;
    res->kp = pid_msg.kp;
    res->ki = pid_msg.ki;
    res->kd = pid_msg.kd;
}

static void handle_set_speed_pid_service(const void *req, void *res)
{
    auto *request = static_cast<const leap_interfaces__srv__SetSpeedPid_Request *>(req);
    auto *response = static_cast<leap_interfaces__srv__SetSpeedPid_Response *>(res);
    if (request == nullptr || response == nullptr || q_speedpid_cmd == nullptr ||
        !isfinite(request->kp) || !isfinite(request->ki) || !isfinite(request->kd))
    {
        set_speed_pid_response(response, false, g_speed_pid_state);
        return;
    }

    PidMsg pid_msg = {};
    pid_msg.kp = request->kp;
    pid_msg.ki = request->ki;
    pid_msg.kd = request->kd;
    esp_err_t save_err = pid_save_speed_config(&pid_msg);
    if (save_err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to save speed PID config: %s", esp_err_to_name(save_err));
        set_speed_pid_response(response, false, g_speed_pid_state);
        return;
    }

    g_speed_pid_state = pid_msg;
    xQueueOverwrite(q_speedpid_cmd, &pid_msg);
    set_speed_pid_response(response, true, pid_msg);
    ESP_LOGI(TAG, "Speed PID service set and saved: kp=%.3f, ki=%.3f, kd=%.3f",
             pid_msg.kp, pid_msg.ki, pid_msg.kd);
}

static void handle_get_speed_pid_service(const void *, void *res)
{
    auto *response = static_cast<leap_interfaces__srv__GetSpeedPid_Response *>(res);
    get_speed_pid_response(response, true, g_speed_pid_state);
}

static void spin_pid_services()
{
    if (s_service_executor_initialized)
    {
        RCSOFTCHECK_TIMED(
            ProbeStage::kService,
            rclc_executor_spin_some(&s_service_executor, RCL_MS_TO_NS(0)));
    }
}

static void set_stamp(std_msgs__msg__Header *header, int64_t stamp_ms)
{
    header->stamp.sec = static_cast<int32_t>(stamp_ms / 1000);
    header->stamp.nanosec = static_cast<uint32_t>((stamp_ms % 1000) * 1000000);
}

static void publish_odom(int64_t stamp_ms)
{
    MotionMsg motion = {};
    if (q_motion_state == nullptr || xQueuePeek(q_motion_state, &motion, 0) != pdTRUE)
    {
        return;
    }

    set_stamp(&s_odom_msg.header, stamp_ms);
    s_odom_msg.pose.pose.position.x = motion.x / 1000.0;
    s_odom_msg.pose.pose.position.y = motion.y / 1000.0;
    s_odom_msg.pose.pose.position.z = 0.0;
    s_odom_msg.pose.pose.orientation.w = motion.qw;
    s_odom_msg.pose.pose.orientation.x = motion.qx;
    s_odom_msg.pose.pose.orientation.y = motion.qy;
    s_odom_msg.pose.pose.orientation.z = motion.qz;
    s_odom_msg.twist.twist.linear.x = motion.vx / 1000.0;
    s_odom_msg.twist.twist.linear.y = motion.vy / 1000.0;
    s_odom_msg.twist.twist.angular.z = motion.wz;
    RCPUBLISHCHECK(ProbeStage::kOdom, rcl_publish(&s_odom_publisher, &s_odom_msg, nullptr));
}
static void publish_motor_debug()
{
    MotorDebugMsg debug = {};

    if (q_motor_debug_state == nullptr ||
        xQueuePeek(q_motor_debug_state, &debug, 0) != pdTRUE)
    {
        return;
    }

    s_motor_debug_data[0] = debug.left_target_rpm;
    s_motor_debug_data[1] = debug.left_actual_rpm;
    s_motor_debug_data[2] = debug.left_raw_pwm;
    s_motor_debug_data[3] = debug.left_final_pwm;

    s_motor_debug_data[4] = debug.right_target_rpm;
    s_motor_debug_data[5] = debug.right_actual_rpm;
    s_motor_debug_data[6] = debug.right_raw_pwm;
    s_motor_debug_data[7] = debug.right_final_pwm;

    s_motor_debug_data[8] = debug.sync_error_rpm;
    s_motor_debug_data[9] = debug.sync_correction_pwm;
    s_motor_debug_data[10] = debug.sync_active ? 1.0f : 0.0f;
    s_motor_debug_data[11] = debug.startup_active ? 1.0f : 0.0f;
    s_motor_debug_data[12] = debug.startup_fault ? 1.0f : 0.0f;

    // Each 16-bit part is represented exactly in Float32MultiArray. Sequence
    // numbers count attempted publications, not delivery acknowledgements.
    const uint32_t publish_time_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    s_motor_debug_data[13] = static_cast<float>(debug.control_seq);
    s_motor_debug_data[14] = static_cast<float>(debug.control_time_ms >> 16);
    s_motor_debug_data[15] = static_cast<float>(debug.control_time_ms & 0xffffU);
    s_motor_debug_data[16] = static_cast<float>(++s_motor_debug_publish_seq);
    s_motor_debug_data[17] = static_cast<float>(publish_time_ms >> 16);
    s_motor_debug_data[18] = static_cast<float>(publish_time_ms & 0xffffU);

    s_motor_debug_msg.data.data = s_motor_debug_data;
    s_motor_debug_msg.data.size = 19;
    s_motor_debug_msg.data.capacity = 19;

    RCPUBLISHCHECK(
        ProbeStage::kMotorDebug,
        rcl_publish(
            &s_motor_debug_publisher,
            &s_motor_debug_msg,
            nullptr));
}

static void publish_imu(int64_t stamp_ms)
{
    ImuMsg imu = {};
    if (q_imu_state == nullptr || xQueuePeek(q_imu_state, &imu, 0) != pdTRUE)
    {
        return;
    }

    set_stamp(&s_imu_msg.header, stamp_ms);
    s_imu_msg.orientation.w = imu.qw;
    s_imu_msg.orientation.x = imu.qx;
    s_imu_msg.orientation.y = imu.qy;
    s_imu_msg.orientation.z = imu.qz;
    s_imu_msg.angular_velocity.x = imu.gyro_x * kDegToRad;
    s_imu_msg.angular_velocity.y = imu.gyro_y * kDegToRad;
    s_imu_msg.angular_velocity.z = imu.gyro_z * kDegToRad;
    s_imu_msg.linear_acceleration.x = imu.acc_x * kGravity;
    s_imu_msg.linear_acceleration.y = imu.acc_y * kGravity;
    s_imu_msg.linear_acceleration.z = imu.acc_z * kGravity;
    RCPUBLISHCHECK(ProbeStage::kImu, rcl_publish(&s_imu_publisher, &s_imu_msg, nullptr));
}

static void publish_scan(int64_t stamp_ms)
{
    LidarMsg lidar = {};
    if (q_lidar_state == nullptr || xQueuePeek(q_lidar_state, &lidar, 0) != pdTRUE)
    {
        return;
    }

    set_stamp(&s_scan_msg.header, stamp_ms);
    if (!DownsampleLaserScanMin(
            lidar.distances,
            kLaserScanSourcePointCount,
            s_scan_msg.ranges.data,
            kLaserScanPointCount))
    {
        return;
    }
    RCPUBLISHCHECK(ProbeStage::kScan, rcl_publish(&s_scan_publisher, &s_scan_msg, nullptr));
}

static void publish_battery(int64_t stamp_ms)
{
    BatteryMsg battery = {};
    if (q_battery_state == nullptr || xQueuePeek(q_battery_state, &battery, 0) != pdTRUE)
    {
        return;
    }
    PowerSafetyMsg safety = {};
    const bool has_safety = q_power_safety_state != nullptr &&
                            xQueuePeek(q_power_safety_state, &safety, 0) == pdTRUE;
    const bool battery_value_valid = battery.valid && isfinite(battery.voltage_v);
    const bool battery_present = battery_value_valid && has_safety &&
                                 safety.adc_valid && safety.sample_fresh;

    set_stamp(&s_battery_msg.header, stamp_ms);
    s_battery_msg.voltage = battery_value_valid ? battery.voltage_v : NAN;
    s_battery_msg.temperature = NAN;
    s_battery_msg.current = NAN;
    s_battery_msg.charge = NAN;
    s_battery_msg.capacity = NAN;
    s_battery_msg.design_capacity = NAN;
    s_battery_msg.percentage = battery_value_valid
        ? static_cast<float>(battery.percentage) / 100.0f
        : NAN;
    s_battery_msg.power_supply_status =
        sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;
    if (!battery_present)
    {
        s_battery_msg.power_supply_health =
            sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN;
    }
    else if (battery.voltage_v <= 6.0f)
    {
        s_battery_msg.power_supply_health =
            sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_DEAD;
    }
    else if (battery.voltage_v > 8.6f)
    {
        s_battery_msg.power_supply_health =
            sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_OVERVOLTAGE;
    }
    else
    {
        s_battery_msg.power_supply_health =
            sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_GOOD;
    }
    s_battery_msg.power_supply_technology =
        sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LIPO;
    s_battery_msg.present = battery_present;
    RCPUBLISHCHECK(ProbeStage::kBattery, rcl_publish(&s_battery_publisher, &s_battery_msg, nullptr));
}

static void publish_ultrasonic(int64_t stamp_ms)
{
    UltrasonicMsg ultrasonic = {};
    if (q_ultrasonic_state == nullptr ||
        xQueuePeek(q_ultrasonic_state, &ultrasonic, 0) != pdTRUE)
    {
        return;
    }

    set_stamp(&s_ultrasonic_msg.header, stamp_ms);
    s_ultrasonic_msg.range = ultrasonic.distance_cm > 0.0f
                                 ? ultrasonic.distance_cm / 100.0f
                                 : NAN;
    RCPUBLISHCHECK(ProbeStage::kUltrasonic, rcl_publish(&s_ultrasonic_publisher, &s_ultrasonic_msg, nullptr));
}

static void publish_state_timer(rcl_timer_t *timer, int64_t)
{
    if (timer == nullptr || g_wifi_comm_mode != WifiCommMode::kMicroRos)
    {
        return;
    }

    const int64_t epoch_start_us = esp_timer_get_time();
    const int64_t stamp_ms = rmw_uros_epoch_millis();
    observe_probe(ProbeStage::kEpoch, epoch_start_us);
    publish_odom(stamp_ms);
    publish_imu(stamp_ms);
    publish_motor_debug();

    if ((++s_publish_tick % 5) == 0)
    {
#if !defined(CONFIG_MICROROS_DISABLE_SCAN_AB_TEST)
        publish_scan(stamp_ms);
#endif
        publish_battery(stamp_ms);
        publish_ultrasonic(stamp_ms);
    }
}

static bool on_agent_discovered(const TransportLocator *locator, void *args)
{
    auto *result = static_cast<AgentDiscoveryResult *>(args);
    if (locator == nullptr || result == nullptr)
    {
        return false;
    }

    char candidate_ip[sizeof(result->ip)] = {0};
    uint16_t candidate_port = 0;
    uxrIpProtocol protocol = UXR_IPv4;
    if (!uxr_locator_to_ip(locator, candidate_ip, sizeof(candidate_ip),
                           &candidate_port, &protocol) ||
        protocol != UXR_IPv4 || candidate_port == 0)
    {
        return false;
    }

    in_addr candidate_addr = {};
    if (inet_pton(AF_INET, candidate_ip, &candidate_addr) != 1)
    {
        return false;
    }

    const uint32_t candidate_host_order = ntohl(candidate_addr.s_addr);
    if (candidate_host_order == 0 || (candidate_host_order >> 24) == 127)
    {
        return false;
    }

    if (result->has_sta_ip_info)
    {
        const uint32_t mask = result->sta_ip_info.netmask.addr;
        const uint32_t local_network = result->sta_ip_info.ip.addr & mask;
        if ((candidate_addr.s_addr & mask) != local_network)
        {
            return false;
        }
    }

    snprintf(result->ip, sizeof(result->ip), "%s", candidate_ip);
    result->port = candidate_port;
    result->found = true;
    return true;
}

static void discover_agent_multicast(AgentDiscoveryResult *result)
{
    TransportLocator multicast_locator = {};
    if (!uxr_ip_to_locator(kAgentDiscoveryMulticastIp, kAgentDiscoveryPort,
                           UXR_IPv4, &multicast_locator))
    {
        return;
    }

    uxr_discovery_agents(
        kAgentMulticastDiscoveryAttempts,
        kAgentMulticastDiscoveryPeriodMs,
        on_agent_discovered,
        result,
        &multicast_locator,
        1);
}

static void discover_agent_unicast(AgentDiscoveryResult *result)
{
    if (result == nullptr || !result->has_sta_ip_info || result->found)
    {
        return;
    }

    const uint32_t local_ip = ntohl(result->sta_ip_info.ip.addr);
    const uint32_t configured_mask = ntohl(result->sta_ip_info.netmask.addr);
    uint32_t network = local_ip & configured_mask;
    uint32_t broadcast = network | ~configured_mask;
    uint64_t usable_hosts = broadcast > network + 1
                                ? static_cast<uint64_t>(broadcast) - network - 1
                                : 0;

    if (usable_hosts == 0 || usable_hosts > kMaxAgentUnicastCandidates)
    {
        // Consumer Wi-Fi networks are normally /24. Limit unexpectedly large
        // subnets so automatic discovery stays bounded in time and memory.
        network = local_ip & 0xFFFFFF00UL;
        broadcast = network | 0x000000FFUL;
        usable_hosts = 254;
        ESP_LOGW(TAG, "Agent discovery subnet is too large; limiting scan to local /24");
    }

    auto *candidates = static_cast<TransportLocator *>(
        calloc(static_cast<size_t>(usable_hosts), sizeof(TransportLocator)));
    if (candidates == nullptr)
    {
        ESP_LOGW(TAG, "Unable to allocate Agent discovery candidate list");
        return;
    }

    size_t candidate_count = 0;
    for (uint32_t candidate = network + 1; candidate < broadcast; ++candidate)
    {
        if (candidate == local_ip)
        {
            continue;
        }

        in_addr candidate_addr = {};
        candidate_addr.s_addr = htonl(candidate);
        char candidate_ip[16] = {0};
        if (inet_ntop(AF_INET, &candidate_addr, candidate_ip,
                      sizeof(candidate_ip)) != nullptr &&
            uxr_ip_to_locator(candidate_ip, kAgentDiscoveryPort, UXR_IPv4,
                              &candidates[candidate_count]))
        {
            ++candidate_count;
        }
    }

    if (candidate_count > 0)
    {
        ESP_LOGI(TAG, "Multicast unavailable; probing %u hosts on discovery port %u",
                 static_cast<unsigned>(candidate_count),
                 static_cast<unsigned>(kAgentDiscoveryPort));
        uxr_discovery_agents(
            kAgentUnicastDiscoveryAttempts,
            kAgentUnicastDiscoveryPeriodMs,
            on_agent_discovered,
            result,
            candidates,
            candidate_count);
    }
    free(candidates);
}

static bool discover_agent_endpoint()
{
    AgentDiscoveryResult result = {};
    result.has_sta_ip_info = wifi_get_sta_ip_info(&result.sta_ip_info) == ESP_OK;

    ESP_LOGI(TAG, "Discovering micro-ROS Agent (discovery port %u)",
             static_cast<unsigned>(kAgentDiscoveryPort));
    const int64_t discovery_start_us = esp_timer_get_time();
    discover_agent_multicast(&result);
    discover_agent_unicast(&result);
    const int64_t discovery_elapsed_ms =
        (esp_timer_get_time() - discovery_start_us) / 1000;

    if (!result.found)
    {
        ESP_LOGW(TAG, "micro-ROS Agent discovery timed out after %lld ms",
                 static_cast<long long>(discovery_elapsed_ms));
        return false;
    }

    snprintf(g_microros_agent_ip, sizeof(g_microros_agent_ip), "%s", result.ip);
    g_microros_agent_port = result.port;
    ESP_LOGI(TAG, "Discovered micro-ROS Agent: %s:%u in %lld ms",
             g_microros_agent_ip,
             static_cast<unsigned>(g_microros_agent_port),
             static_cast<long long>(discovery_elapsed_ms));
    return true;
}

static bool ensure_agent_endpoint()
{
    if (!g_microros_agent_auto_discovery)
    {
        return g_microros_agent_ip[0] != '\0';
    }
    if (g_microros_agent_ip[0] != '\0')
    {
        return true;
    }
    return discover_agent_endpoint();
}

static void forget_auto_discovered_agent()
{
    if (g_microros_agent_auto_discovery)
    {
        g_microros_agent_ip[0] = '\0';
    }
}

static bool setup_udp_transport()
{
    memset(&s_udp_ctx, 0, sizeof(s_udp_ctx));
    s_udp_ctx.fd = -1;
    s_udp_ctx.local_port = static_cast<uint16_t>(atoi(CONFIG_MICRO_ROS_LOCAL_PORT));
    s_udp_ctx.remote.sin_family = AF_INET;
    s_udp_ctx.remote.sin_port = htons(g_microros_agent_port);

    if (inet_pton(AF_INET, g_microros_agent_ip, &s_udp_ctx.remote.sin_addr.s_addr) != 1)
    {
        ESP_LOGE(TAG, "invalid micro-ROS agent IP: %s", g_microros_agent_ip);
        return false;
    }

    rmw_uros_set_custom_transport(
        false,
        &s_udp_ctx,
        transport_open_udp,
        transport_close_udp,
        transport_write_udp,
        transport_read_udp);
    return true;
}

static bool create_ros_entities()
{
    s_publish_tick = 0;
    s_last_agent_check_tick = xTaskGetTickCount();
    s_ros_session_error = false;
    s_agent_ping_health.Reset();
    reset_ros_handles();

    rosidl_runtime_c__String__init(&s_odom_msg.header.frame_id);
    rosidl_runtime_c__String__init(&s_odom_msg.child_frame_id);
    rosidl_runtime_c__String__init(&s_imu_msg.header.frame_id);
    rosidl_runtime_c__String__init(&s_scan_msg.header.frame_id);
    rosidl_runtime_c__String__init(&s_ultrasonic_msg.header.frame_id);
    s_strings_initialized = true;

    (void)rosidl_runtime_c__String__assign(&s_odom_msg.header.frame_id, "odom");
    (void)rosidl_runtime_c__String__assign(&s_odom_msg.child_frame_id, "base_link");
    (void)rosidl_runtime_c__String__assign(&s_imu_msg.header.frame_id, "imu_link");
    (void)rosidl_runtime_c__String__assign(&s_scan_msg.header.frame_id, "laser_frame");
    (void)rosidl_runtime_c__String__assign(&s_ultrasonic_msg.header.frame_id, "ultrasonic_link");

    if (!rosidl_runtime_c__float32__Sequence__init(&s_scan_msg.ranges, kLaserScanPointCount))
    {
        ESP_LOGE(TAG, "failed to allocate LaserScan ranges");
        return false;
    }
    s_scan_ranges_initialized = true;

    if (!sensor_msgs__msg__BatteryState__init(&s_battery_msg))
    {
        ESP_LOGE(TAG, "failed to initialize BatteryState message");
        return false;
    }
    s_battery_msg_initialized = true;
    (void)rosidl_runtime_c__String__assign(&s_battery_msg.header.frame_id, "battery");
    (void)rosidl_runtime_c__String__assign(&s_battery_msg.location, "main");

    s_scan_msg.angle_min = 0.0f;
    s_scan_msg.angle_increment = static_cast<float>(kFullCircleRad / kLaserScanPointCount);
    s_scan_msg.angle_max = static_cast<float>(
        (kLaserScanPointCount - 1) * s_scan_msg.angle_increment);
    s_scan_msg.time_increment = 0.0f;
    s_scan_msg.scan_time = 0.1f;
    s_scan_msg.range_min = 0.02f;
    s_scan_msg.range_max = 12.0f;

    s_ultrasonic_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    s_ultrasonic_msg.field_of_view = 0.26f;
    s_ultrasonic_msg.min_range = 0.02f;
    s_ultrasonic_msg.max_range = 4.0f;
    s_ultrasonic_msg.range = NAN;

    s_imu_msg.orientation_covariance[0] = -1.0;
    s_imu_msg.angular_velocity_covariance[0] = -1.0;
    s_imu_msg.linear_acceleration_covariance[0] = -1.0;

    s_allocator = rcl_get_default_allocator();
    s_init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&s_init_options, s_allocator));
    s_init_options_initialized = true;

    const int64_t ping_start_us = esp_timer_get_time();
    const rmw_ret_t ping_ret = rmw_uros_ping_agent(300, 3);
    observe_probe(ProbeStage::kSetupPing, ping_start_us, ping_ret == RMW_RET_OK);
    if (ping_ret != RMW_RET_OK)
    {
        // ESP_LOGW(TAG, "micro-ROS agent unavailable: %s:%u",
        //          g_microros_agent_ip, static_cast<unsigned>(g_microros_agent_port));
        return false;
    }

    s_context = rcl_get_zero_initialized_context();
    RCCHECK(rcl_init(0, nullptr, &s_init_options, &s_context));
    s_context_initialized = true;

    if (!rmw_uros_epoch_synchronized())
    {
        (void)rmw_uros_sync_session(1000);
    }

    s_node = rcl_get_zero_initialized_node();
    rcl_node_options_t node_options = rcl_node_get_default_options();
    node_options.enable_rosout = false;
    const rcl_ret_t node_ret = rcl_node_init(&s_node, "leap_low_driver", "", &s_context, &node_options);
    cleanup_result(rcl_node_options_fini(&node_options));
    if (node_ret != RCL_RET_OK)
    {
        ESP_LOGW(TAG, "rcl status line %d: %d", __LINE__, static_cast<int>(node_ret));
        return false;
    }
    s_node_initialized = true;

    rcl_publisher_options_t pub_options = rcl_publisher_get_default_options();
    rcl_publisher_options_t telemetry_pub_options = pub_options;
#if defined(CONFIG_MICROROS_TELEMETRY_BEST_EFFORT)
    telemetry_pub_options.qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
#endif
    rcl_publisher_options_t sensor_pub_options = rcl_publisher_get_default_options();
    sensor_pub_options.qos = rmw_qos_profile_sensor_data;
    s_odom_publisher = rcl_get_zero_initialized_publisher();
    RCCHECK(rcl_publisher_init(
        &s_odom_publisher,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom",
        &telemetry_pub_options));
    s_odom_publisher_initialized = true;

    s_motor_debug_publisher = rcl_get_zero_initialized_publisher();

    RCCHECK(rcl_publisher_init(
        &s_motor_debug_publisher,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(
            std_msgs,
            msg,
            Float32MultiArray),
        "motor_debug",
        &telemetry_pub_options));

    s_motor_debug_publisher_initialized = true;

    s_imu_publisher = rcl_get_zero_initialized_publisher();
    RCCHECK(rcl_publisher_init(
        &s_imu_publisher,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu",
        &sensor_pub_options));
    s_imu_publisher_initialized = true;

    s_scan_publisher = rcl_get_zero_initialized_publisher();
    RCCHECK(rcl_publisher_init(
        &s_scan_publisher,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "scan",
        &telemetry_pub_options));
    s_scan_publisher_initialized = true;

    s_battery_publisher = rcl_get_zero_initialized_publisher();
    RCCHECK(rcl_publisher_init(
        &s_battery_publisher,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
        "battery_state",
        &sensor_pub_options));
    s_battery_publisher_initialized = true;

    s_ultrasonic_publisher = rcl_get_zero_initialized_publisher();
    RCCHECK(rcl_publisher_init(
        &s_ultrasonic_publisher,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "ultrasonic",
        &sensor_pub_options));
    s_ultrasonic_publisher_initialized = true;

    rcl_subscription_options_t sub_options = rcl_subscription_get_default_options();
    sub_options.qos = rmw_qos_profile_sensor_data;
    s_cmd_vel_subscriber = rcl_get_zero_initialized_subscription();
    RCCHECK(rcl_subscription_init(
        &s_cmd_vel_subscriber,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel",
        &sub_options));
    s_cmd_vel_subscriber_initialized = true;

    // 急停必须可靠送达，因此使用默认（Reliable）QoS，而不是 sensor_data。
    rcl_subscription_options_t estop_options = rcl_subscription_get_default_options();
    estop_options.qos = rmw_qos_profile_default;
    s_estop_subscriber = rcl_get_zero_initialized_subscription();
    RCCHECK(rcl_subscription_init(
        &s_estop_subscriber,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "emergency_stop",
        &estop_options));
    s_estop_subscriber_initialized = true;

    rcl_service_options_t service_options = rcl_service_get_default_options();
    service_options.qos = rmw_qos_profile_services_default;
    RCCHECK(rcl_service_init(
        &s_set_speed_pid_service,
        &s_node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(leap_interfaces, srv, SetSpeedPid),
        "set_speed_pid",
        &service_options));
    s_set_speed_pid_service_initialized = true;

    RCCHECK(rcl_service_init(
        &s_get_speed_pid_service,
        &s_node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(leap_interfaces, srv, GetSpeedPid),
        "get_speed_pid",
        &service_options));
    s_get_speed_pid_service_initialized = true;

    RCCHECK(rclc_executor_init(&s_service_executor, &s_context, 2, &s_allocator));
    s_service_executor_initialized = true;
    RCCHECK(rclc_executor_add_service(
        &s_service_executor,
        &s_set_speed_pid_service,
        &s_set_speed_pid_req,
        &s_set_speed_pid_res,
        handle_set_speed_pid_service));
    RCCHECK(rclc_executor_add_service(
        &s_service_executor,
        &s_get_speed_pid_service,
        &s_get_speed_pid_req,
        &s_get_speed_pid_res,
        handle_get_speed_pid_service));

    s_clock = {};
    s_publish_timer = rcl_get_zero_initialized_timer();
    RCCHECK(rcl_steady_clock_init(&s_clock, &s_allocator));
    s_clock_initialized = true;
    RCCHECK(rcl_timer_init(
        &s_publish_timer,
        &s_clock,
        &s_context,
        RCL_MS_TO_NS(20),
        publish_state_timer,
        s_allocator));
    s_timer_initialized = true;

    s_wait_set = rcl_get_zero_initialized_wait_set();
    RCCHECK(rcl_wait_set_init(&s_wait_set, 2, 0, 1, 0, 0, 0, &s_context, s_allocator));
    s_wait_set_initialized = true;
    s_ros_created = true;
    return true;
}

static void destroy_ros_entities()
{
    s_agent_ping_health.Reset();
    if (!s_ros_created &&
        !s_strings_initialized &&
        !s_scan_ranges_initialized &&
        !s_init_options_initialized &&
        !s_context_initialized)
    {
        return;
    }

    if (s_context_initialized)
    {
        rmw_context_t *rmw_context = rcl_context_get_rmw_context(&s_context);
        if (rmw_context)
        {
            (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
        }
    }

    if (s_wait_set_initialized)
    {
        cleanup_result(rcl_wait_set_fini(&s_wait_set));
        s_wait_set_initialized = false;
    }
    if (s_timer_initialized)
    {
        cleanup_result(rcl_timer_fini(&s_publish_timer));
        s_timer_initialized = false;
    }
    if (s_clock_initialized)
    {
        cleanup_result(rcl_clock_fini(&s_clock));
        s_clock_initialized = false;
    }
    if (s_ultrasonic_publisher_initialized)
    {
        cleanup_result(rcl_publisher_fini(&s_ultrasonic_publisher, &s_node));
        s_ultrasonic_publisher_initialized = false;
    }
    if (s_scan_publisher_initialized)
    {
        cleanup_result(rcl_publisher_fini(&s_scan_publisher, &s_node));
        s_scan_publisher_initialized = false;
    }
    if (s_battery_publisher_initialized)
    {
        cleanup_result(rcl_publisher_fini(&s_battery_publisher, &s_node));
        s_battery_publisher_initialized = false;
    }
    if (s_imu_publisher_initialized)
    {
        cleanup_result(rcl_publisher_fini(&s_imu_publisher, &s_node));
        s_imu_publisher_initialized = false;
    }
    if (s_odom_publisher_initialized)
    {
        cleanup_result(rcl_publisher_fini(&s_odom_publisher, &s_node));
        s_odom_publisher_initialized = false;
    }
    if (s_motor_debug_publisher_initialized)
    {
        cleanup_result(rcl_publisher_fini(&s_motor_debug_publisher, &s_node));
        s_motor_debug_publisher_initialized = false;
    }
    if (s_cmd_vel_subscriber_initialized)
    {
        cleanup_result(rcl_subscription_fini(&s_cmd_vel_subscriber, &s_node));
        s_cmd_vel_subscriber_initialized = false;
    }
    if (s_estop_subscriber_initialized)
    {
        cleanup_result(rcl_subscription_fini(&s_estop_subscriber, &s_node));
        s_estop_subscriber_initialized = false;
    }
    if (s_service_executor_initialized)
    {
        cleanup_result(rclc_executor_fini(&s_service_executor));
        s_service_executor_initialized = false;
    }
    if (s_set_speed_pid_service_initialized)
    {
        cleanup_result(rcl_service_fini(&s_set_speed_pid_service, &s_node));
        s_set_speed_pid_service_initialized = false;
    }
    if (s_get_speed_pid_service_initialized)
    {
        cleanup_result(rcl_service_fini(&s_get_speed_pid_service, &s_node));
        s_get_speed_pid_service_initialized = false;
    }
    if (s_node_initialized)
    {
        cleanup_result(rcl_node_fini(&s_node));
        s_node_initialized = false;
    }
    if (s_context_initialized)
    {
        cleanup_result(rcl_shutdown(&s_context));
        cleanup_result(rcl_context_fini(&s_context));
        s_context_initialized = false;
    }
    if (s_init_options_initialized)
    {
        cleanup_result(rcl_init_options_fini(&s_init_options));
        s_init_options_initialized = false;
    }
    if (s_strings_initialized)
    {
        rosidl_runtime_c__String__fini(&s_odom_msg.header.frame_id);
        rosidl_runtime_c__String__fini(&s_odom_msg.child_frame_id);
        rosidl_runtime_c__String__fini(&s_imu_msg.header.frame_id);
        rosidl_runtime_c__String__fini(&s_scan_msg.header.frame_id);
        rosidl_runtime_c__String__fini(&s_ultrasonic_msg.header.frame_id);
        s_strings_initialized = false;
    }
    if (s_scan_ranges_initialized)
    {
        rosidl_runtime_c__float32__Sequence__fini(&s_scan_msg.ranges);
        s_scan_ranges_initialized = false;
    }
    if (s_battery_msg_initialized)
    {
        sensor_msgs__msg__BatteryState__fini(&s_battery_msg);
        s_battery_msg_initialized = false;
    }

    s_ros_created = false;
    s_ros_session_error = false;
    reset_ros_handles();
}

static bool spin_once(int timeout_ms)
{
    if (!s_wait_set_initialized)
    {
        return false;
    }

    ProbeScope spin_scope(ProbeStage::kSpin);

    rcl_ret_t ret = rcl_wait_set_clear(&s_wait_set);
    if (ret != RCL_RET_OK)
    {
        ESP_LOGW(TAG, "rcl_wait_set_clear failed: %d", static_cast<int>(ret));
        return false;
    }
    ret = rcl_wait_set_add_subscription(&s_wait_set, &s_cmd_vel_subscriber, nullptr);
    if (ret == RCL_RET_OK)
    {
        ret = rcl_wait_set_add_subscription(&s_wait_set, &s_estop_subscriber, nullptr);
    }
    if (ret != RCL_RET_OK)
    {
        ESP_LOGW(TAG, "rcl_wait_set_add_subscription failed: %d", static_cast<int>(ret));
        return false;
    }
    ret = rcl_wait_set_add_timer(&s_wait_set, &s_publish_timer, nullptr);
    if (ret != RCL_RET_OK)
    {
        ESP_LOGW(TAG, "rcl_wait_set_add_timer failed: %d", static_cast<int>(ret));
        return false;
    }

    const int64_t wait_start_us = esp_timer_get_time();
    ret = rcl_wait(&s_wait_set, RCL_MS_TO_NS(timeout_ms));
    observe_probe(ProbeStage::kWait, wait_start_us,
                  ret == RCL_RET_OK || ret == RCL_RET_TIMEOUT);
    if (ret == RCL_RET_TIMEOUT)
    {
        return true;
    }
    if (ret != RCL_RET_OK)
    {
        ESP_LOGW(TAG, "rcl_wait failed: %d", static_cast<int>(ret));
        return false;
    }

    // 先处理急停，再处理同一轮到达的 cmd_vel。
    if (s_wait_set.subscriptions[1] != nullptr)
    {
        const rcl_ret_t take_ret = rcl_take(&s_estop_subscriber, &s_estop_msg, nullptr, nullptr);
        if (take_ret == RCL_RET_OK)
        {
            handle_emergency_stop(&s_estop_msg);
        }
        else if (take_ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED)
        {
            ESP_LOGW(TAG, "rcl_take(emergency_stop) failed: %d", static_cast<int>(take_ret));
        }
    }

    if (s_wait_set.subscriptions[0] != nullptr)
    {
        ProbeScope command_scope(ProbeStage::kCommand);
        const rcl_ret_t take_ret = rcl_take(&s_cmd_vel_subscriber, &s_cmd_vel_msg, nullptr, nullptr);
        if (take_ret == RCL_RET_OK)
        {
            handle_cmd_vel(&s_cmd_vel_msg);
        }
        else if (take_ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED)
        {
            ESP_LOGW(TAG, "rcl_take failed: %d", static_cast<int>(take_ret));
        }
    }

    if (s_wait_set.timers[0] != nullptr)
    {
        RCSOFTCHECK_TIMED(ProbeStage::kTimer, rcl_timer_call(&s_publish_timer));
    }
    spin_pid_services();
    if (s_ros_session_error)
    {
        ESP_LOGW(TAG, "micro-ROS publish failed, recreating session");
        return false;
    }
    return true;
}

static bool agent_still_reachable(TickType_t now)
{
    if ((now - s_last_agent_check_tick) < kAgentCheckIntervalTicks)
    {
        return true;
    }

    s_last_agent_check_tick = now;
    const int64_t ping_start_us = esp_timer_get_time();
    const rmw_ret_t ping_ret = rmw_uros_ping_agent(kAgentPingTimeoutMs, kAgentPingAttempts);
    observe_probe(ProbeStage::kPing, ping_start_us, ping_ret == RMW_RET_OK);
    const uint8_t previous_failures = s_agent_ping_health.consecutive_failures;
    const uint32_t previous_failure_age_ticks =
        s_agent_ping_health.FailureAgeTicks(static_cast<uint32_t>(now));
    if (ping_ret == RMW_RET_OK)
    {
        (void)s_agent_ping_health.KeepSession(
            true,
            static_cast<uint32_t>(now),
            kAgentPingFailureWindowTicks,
            kAgentPingMinimumFailures);
        if (previous_failures > 0)
        {
            ESP_LOGI(TAG, "micro-ROS agent health recovered after %u failed ping(s) over %u ms",
                     static_cast<unsigned>(previous_failures),
                     static_cast<unsigned>(pdTICKS_TO_MS(previous_failure_age_ticks)));
        }
        return true;
    }

    const bool keep_session =
        s_agent_ping_health.KeepSession(
            false,
            static_cast<uint32_t>(now),
            kAgentPingFailureWindowTicks,
            kAgentPingMinimumFailures);
    const uint32_t failure_age_ms = pdTICKS_TO_MS(
        s_agent_ping_health.FailureAgeTicks(static_cast<uint32_t>(now)));
    if (keep_session)
    {
        if (previous_failures == 0)
        {
            ESP_LOGW(TAG, "micro-ROS agent health ping failed; starting %u ms confirmation window",
                     static_cast<unsigned>(kAgentPingFailureWindowMs));
        }
        return true;
    }

    ESP_LOGW(TAG, "micro-ROS agent health failed %u time(s) over %u ms; recreating session",
             static_cast<unsigned>(s_agent_ping_health.consecutive_failures),
             static_cast<unsigned>(failure_age_ms));
    return false;
}

void microros_task(void *p)
{
    (void)p;

    while (1)
    {
        if (g_wifi_comm_mode == WifiCommMode::kMicroRos)
        {
            const int64_t loop_start_us = esp_timer_get_time();
            if (s_last_active_loop_us != 0)
            {
                observe_probe(ProbeStage::kLoopGap, s_last_active_loop_us);
            }
            s_last_active_loop_us = loop_start_us;
            report_probe_if_due();

            if (!s_ros_created)
            {
                if (ensure_agent_endpoint() &&
                    setup_udp_transport() &&
                    create_ros_entities())
                {
                    ++s_probe_session_generation;
                    ESP_LOGI(TAG, "micro-ROS active: session=%u agent_mode=%s agent=%s:%u, telemetry_qos=%s, scan_publish=%s scan_points=%u, health_ping=%dms_x%u every=%ums fail_window=%ums min_fail=%u, pub=/odom,/motor_debug,/imu,/scan,/battery_state,/ultrasonic, sub=/cmd_vel, srv=/set_speed_pid,/get_speed_pid",
                             static_cast<unsigned>(s_probe_session_generation),
                             g_microros_agent_auto_discovery ? "auto" : "manual",
                             g_microros_agent_ip,
                             static_cast<unsigned>(g_microros_agent_port),
                             kTelemetryQosMode,
                             kScanPublishMode,
                             static_cast<unsigned>(kLaserScanPointCount),
                             kAgentPingTimeoutMs,
                             static_cast<unsigned>(kAgentPingAttempts),
                             static_cast<unsigned>(kAgentCheckIntervalMs),
                             static_cast<unsigned>(kAgentPingFailureWindowMs),
                             static_cast<unsigned>(kAgentPingMinimumFailures));
                }
                else
                {
                    destroy_ros_entities();
                    forget_auto_discovered_agent();
                    s_last_active_loop_us = 0;
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    continue;
                }
            }
            // Service commands and due publications before the optional health
            // check so a slow ping cannot delay already-ready work.
            if (!spin_once(20))
            {
                report_probe_if_due(true);
                destroy_ros_entities();
                forget_auto_discovered_agent();
                s_last_active_loop_us = 0;
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            if (!agent_still_reachable(xTaskGetTickCount()))
            {
                report_probe_if_due(true);
                destroy_ros_entities();
                forget_auto_discovered_agent();
                s_last_active_loop_us = 0;
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
        }
        else
        {
            if (s_ros_created)
            {
                report_probe_if_due(true);
                destroy_ros_entities();
                forget_auto_discovered_agent();
                ESP_LOGI(TAG, "micro-ROS inactive");
            }
            s_last_active_loop_us = 0;
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
