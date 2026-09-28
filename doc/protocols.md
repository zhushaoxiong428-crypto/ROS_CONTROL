# Leap Low v1 MAVLink 与 micro-ROS 协议说明

软件版本：`v1.5`

本文档说明 `leap_low_v1` 固件提供的网络与串口通信协议。

## 运行模式

固件同一时间只启用一种通信模式：

| 模式 | 运行值 | 默认 |
| --- | --- | --- |
| micro-ROS | `micro_ros` | 是 |
| MAVLink UDP | `mavlink_udp` | 否 |
| MAVLink UART | `uart_mavlink` | 否 |

通信模式保存在 NVS 的运行配置中，可通过网页配置页面修改。micro-ROS Agent
默认使用自动发现，不保存自动发现到的 IP 地址。

## 单位约定

| 数据 | 内部单位 | MAVLink 单位 | ROS 单位 |
| --- | --- | --- | --- |
| 线性位置 | mm | m | m |
| 线速度 | mm/s | m/s | m/s |
| 航向角 / 角速度 | rad / rad/s | rad / rad/s | rad / rad/s |
| IMU 加速度 | g | `SCALED_IMU` 中为 mg，ROS 中为 m/s^2 | m/s^2 |
| IMU 陀螺仪 | deg/s | rad/s 或 mrad/s | rad/s |
| 雷达距离 | mm | `OBSTACLE_DISTANCE` 中为 cm | m |
| 电池电压 | V | mV | V |

## micro-ROS

传输方式：基于 Wi-Fi 的自定义 UDP 传输。

| 项目 | 值 |
| --- | --- |
| 节点名称 | `leap_low_driver` |
| 本地 UDP 端口 | `CONFIG_MICRO_ROS_LOCAL_PORT`，默认 `8888` |
| Agent 查找 | 先通过 Micro XRCE-DDS 组播 `239.255.0.2:8889` 自动发现；失败后对当前子网做有界单播发现；可在网页中切换为手动地址 |
| Agent 地址 | 自动模式下仅保存在运行内存 `g_microros_agent_ip:g_microros_agent_port`，不会写入 NVS |
| 定时器周期 | 20 ms |

电脑端使用普通启动命令即可，Agent 的发现服务默认启用：

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 --discovery 8889
```

固件升级到支持自动发现的版本后，会执行一次 NVS 迁移：删除旧的 `uros_ip`
固定地址，并写入自动发现模式。以后更换 Wi-Fi 网络时，小车会在新局域网重新发现
Agent。手机热点等设备即使禁止客户端组播，固件也会自动使用单播发现；若网络同时
禁止客户端之间的单播通信，才需要在网页通信设置中选择“手动地址”作为回退。

建链成功后，运行期健康检查每 2 秒在 micro-ROS 任务内串行执行一次，单次超时为
200 ms、仅尝试一次。只有从首次失败开始持续 12 秒且至少失败 3 次，才销毁会话并
重新发现 Agent；期间任一次成功都会清除失败窗口。每轮会先处理已经就绪的
`/cmd_vel` 和发布定时器，再执行健康检查。建链阶段仍使用较宽松的 `300 ms × 3`
探测。这样既避免对同一个 XRCE 会话进行并发 RMW 调用，也将运行期单次检查的阻塞
限制在 500 ms 速度指令看门狗以内。真正的运动失联保护仍由独立的 500 ms 看门狗
负责，因此约 12～14 秒的会话故障确认时间不会延长停车时间。

### 订阅话题

| 话题 | 类型 | QoS | 映射 |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | sensor data | `linear.x/y` 由 m/s 转为目标 `vx/vy`，单位 mm/s；`angular.z` 转为目标 `wz`，单位 rad/s |
| `/emergency_stop` | `std_msgs/msg/Bool` | reliable | `true` 锁存急停并立即制动；`false` 释放急停 |

收到 `/cmd_vel` 后会按顺序写入 `q_motion_cmd`（深度 8，满时丢弃最旧一条），将控制来源标记为 micro-ROS。运动命令**不会**清除急停。

急停（`/emergency_stop`、MAVLink DISARM 或板载 BOOT 键均可触发）锁存期间，电机持续制动，所有运动命令被丢弃。释放急停后，必须先收到一条全零的 mode 0 速度命令，之后的非零命令才会执行，例如：

```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}"   # 急停
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: false}"  # 释放
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"              # 重新武装
```

所有运动命令仍须通过电池运动互锁；MCU 启动或电池故障恢复后，必须先收到一条所有线速度和角速度均为零的 mode 0 命令，下一条非零命令才可执行。详见 [电池运动互锁](power_motion_interlock.md)。

### 发布话题

| 话题 | 类型 | 频率 | 坐标系 | 说明 |
| --- | --- | --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | 50 Hz | `odom`，子坐标系 `base_link` | 来自运动状态的位置与速度 |
| `/motor_debug` | `std_msgs/msg/Float32MultiArray` | 50 Hz | 无 | 轮速控制、起步与同步诊断，字段顺序见下表 |
| `/imu` | `sensor_msgs/msg/Imu` | 50 Hz | `imu_link` | 四元数、陀螺仪、加速度 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 10 Hz | `laser_frame` | 默认 320 个点，约 1.125 度/点，完整覆盖 360 度，范围 0.02-12.0 m |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | 10 Hz | `battery` | 电压与电量百分比；锂电池，放电状态 |
| `/ultrasonic` | `sensor_msgs/msg/Range` | 10 Hz | `ultrasonic_link` | HC-SR04 超声波距离，单位 m |

`/motor_debug.data` 字段：

| 索引 | 字段 | 单位 | 说明 |
| ---: | --- | --- | --- |
| 0 | `left_target_rpm` | rpm | 加速度限制后的左轮目标 |
| 1 | `left_actual_rpm` | rpm | 左轮编码器转速，经 `alpha=0.35` EMA 滤波 |
| 2 | `left_raw_pwm` | PWM | 左轮独立速度 PI 输出，不含前馈 |
| 3 | `left_final_pwm` | PWM | 前馈、起步协调、同步修正和限幅后的整数输出 |
| 4 | `right_target_rpm` | rpm | 加速度限制后的右轮目标 |
| 5 | `right_actual_rpm` | rpm | 右轮编码器转速，经 `alpha=0.35` EMA 滤波 |
| 6 | `right_raw_pwm` | PWM | 右轮独立速度 PI 输出，不含前馈 |
| 7 | `right_final_pwm` | PWM | 前馈、起步协调、同步修正和限幅后的整数输出 |
| 8 | `sync_error_rpm` | rpm | `(L target-L actual)-(R target-R actual)` |
| 9 | `sync_correction_pwm` | PWM | 执行起步约束前、受执行器余量限制的轮间修正 |
| 10 | `sync_active` | bool-as-float | `1.0` 表示直行轮间同步已启用 |
| 11 | `startup_active` | bool-as-float | `1.0` 表示本周期仍处于起步协调或换向等待 |
| 12 | `startup_fault` | bool-as-float | `1.0` 表示起步超时/非法控制输入，电机已制动 |
| 13 | `control_seq_mod65536` | count | 运动控制任务快照序号，按 16 位回绕 |
| 14 | `control_time_ms_hi16` | ms 高 16 位 | 运动控制快照的 MCU 单调时间高半字 |
| 15 | `control_time_ms_lo16` | ms 低 16 位 | 运动控制快照的 MCU 单调时间低半字 |
| 16 | `publish_seq_mod65536` | count | `/motor_debug` 发布尝试序号，按 16 位回绕，不代表主机确认收到 |
| 17 | `publish_time_ms_hi16` | ms 高 16 位 | 发布尝试的 MCU 单调时间高半字 |
| 18 | `publish_time_ms_lo16` | ms 低 16 位 | 发布尝试的 MCU 单调时间低半字 |

数组的前 13 项保持原顺序，旧客户端可以继续按前缀读取；严格要求数组长度恰好为
13 的客户端需要更新为 19 项。两个 MCU 时间均按
`time_ms = hi16 * 65536 + lo16` 重建。仓库中的 `record_motor_debug.py` 会同时记录主机
单调接收时间，`analyze_motor_timing.py` 用两组时间和序号区分控制周期、发布调度与
主机未观察到的发布尝试；它不能仅凭 CSV 将丢样唯一归因于 Wi-Fi、Agent、DDS 或主机。

`/battery_state` 字段：

| 字段 | 值 |
| --- | --- |
| `voltage` | 实测电池电压，单位 V |
| `percentage` | `0.0` 到 `1.0` |
| `power_supply_status` | `DISCHARGING` |
| `power_supply_health` | 安全范围内为 `GOOD`；`<=6.0 V` 为 `DEAD`；`>8.6 V` 为 `OVERVOLTAGE`；ADC 无效或样本陈旧为 `UNKNOWN` |
| `power_supply_technology` | `LIPO` |
| `present` | ADC 有效且样本未超过 MCU 新鲜度期限时为 `true`，否则为 `false` |
| `temperature/current/charge/capacity/design_capacity` | `NaN` |

## MAVLink UDP

传输方式：绑定到 `14550` 端口的 UDP socket。

| 项目 | 值 |
| --- | --- |
| 本地端口 | `14550` |
| 初始目标 | 广播地址 `255.255.255.255:14550` |
| 已连接目标 | 第一个发送 MAVLink 数据的地址 |
| System ID | `1` |
| Component ID | `1` |
| 主循环周期 | 20 ms |
| 车辆类型 | `MAV_TYPE_GROUND_ROVER` |
| Autopilot | `MAV_AUTOPILOT_GENERIC` |

## MAVLink UART

传输方式：`UART0`，波特率 `230400`，8N1。运行日志与 MAVLink UART 共用 UART0 串口。

| 项目 | 值 |
| --- | --- |
| TX 引脚 | GPIO43 |
| RX 引脚 | GPIO44 |
| 日志端口 | 共用 UART0 |
| System ID | `1` |
| Component ID | `1` |
| 主循环周期 | 20 ms |

## MAVLink 消息

### 接收消息

| 消息 | 作用 |
| --- | --- |
| `SET_POSITION_TARGET_LOCAL_NED` | 启用 x/y/yaw 字段时作为位置指令；启用 vx/vy/yaw_rate 字段时作为速度指令 |
| `COMMAND_LONG` | 处理解锁/上锁、里程计复位、舵机、直接轮速、相对运动和 PID 更新指令 |
| `PARAM_REQUEST_READ` | 返回单个 PID 参数 |
| `PARAM_REQUEST_LIST` | 返回全部 PID 参数 |
| `PARAM_SET` | 更新单个 PID 参数并返回新值 |

### 服务

| 服务 | 作用 |
| --- | --- |
| `/set_speed_pid` | 设置速度 PID 的 `kp/ki/kd`，并写入 NVS |
| `/get_speed_pid` | 获取当前速度 PID 参数 |

### COMMAND_LONG 指令

| 指令 | 参数 | 结果 |
| --- | --- | --- |
| `MAV_CMD_COMPONENT_ARM_DISARM` | `param1 < 0.5`：锁存急停并停车；否则释放急停（之后仍需一条零速命令重新武装） | `ACCEPTED` |
| `MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS` | 无 | 请求复位里程计，由控制任务在下一个 20 ms 周期执行 | `ACCEPTED` |
| `MAV_CMD_DO_SET_SERVO` | `param2`：舵机角度 | 写入 `q_servo_cmd` |
| `MAV_CMD_DO_SET_ACTUATOR` | `param1`：左轮目标，`param2`：右轮目标 | 轮速模式 |
| `MAV_CMD_USER_2` | `param1`：相对距离，`param2`：相对航向角 | 相对运动模式 |
| `MAV_CMD_USER_4` | `param1`：PID 目标，`param2`：kp，`param3`：ki，`param4`：kd | 更新速度或位置 PID |

自定义指令 ID：

| 名称 | 值 | 含义 |
| --- | --- | --- |
| `MATURO_MAV_CMD_MOVE_RELATIVE` | `MAV_CMD_USER_2` | 相对运动 |
| `MATURO_MAV_CMD_SET_PID` | `MAV_CMD_USER_4` | 设置 PID 参数 |

PID 目标值：

| 值 | 目标 |
| --- | --- |
| `1` | 速度 PID |
| `2` | 位置 PID |

### 参数

MAVLink 参数协议暴露以下参数：

| 参数 | 类型 | 含义 |
| --- | --- | --- |
| `SPD_KP` | `MAV_PARAM_TYPE_REAL32` | 速度 PID kp |
| `SPD_KI` | `MAV_PARAM_TYPE_REAL32` | 速度 PID ki |
| `SPD_KD` | `MAV_PARAM_TYPE_REAL32` | 速度 PID kd |
| `POS_KP` | `MAV_PARAM_TYPE_REAL32` | 位置 PID kp |
| `POS_KI` | `MAV_PARAM_TYPE_REAL32` | 位置 PID ki |
| `POS_KD` | `MAV_PARAM_TYPE_REAL32` | 位置 PID kd |

### 发布消息

有对应源数据时每 20 ms 发布：

| 消息 | 内容 |
| --- | --- |
| `ATTITUDE` | roll/pitch/yaw 与陀螺仪，单位 rad |
| `ATTITUDE_QUATERNION` | 四元数与陀螺仪，单位 rad/s |
| `SCALED_IMU` | 加速度单位 mg，陀螺仪单位 mrad/s，磁力计为 0 |
| `ODOMETRY` | 本地 FLU 位姿与机体 FRD 速度 |

每 100 ms 发布：

| 消息 | 内容 |
| --- | --- |
| `DISTANCE_SENSOR` | 超声波距离，2-400 cm |
| `DEBUG` | index `1`，运动忙状态以 `0.0` 或 `1.0` 表示 |
| `RAW_RPM` | index `0`：左轮 RPM；index `1`：右轮 RPM |

每完成一帧雷达扫描后发布：

| 消息 | 内容 |
| --- | --- |
| `OBSTACLE_DISTANCE` | 360 度雷达扫描拆分为 5 个数据包；每包 72 个 1 度 bin，`angle_offset` 分别为 `0/72/144/216/288` 度 |

每 1 s 发布：

| 消息 | 内容 |
| --- | --- |
| `HEARTBEAT` | 地面车心跳 |
| `STATUSTEXT` | `hostname=<device_name> ip=<sta_ipv4>` |
| `ONBOARD_COMPUTER_STATUS` | 堆内存、Flash、Wi-Fi 链路估计、有效时的板载温度 |
| `SYS_STATUS` | 电池电压，单位 mV；剩余电量百分比 |
| `BATTERY_STATUS` | 锂电池电压与剩余电量百分比 |

首次看到 MAVLink 客户端后发布一次：

| 消息 | 内容 |
| --- | --- |
| `COMPONENT_INFORMATION_BASIC` | vendor `Maturo`，model `Driver`，software `v1.4`，hardware `ESP32`，serial 为设备名 |

MAVLink 发布行为遵循当前启用的通信模式和对应传输方式。

### 电池上报

电池状态基于 GPIO3 ADC 采样：

| 字段 | 值 |
| --- | --- |
| 满电电压 | `8.4 V` |
| 空电估计电压 | `6.0 V` |
| 分压比 | `8.4 / 2.6857` |
| 百分比 | 根据 6.0 V 到 8.4 V 线性估算 |

ADC 采集任务每 `500 ms` 写入独立的 `sample_seq` 和 MCU 单调 `sample_time_ms`。HTTP `/api/status` 的 `battery.safety` 对象提供互锁原因、新鲜度、恢复计数、武装状态、运动许可和跳闸计数；详细语义见 [电池运动互锁](power_motion_interlock.md)。

MAVLink 电池字段：

| 消息 | 字段 | 值 |
| --- | --- | --- |
| `SYS_STATUS` | `voltage_battery` | 电池电压，单位 mV；无效时为 `UINT16_MAX` |
| `SYS_STATUS` | `current_battery` | `-1`，表示未知 |
| `SYS_STATUS` | `battery_remaining` | `0..100`，无效时为 `-1` |
| `BATTERY_STATUS` | `battery_function` | `MAV_BATTERY_FUNCTION_ALL` |
| `BATTERY_STATUS` | `type` | `MAV_BATTERY_TYPE_LIPO` |
| `BATTERY_STATUS` | `voltages[0]` | 电池电压，单位 mV |
| `BATTERY_STATUS` | `current_battery/current_consumed/energy_consumed` | `-1`，表示未知 |
| `BATTERY_STATUS` | `charge_state` | 正常；低电量为 <=20%；严重低电量为 <=10% |

## 模式切换说明

只有当前启用的模式会发送通信数据：

| 当前模式 | 行为 |
| --- | --- |
| `micro_ros` | 创建并运行 micro-ROS 实体；MAVLink UDP 和 MAVLink UART 不发布、不接收 |
| `mavlink_udp` | MAVLink UDP 任务接收和发布；micro-ROS 与 MAVLink UART 不工作 |
| `uart_mavlink` | MAVLink UART 任务接收和发布；micro-ROS 与 MAVLink UDP 不工作 |

网页运行配置会将通信模式和 Agent 查找方式保存到 NVS。自动发现模式不会保存解析到的
Agent IP；手动模式才会保存用户填写的 IP 与端口。

## BOOT 键：急停与回到配网模式

按下 BOOT 键会立即锁存急停（见 micro-ROS 订阅话题一节），需要上位机显式释放。

运行时长按 BOOT 键约 3 秒会清除已保存的 Wi-Fi STA 配置，状态灯进入快闪。松开 BOOT 键后设备会重启，并进入 Wi-Fi 配网模式。
