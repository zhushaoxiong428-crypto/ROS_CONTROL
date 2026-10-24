# leap_ros

软件版本：`v2.1`

[![CI](https://github.com/zhushaoxiong428-crypto/ROS_CONTROL/actions/workflows/ci.yml/badge.svg)](https://github.com/zhushaoxiong428-crypto/ROS_CONTROL/actions/workflows/ci.yml)

`leap_ros` 是 Leap_ROS 底盘的 ESP32-S3 下位机驱动固件，用于电机控制、里程计、IMU、雷达、超声波、电池状态、状态灯和外设数据采集。固件支持 micro-ROS 与 MAVLink 通信模式，并提供 Wi-Fi 配网页用于运行参数配置。

## 项目来源

本仓库基于上游项目 [czu963889306-dev/leap_ros_mcu_driver](https://github.com/czu963889306-dev/leap_ros_mcu_driver)（`v2.1` 之后的 `78389aca`）。在上游基础上的改动：

- **运动安全**：速度指令超时停车；急停改为锁存，之前它从未被置位（见下文“运动安全”）。
- **Agent 自动发现**：Agent IP 填 `auto` 时，小车在本网段自动找到 micro-ROS Agent，电脑 IP 变化后也能自动重连（见 [doc/protocols.md](doc/protocols.md)）。连不上 Agent 时串口会给出告警，不再静默重试。
- **可复现构建**：micro-ROS 组件移到 `components/`，锁文件不再依赖作者机器的绝对路径；新增 `tools/prepare_build.sh`。
- **测试与 CI**：主机单元测试改用 CMake/CTest 管理；GitHub Actions 自动编译固件并运行主机测试。

## v2.1 更新内容

- 支持通过服务通讯修改速度 PID 参数，并提供获取 PID 参数服务。
- 超声波改为独立 HC-SR04，TRIG 使用 `GPIO21`，ECHO 使用 `GPIO47`。
- micro-ROS 新增 `/ultrasonic` 话题，发布超声波距离数据。
- 支持 MAVLink UDP、MAVLink UART 与 micro-ROS 通信模式切换。
- 支持 micro-ROS UART 模式：复用 MAVLink UART 的 UART0，921600 波特率，系统启动 5 秒后连接 Agent。
- 支持长按 BOOT 键清除 Wi-Fi 配置，并重启回到配网模式。
- 接入 I²C 手柄控制：左摇杆 Y 控制前后，右摇杆 X 控制转向；断连后自动发送零速指令。

## 底盘参数

- 两轮差速底盘：轮径 `65 mm`、轮距 `131.7 mm`、每圈编码器计数 `4680`，集中在 `main/config/motion_config.h`。
- 保留现有 Camsense 雷达（115200 波特率）、电池 ADC、HC-SR04、单舵机、I²C 手柄和四种通信模式。
- 目录组织参考 `leap_ros_four`；硬件参数和协议保持本工程配置，不包含机械臂运动学、机械臂任务或机械臂 ROS 接口。

## 代码结构

```text
main/
├── app_main.cpp / app_runtime.cpp     # 入口、初始化和任务创建
├── config/                           # 编译期配置及 PID 持久化
│   ├── board_config.h                # GPIO、I²C、ADC、外设采样周期
│   ├── motion_config.h               # 两轮几何、编码器、PID 默认值、控制周期
│   ├── network_config.h              # Wi-Fi、NVS 键、串口、端口、启动等待
│   ├── ros_config.h                  # ROS 节点、话题、服务、frame、发布周期
│   └── pid_config.h / pid_config.cpp # 速度 PID 的 NVS 读取和保存
├── core/system_globals.*             # flags / runtime / queues / control 共享状态
├── platform/board.*                  # 硬件对象及初始化
├── control/
│   ├── differential_kinematics.*     # 两轮差速正逆运动学，轮序为左、右
│   └── motion_controller.*           # PID、里程计、位置和相对运动控制
├── network/wifi_app.*                # Wi-Fi 连接及运行配置持久化
├── msg/                              # 任务间消息
└── tasks/
    ├── control/ / sensors/ / io/
    └── network/
        ├── microros/                 # task、runtime、state、transport、发布、订阅、服务
        ├── mavlink/                  # 共用协议处理、UDP 和 UART 任务
        └── provision/                # 任务入口、HTTP/DNS、独立 HTML 页面
```

共享状态通过 `system_globals()` 访问，例如 `system_globals().queues.motion_cmd`、`system_globals().runtime.wifi_comm_mode`。主机信息的临界区锁仍保留。

## 常用配置位置

| 修改内容 | 文件 | 配置项 |
| --- | --- | --- |
| micro-ROS UART 启动等待 | `main/config/network_config.h` | `kMicroRosUartStartupDelayMs`，默认 `5000`；设为 `10000` 即等待 10 秒 |
| Agent 默认地址和端口 | `main/config/network_config.h` | `kDefaultMicroRosAgentIp`（默认 `auto`，自动发现）、`kDefaultMicroRosAgentPort` |
| micro-ROS / MAVLink 串口波特率 | `main/config/network_config.h` | `kMicroRosUartBaudRate` / `kMavlinkUartBaudRate` |
| 雷达串口和波特率 | `main/config/network_config.h` | `kLidarUartPort`、`kLidarBaudRate` |
| 电机、编码器、传感器引脚 | `main/config/board_config.h` | 对应的 `kMotor*`、`kEncoder*`、`kUltrasonic*` 等 |
| 轮径、轮距、编码器方向 | `main/config/motion_config.h` | `kWheelDiameterMm`、`kTrackWidthMm`、`kEncoderSigns` |
| 速度 PID 默认值 | `main/config/motion_config.h` | `kDefaultSpeedKp`、`kDefaultSpeedKi`、`kDefaultSpeedKd` |
| ROS 名称及发布周期 | `main/config/ros_config.h` | `kNodeName`、`k*Topic`、`k*Service`、`kPublishPeriodMs` |

启动等待只作用于 micro-ROS UART 模式，从通信任务启动时开始计时。编译期参数修改后需要重新编译、烧录。已保存的 Wi-Fi、通信模式、Agent 地址和速度 PID 仍优先使用 NVS 配置；micro-ROS 本地 UDP 端口继续支持 `CONFIG_MICRO_ROS_LOCAL_PORT`。

## 构建与验证

使用 ESP-IDF `v5.5.3`。clone 或切换分支后先运行一次 `tools/prepare_build.sh`：micro-ROS 库是预编译好随仓库提供的，但 make 按文件时间戳判断要不要重新编译，刚 checkout 的时间戳是乱序的，可能触发需要联网的完整源码编译。

```bash
tools/prepare_build.sh
idf.py build
```

固件为 `build/leap_ros.bin`，整包为 `build/merged-binary.bin`。

不依赖硬件的逻辑（差速运动学、指令超时等）有主机单元测试，无需连接开发板：

```bash
cmake -S tests -B build-tests
cmake --build build-tests
ctest --test-dir build-tests --output-on-failure
```

每次 push 和 PR 都会由 GitHub Actions（[.github/workflows/ci.yml](.github/workflows/ci.yml)）编译固件并运行这些测试，编译出的固件可以在 Actions 页面下载。

## 运动安全

- **指令超时**：速度类指令（`/cmd_vel`、MAVLink 速度和轮速指令）超过 `500 ms` 没有更新就自动停车，阈值为 `motion_config.h` 中的 `kCommandTimeoutMs`。
- **急停锁存**：MAVLink `MAV_CMD_COMPONENT_ARM_DISARM` 上锁（`param1 = 0`）后急停锁存，此时 MAVLink 运动指令返回 `DENIED`，手柄和 `/cmd_vel` 也无法驱动底盘，只有解锁（`param1 = 1`）能解除。

上板验证应覆盖前进/后退/原地转向、手柄接管、四种通信模式、ROS 电池和超声波话题，以及配网和 PID 保存重启。

## 协议说明

MAVLink 与 micro-ROS 的消息、话题和参数说明见 [doc/protocols.md](doc/protocols.md)。
