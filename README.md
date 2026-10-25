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
- **速度控制**：在地面上实测并重新调了速度 PID；另外试过前馈、10 位 PWM 和目标速度斜坡，实测都没有带来改善，所以默认不启用（见 [doc/speed_control.md](doc/speed_control.md)）。
- **正确性与实时性**：编码器溢出改由 PCNT 驱动累加，并处理了驱动里的读数竞态；控制循环改用 `vTaskDelayUntil`、按实测周期计算 `dt`，并每 10 秒打印一次周期统计。
- **实车测试工具**：`tools/` 下的脚本通过 ROS 话题驱动小车并记录数据，`data/` 下保存了每一组实测数据。

## 实测结果

除注明“架空”的两项外，都是小车放在地上实测的数据，原始数据见各行链接。速度控制的完整过程，包括没有效果的尝试和中途修正过的测量错误，见 [doc/speed_control.md](doc/speed_control.md)。

| 项目 | 改动前 | 现在 | 数据 |
| --- | --- | --- | --- |
| 速度稳态误差（0.1 / 0.2 m/s） | -10.8 % / -14.7 % | 1 % 以内 | [基线](data/baseline_20261023_ground)、[调参](data/pid_grid_20261023_ground) |
| 0.1 m/s 阶跃到达 90 % | 2.1 s | 0.18–0.22 s | 同上 |
| 低速（≤ 0.1 m/s）超调 | — | 3.9 %，测量噪声峰值 2.8 % | [调参](data/pid_grid_20261023_ground) |
| `/cmd_vel` 中断后停车（架空） | 不停，一直按最后的指令行驶 | 最后一条指令后 0.71–0.74 s 停稳 | [超时测试](data/baseline_20261023_suspended) |
| 控制周期（目标 20 ms） | 未测量 | 均值 20.000 ms，最大抖动 1.3 ms，无超时 | [周期统计](data/loop_timing_20261023) |
| 编码器跨溢出（架空） | 每次溢出差 1 个计数 | 正反各跨溢出 2 次，速度没有跳变 | [溢出测试](data/encoder_overflow_20261023) |
| 里程计：直行 1.5 m / 原地转 3 圈 | 未标定 | 误差 0.15 % / < 0.2 % | [里程计标定](data/odom_calibration_20261024) |
| micro-ROS Agent 连接 | 需要手动填写电脑 IP | Wi-Fi 连上后 0.12 s 自动找到 | — |

调参的取舍：在 `kp=1.2` 下，`ki=4` 比 `ki=6` 的低速超调低一半多，但 -0.2 m/s 的阶跃要慢约 100 ms（269 对 173 ms）。速度测量的分辨率也有上限：0.05 m/s 时，每个 20 ms 周期只有约 23 个编码器计数，1 个计数就相当于 4 % 的速度。

**试过但没有采用的方案**（代码保留，默认关闭）：

- **速度前馈**：按实测电机曲线直接给出 PWM，只有大幅阶跃变快了；电机加速期间积分项持续累积，0.1 m/s 时反而出现 8–11 % 的超调。
- **10 位 PWM**：没有改善，低速超调还略高（每次 9.9–10.0 %，8 位为 7.0–8.6 %），在实验分支 `exp/pwm-10bit` 上。
- **目标速度斜坡**：纯 PID 的超调本来就在噪声水平，加斜坡测不出差别。

**已知局限**：

- IMU 航向不可靠：同样原地转了 1084°，四次读数在 1073° 到 1114° 之间；直行 15 秒漂移 2–12°。在查清原因之前，不要用它做航向参考。
- 急停锁存还没有在实车上验证；micro-ROS 侧也还不能触发急停，因为订阅名额已经用满。

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
│   ├── motion_controller.*           # PID、里程计、位置和相对运动控制
│   ├── command_watchdog.h            # 速度指令超时
│   ├── cycle_timing_stats.h          # 控制周期统计
│   ├── wheel_feedforward.h           # 速度前馈模型（默认关闭）
│   └── velocity_ramp.h               # 目标速度斜坡（默认关闭）
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
| 速度 PID 默认值 | `main/config/motion_config.h` | `kDefaultSpeedKp`、`kDefaultSpeedKi`、`kDefaultSpeedKd`，默认 `1.2 / 4 / 0` |
| 指令超时 | `main/config/motion_config.h` | `kCommandTimeoutMs`，默认 `500` |
| 速度前馈、目标速度斜坡 | `main/config/motion_config.h` | `kUseSpeedFeedforward`、`kFf*`、`kDefaultVelocityAccelLimitMps2`；运行时可通过 `POST /api/motion-tuning` 临时修改，不保存 |
| ROS 名称及发布周期 | `main/config/ros_config.h` | `kNodeName`、`k*Topic`、`k*Service`、`kPublishPeriodMs` |

启动等待只作用于 micro-ROS UART 模式，从通信任务启动时开始计时。编译期参数修改后需要重新编译、烧录。已保存的 Wi-Fi、通信模式、Agent 地址和速度 PID 仍优先使用 NVS 配置；micro-ROS 本地 UDP 端口继续支持 `CONFIG_MICRO_ROS_LOCAL_PORT`。

## 构建与验证

使用 ESP-IDF `v5.5.3`。clone 或切换分支后先运行一次 `tools/prepare_build.sh`：micro-ROS 库是预编译好随仓库提供的，但 make 按文件时间戳判断要不要重新编译，刚 checkout 的时间戳是乱序的，可能触发需要联网的完整源码编译。

```bash
tools/prepare_build.sh
idf.py build
```

固件为 `build/leap_ros.bin`，整包为 `build/merged-binary.bin`。

不依赖硬件的逻辑有主机单元测试，包括差速运动学、指令超时、Agent 发现的子网计算、控制周期统计、速度前馈和斜坡，无需连接开发板：

```bash
cmake -S tests -B build-tests
cmake --build build-tests
ctest --test-dir build-tests --output-on-failure
```

每次 push 和 PR 都会由 GitHub Actions（[.github/workflows/ci.yml](.github/workflows/ci.yml)）编译固件并运行这些测试，编译出的固件可以在 Actions 页面下载。

## 实车测试工具

脚本通过 ROS 2 话题驱动小车，运行前需要先 `source /opt/ros/humble/setup.bash`，并确保 micro-ROS Agent 已连接。小车会移动，测试前请把轮子架空，或者在地上留出足够空地。

| 脚本 | 用途 |
| --- | --- |
| `tools/cmd_timeout_test.py` | 发送一段 `/cmd_vel` 后停发，测量小车多久停下 |
| `tools/speed_step_test.py` | 速度阶跃响应：t90、超调、噪声峰值、稳态误差，保存为 CSV |
| `tools/summarize_steps.py` | 汇总多次阶跃测试，对比不同参数组合（离线运行，不需要 ROS） |
| `tools/odom_drive_test.py` | 按里程计直行一段距离或转若干圈，同时输出 IMU 航向，用来和卷尺实测对比 |
| `tools/motor_characterization.py` | 开环测量 PWM 与轮速的关系，需要诊断版固件（`sdkconfig.motor_characterization`） |

## 运动安全

- **指令超时**：速度类指令（`/cmd_vel`、MAVLink 速度和轮速指令）超过 `500 ms` 没有更新就自动停车，阈值为 `motion_config.h` 中的 `kCommandTimeoutMs`。
- **急停锁存**：MAVLink `MAV_CMD_COMPONENT_ARM_DISARM` 上锁（`param1 = 0`）后急停锁存，此时 MAVLink 运动指令返回 `DENIED`，手柄和 `/cmd_vel` 也无法驱动底盘，只有解锁（`param1 = 1`）能解除。

上板验证应覆盖前进/后退/原地转向、手柄接管、四种通信模式、ROS 电池和超声波话题，以及配网和 PID 保存重启。

## 协议说明

MAVLink 与 micro-ROS 的消息、话题和参数说明见 [doc/protocols.md](doc/protocols.md)。
