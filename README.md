# leap_ros_mcu_driver

[![CI](https://github.com/zhushaoxiong428-crypto/ROS_CONTROL/actions/workflows/ci.yml/badge.svg)](https://github.com/zhushaoxiong428-crypto/ROS_CONTROL/actions/workflows/ci.yml)

ESP32-S3 差速底盘下位机固件（FreeRTOS + micro-ROS / MAVLink）：电机闭环、里程计、IMU、激光雷达、超声波、电池监测，经 Wi-Fi 与 ROS 2 通信。

> 基于开源项目 [LEAP ROS2](https://github.com/czu963889306-dev/leap_ros2) 的下位机固件
> [leap_ros_mcu_driver](https://github.com/czu963889306-dev/leap_ros_mcu_driver)（v1.5）二次开发，保留上游提交历史。
> 以下是在上游基础上完成的工作。

## 主要工作

| 方向 | 内容 | 文档 |
| --- | --- | --- |
| 直行偏航 | 定位出左右轮 5.4% 的行程比例差并标定；加入 IMU 航向保持。1 m 直行航向误差从 14° 降到约 1–2° | [直行航向保持](doc/heading_hold.md) |
| 起步控制 | 分轮前馈（悬空实验拟合）、起步托底与 300 ms 线性释放、直行交叉耦合 PI、起步故障锁存 | [起步标定与验证](doc/motor_startup_tuning.md) |
| 安全 | 电池运动互锁（欠压/过压/采样失效即制动，恢复需零速重新武装）；锁存式急停（ROS 话题 / MAVLink / BOOT 键） | [电池运动互锁](doc/power_motion_interlock.md)、[协议](doc/protocols.md) |
| 并发正确性 | 消除通信任务与控制任务跨核竞争；命令队列改为有序 FIFO，避免零速武装命令被覆盖 | 提交记录 |
| 通信链路 | 遥测 QoS A/B 实验与发布耗时探针；LaserScan 下采样到 320 点避免 IP 分片，`/motor_debug` 主机接收率 52% → 94% | [起步标定与验证](doc/motor_startup_tuning.md#遥测发布-qos-ab-实验) |
| 实时性 | 控制周期抖动、各任务栈余量与 CPU 占用的在线统计；统计输出放在最低优先级任务，避免测量本身引入抖动 | [实时性实测](doc/realtime.md) |
| 工程化 | 12 个主机单元测试（控制器、状态机、日志分析）；GitHub Actions 编译固件并跑测试；修复上游仓库在作者机器以外无法编译的问题 | [CI](.github/workflows/ci.yml) |

## 实测：直行偏航

落地直行 1 m（`tools/straight_drive_test.py`），IMU 航向为车头实际转角，侧向偏移取自里程计（比例标定后里程计与 IMU 一致）：

| 配置 | 速度 | IMU 航向变化 | 侧向偏移 |
| --- | --- | ---: | ---: |
| 上游控制（仅编码器同步） | 0.05–0.2 m/s | 各速度均明显左偏；由比例差推算约 20° / 0.9 m | — |
| + 左右轮比例标定 | 0.1 m/s | +14.2° | +18.4 cm |
| | 0.2 m/s | +2.8°（峰值 +6.0°） | +11.3 cm |
| + 比例标定 + IMU 航向保持 | 0.1 m/s | +1.2° | −3.3 cm |
| | 0.2 m/s | −2.0° | −1.1 cm |

（+ 为向左；每个配置各 1 次试验。）

定位过程：开启航向保持后车头走直了，里程计却推算出约 20° 的右转，说明右轮每个编码器脉冲实际走得比左轮远约 5.4%；
0.1 与 0.2 m/s 下比值为 1.0531 / 1.0553，与速度无关，属于减速比或轮径差而非打滑。标定后比值为 1.0005。
只做标定时，匀速段航向已经不再变化，但起步约 4 s 内仍累积约 14° 左偏且无法回补，因此需要航向闭环兜底。

## 实测：实时性

静止、micro-ROS 遥测正常发布时（详见 [实时性实测](doc/realtime.md)）：

| 指标 | 实测 |
| --- | --- |
| 20 ms 控制周期 | 平均 20.000 ms，标准差 0.066 ms，最大抖动 ±0.43 ms，500 个周期 0 次超限 |
| 控制循环执行时间 | 平均 0.50 ms，最大 1.96 ms |
| CPU 负载 | core0 15.6%，core1 33.8%（控制任务 2.5%，micro-ROS 24%） |
| 栈余量 | 最紧的 `motion` 任务使用 69%（4096 B 中剩 1268 B） |

## 系统结构

```text
imu_task ──────┐                              ┌── micro-ROS / MAVLink 通信任务
battery_task ──┼─> 单槽状态队列 ──> motion_task (20 ms, vTaskDelayUntil) <── 运动命令 FIFO
lidar_task ────┘                     │  电池互锁 → 急停门控 → 模式分发
                                     │  MotionController：速度环 PI + 前馈
                                     │   ├─ WheelPairController（起步协调 / 同步 PI）
                                     │   └─ HeadingHold（IMU 航向保持）
                                     └─> 电机 PWM、里程计、/motor_debug 遥测
```

控制相关的纯逻辑（`WheelPairController`、`BatteryMotionInterlock`、`HeadingHold`、`EmergencyStopGate`）不依赖 ESP-IDF，可在主机上单元测试。

## 构建与测试

需要 ESP-IDF 5.5。克隆后首次构建前运行一次 `tools/prepare_build.sh`：它把 `dependencies.lock` 中 micro-ROS 组件的绝对路径改为本机路径，并修正预编译库的时间戳，避免触发需要 colcon 和联网的 micro-ROS 完整重编。

```bash
tools/prepare_build.sh
. ~/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

不接硬件运行主机测试：

```bash
cmake -S tests -B /tmp/leap-host-tests
cmake --build /tmp/leap-host-tests
ctest --test-dir /tmp/leap-host-tests --output-on-failure
```

落地直行测试（车放地上，前方留足空间）：

```bash
source /opt/ros/humble/setup.bash
python3 tools/straight_drive_test.py --speed 0.1 --distance 1.0 --confirm-ground
```

## 可配置项

`idf.py menuconfig → Leap low settings`：

| 选项 | 默认 | 说明 |
| --- | --- | --- |
| Hold IMU heading while driving straight | 开 | IMU 航向保持，关闭仅用于 A/B 对比 |
| Right/left wheel travel ratio (×10000) | 10000 | 左右轮行程比标定，本车为 10542 |
| Published LaserScan point count | 320 | `/scan` 点数 |
| Fallback Wi-Fi SSID / password | Maturo / maturo2026 | 未配网时尝试连接的热点 |

## 目录

```text
main/control/        运动控制、轮对控制、航向保持、电池互锁、急停门控
main/tasks/          FreeRTOS 任务（控制、传感器、通信）
components/          电机、编码器（PCNT）、IMU、雷达等驱动
tests/               主机单元测试
tools/               实车试验与日志分析脚本
data/                实车试验数据
doc/                 设计与验证文档、通信协议
```

## 已知问题

- 起步约 2 s 内车头仍会先左偏 3–4° 再被拉回，略有过冲（起步打滑/万向轮转向，编码器不可见）。可为航向保持加入陀螺角速度阻尼。
- 实测每个配置仅 1 次试验；比例标定基于当前轮胎与载荷，更换轮子或负载后需要重新标定。

## 上游 v1.5 功能

速度 PID 参数读写服务、HC-SR04 超声波（TRIG `GPIO21` / ECHO `GPIO47`）与 `/ultrasonic` 话题、MAVLink UDP / UART 与 micro-ROS 模式切换、长按 BOOT 清除 Wi-Fi 配置。轮距 `kTrackWidth = 131.7 mm`。
