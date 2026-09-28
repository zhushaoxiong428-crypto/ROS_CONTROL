# leap_ros_mcu_driver

软件版本：`v1.5`

`leap_ros_mcu_driver` 是 Leap_ROS 底盘的 ESP32-S3 下位机驱动固件，用于电机控制、里程计、IMU、雷达、超声波、电池状态、状态灯和外设数据采集。固件支持 micro-ROS 与 MAVLink 通信模式，并提供 Wi-Fi 配网页用于运行参数配置。

## v1.5 更新内容

- 支持通过服务通讯修改速度 PID 参数，并提供获取 PID 参数服务。
- 超声波改为独立 HC-SR04，TRIG 使用 `GPIO21`，ECHO 使用 `GPIO47`。
- micro-ROS 新增 `/ultrasonic` 话题，发布超声波距离数据。
- 支持 MAVLink UDP、MAVLink UART 与 micro-ROS 通信模式切换。
- 支持长按 BOOT 键清除 Wi-Fi 配置，并重启回到配网模式。

## 底盘参数

- `kTrackWidth` 已修改为 `131.7 mm`，源码位置：`main/control/motion_controller.cpp`。

## 直线起步同步控制

当前开发分支针对左右轮起步响应不一致增加了两层控制：

- 编码器反馈的起步协调：未克服静摩擦的车轮获得有限 PWM 托底；若另一轮已经起转，会限制快轮继续加速，直到两轮均沿指令方向达到起转阈值。
- 仅直行速度模式启用的交叉耦合 PI：根据左右轮跟踪误差之差，对两轮施加等大反向的 PWM 微调；转弯、单轮控制、停车和换向过程不启用。

启动超过 `300 ms` 仍未检测到两轮正常起转时，控制器会锁存启动故障并制动。发送零速度指令后才允许重新启动。设计、基线数据、参数和实车验收流程见 [直线起步标定与验证](doc/motor_startup_tuning.md)。

## 电池运动互锁

运动输出默认闭锁。固件必须先看到连续 3 个独立、位于 `6.4–8.5 V` 的 2S 电池样本，再收到一条新的全车零速命令，才允许后续运动。`<=6.0 V`、`>8.6 V`、ADC 无效或样本超过 `1200 ms` 未更新都会立即清空目标并制动；电压恢复不会自动继续旧命令。状态机、诊断字段和架空验收步骤见 [电池运动互锁](doc/power_motion_interlock.md)。

## 构建与测试

先加载 ESP-IDF 5.5 环境，再构建固件：

```bash
. ~/esp/esp-idf/export.sh
idf.py build
```

不连接硬件也可以运行轮对控制、电池互锁状态机和日志分析器测试：

```bash
cmake -S tests -B /tmp/leap-host-tests
cmake --build /tmp/leap-host-tests
ctest --test-dir /tmp/leap-host-tests --output-on-failure
```

生成物为 `build/leap_low_v1.bin` 和 `build/merged-binary.bin`。

## 协议说明

MAVLink 与 micro-ROS 的消息、话题和参数说明见 [doc/protocols.md](doc/protocols.md)。
