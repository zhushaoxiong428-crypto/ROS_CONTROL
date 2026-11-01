# leap_mavlink_bridge

Linux 侧通过串口直接和固件对话的 MAVLink 桥接程序，不依赖 micro-ROS 和 Agent。目前是命令行程序：每秒打印一次里程计、到达频率和链路统计，也可以让小车按固定速度行驶。之后会在它的基础上封装成 ROS 2 节点。

## 固件侧准备

1. 在配网页把通信模式切到 **MAVLink UART**（`uart_mavlink`，见 [doc/protocols.md](../../doc/protocols.md#mavlink-uart)）。
2. USB 线接开发板的 **UART0 口**（GPIO43/44，经 USB 转串口芯片，通常是 `/dev/ttyUSB0` 或 `/dev/ttyACM0`），不是 USB-Serial-JTAG 口。参数为 230400、8N1。
3. 当前用户需要串口权限：`sudo usermod -aG dialout $USER`，重新登录后生效。
4. Ubuntu 22.04 自带的 `brltty` 会抢占 CH340 这类 USB 转串口芯片，表现为 `/dev/ttyUSB0` 插上后马上消失：`sudo apt remove brltty`。

UART0 同时也是日志控制台，MAVLink 帧和日志文本混在一起。解析器会跳过日志文本，原因是 ASCII 字符里不会出现 MAVLink 起始字节（0xFD/0xFE）。

## 编译与测试

```bash
cmake -S host/mavlink_bridge -B build-bridge
cmake --build build-bridge
ctest --test-dir build-bridge --output-on-failure
```

## 运行

```bash
# 只接收，不发任何指令
build-bridge/leap_mavlink_bridge -d /dev/ttyUSB0

# 以 0.1 m/s 前进 3 秒，然后发停车指令并退出（先把轮子架空）
build-bridge/leap_mavlink_bridge -d /dev/ttyUSB0 --vx 0.1 -t 3
```

输出示例：

```text
odom  49.6 Hz [ 20.0/ 20.1/ 20.5 ms]  x=  0.276 y=  0.000 yaw=    0.0 deg  vx= 0.100 wz= 0.000 | frames 125 lost 1 crc 0 hb 2
```

| 字段 | 含义 |
| --- | --- |
| `odom Hz [min/mean/max]` | 本秒内 ODOMETRY 的到达频率和到达间隔，用来和 micro-ROS 的 `/odom` 对比（`tools/topic_rates.py`） |
| `x y yaw vx wz` | 最新一帧里程计 |
| `frames` | 累计收到的完整帧 |
| `lost` | 按固件发送序号的间隔推算出的丢帧数，包括固件 TX 缓冲区满时没发出去的帧 |
| `crc` | 校验失败的帧数 |
| `hb` | 心跳数，固件每秒发一次 |

加 `--csv FILE` 会把每一帧里程计记下来（PC 接收时刻、固件时间戳、速度、累计帧数 / 丢帧 / CRC 错误），退出时打印整次运行的到达间隔分位数：

```text
odom gaps: n 582  p50 20.0  p95 20.0  p99 20.1  max 40.0 ms
```

`python3 tools/bridge_log_stats.py FILE` 把 PC 接收间隔和固件发布间隔分开统计，用来判断一次迟到是固件发晚了还是链路上耽误了。固件时间戳来自 FreeRTOS tick，当前 tick 为 100 Hz，所以它的分辨率是 10 ms。

速度指令用 `SET_POSITION_TARGET_LOCAL_NED`，单位和符号与 `/cmd_vel` 相同（`vx` 单位 m/s，`wz` 单位 rad/s，逆时针为正），以 20 Hz 发送。固件 500 ms 收不到指令就会停车，所以本程序退出或被杀掉后小车会自己停下。

## 设计

单线程，所有事件都由一个 `epoll` 等待：

| fd | 作用 |
| --- | --- |
| 串口（非阻塞，`termios` raw 模式） | 收到字节就送进 `MavlinkLink::Feed` |
| `timerfd` 50 ms | 发速度指令（仅在传了 `--vx`/`--wz` 时创建） |
| `timerfd` 1 s | 打印统计 |
| `signalfd` | 把 Ctrl-C 变成普通事件，退出路径上能先发停车指令 |

打开串口时会同时释放 DTR 和 RTS，并清掉 `HUPCL`。ESP32 开发板用这两根线控制 EN/IO0，处理不当，开关串口时芯片就会被复位。

| 文件 | 内容 |
| --- | --- |
| `include/leap_bridge/serial_port.h` | 串口打开、配置和读写 |
| `include/leap_bridge/mavlink_link.h` | 字节流和 MAVLink 消息的互相转换，统计丢帧和 CRC 错误 |
| `include/leap_bridge/rate_stats.h` | 到达频率和间隔统计 |
| `src/main.cpp` | 命令行参数和事件循环 |
| `test/` | 主机单元测试，不需要硬件 |

## 验证状态

- 单元测试：日志与帧混传、7 字节分片输入、CRC 损坏、丢帧和序号回绕。
- PTY 模拟固件端到端联调：50 Hz，人为丢掉的帧被 `lost` 统计到。
- **实车联调：进行中**，按下表进行，原始日志放进 `data/bridge_hw_<日期>/`。

| 编号 | 场景 | 命令 / 操作 | 看什么 |
| --- | --- | --- | --- |
| H1 | 只听，静止 10 min | `-t 600 --csv listen_idle.csv` | 频率、`lost`、`crc`、间隔 p99 / 最大值 |
| H2 | 架空行驶 60 s | `--vx 0.1 -t 60 --csv drive_lifted.csv` | 指令下发后的速度、退出时是否立即停车 |
| H3 | 行驶中拔掉 USB | 运行 H2，中途拔线 | 程序是否报 `device gone` 退出；固件是否靠 500 ms 看门狗停车 |
| H4 | 行驶中复位 MCU | 运行 H2，中途按 EN 键 | 串口桥是否继续工作；复位前后的 `lost` 和固件时钟回退 |
| H5 | 地面失联停车 | 地面 0.1 / 0.2 m/s 行驶，`kill -9` 串口桥（不发停车指令） | 停车时间和停车距离（卷尺），每档 3 次 |

## 下一步（第 2 周）

- [ ] 完成上表 H1–H5，结果写进 `doc/`
- [ ] 断开后自动重连，重连期间保持安全状态
- [ ] 加 ROS 2 节点：订阅 `/cmd_vel`，发布 `/odom` 和 TF，复用 `leap_bridge` 库
- [ ] 和 micro-ROS 对比 `/odom` 的频率、间隔抖动和上位机 CPU 占用，数据放进 `data/`，结论写进 `doc/`
- [ ] udev 规则把串口固定为 `/dev/leap_mcu`
