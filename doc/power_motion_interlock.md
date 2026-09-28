# 电池运动互锁

## 目的与边界

电池 ADC 的 `valid` 只表示一次采集至少得到有效 ADC 结果，不能表示动力电源适合驱动车轮。例如 USB 供电但电机电池关闭时，板上曾测得约 `3.63 V`，该值在 ADC 层仍然是有效读数。电池运动互锁因此独立维护“样本有效”“样本新鲜”“电压已恢复”“操作员已重新武装”四类状态，并在 `motion_task` 和最终 PWM 输出处做两层门控。

互锁不代替硬件断电、保险丝、驱动器过流保护或 `500 ms` 速度命令看门狗。

## 状态与门限

| 条件 | 动作 |
| --- | --- |
| 启动后还没有独立 ADC 样本 | 保持制动，原因 `no_sample` |
| ADC 无效或电压非有限 | 立即撤销许可，原因 `adc_invalid` |
| `V <= 6.0 V` | 立即撤销许可，原因 `undervoltage` |
| `V > 8.6 V` | 立即撤销许可，原因 `overvoltage` |
| 距最后一个独立样本超过 `1200 ms` | 立即撤销许可，原因 `stale` |
| 故障后连续 3 个不同序号的样本位于 `6.4–8.5 V` | 电压恢复，但仍保持制动，原因 `needs_zero_rearm` |
| 恢复后新收到 mode 0 全车零速命令 | 清空旧目标并武装；该命令本身不产生运动 |
| 武装后再收到新的运动命令 | 允许进入原运动控制路径 |

电池任务每 `500 ms` 采样一次并递增 `sample_seq`，所以 3 个独立恢复样本约跨越 `1 s`。运动任务每 `20 ms` 读取单槽队列，但相同序号不会重复累计。已经处于安全运行状态时使用 `(6.0, 8.6] V` 作为保持区间；恢复区间收窄到 `[6.4, 8.5] V`，用于避免负载撤除后在临界电压附近反复启停，同时给满电 2S 电池的 ADC 与分压电阻误差留出约 `0.1 V` 余量。

恢复电压不会自动恢复运动。持续非零发布者、故障前的旧目标以及故障前发送的零命令都不能重新武装；必须在恢复完成以后再收到一条新的 mode 0 全车零速命令。位置、相对运动、单轮和双轮模式都受同一入口门控，单轮零速不算全车重新武装。

## 运行时诊断

串口只在互锁状态转换时打印摘要，例如：

```text
Battery motion interlock: reason=needs_zero_rearm ... ready=1 armed=0 allowed=0 ...
Battery motion interlock: reason=none ... ready=1 armed=1 allowed=1 ...
```

网页状态接口保留原电池字段，并增加 `battery.safety`：

```bash
curl -fsS http://小车IP/api/status | python3 -c \
  'import json,sys; print(json.load(sys.stdin)["battery"]["safety"])'
```

关键字段为 `reason`、`sample_fresh`、`voltage_ready`、`armed`、`motion_allowed`、`recovery_samples`、`sample_age_ms` 和 `trip_count`。这里的 `armed` 只表示电池互锁已在恢复后收到全车零速命令，不是 MAVLink 飞控语义中的 vehicle armed。`/motor_debug` 仍保持 19 字段，旧实验数据与分析工具不受影响。

`/battery_state` 在 ADC 无效或互锁认为样本陈旧时发布 `present=false`、`power_supply_health=UNKNOWN`；`<=6.0 V` 映射为 `DEAD`，`>8.6 V` 映射为 `OVERVOLTAGE`，安全范围内为 `GOOD`。运动许可仍应以 `battery.safety.motion_allowed` 为准，不能只看 `BatteryState.health`。

## 命令使用变化

每次 MCU 启动或电池故障恢复后，手工控制前先发全车零速：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

确认 `motion_allowed=true` 后，下一条非零命令才会执行。`tools/run_motor_startup_trial.py` 已自动加入 `300 ms` 零速重新武装阶段，并把事件标记为 `interlock_rearm`；该阶段不会污染正式停车延迟的计时。

## 验证顺序

1. 先运行主机测试，确认状态机边界、重复序号、超时和 32 位时间回绕均通过。
2. 首次刷入后保持车轮架空。只用 USB、关闭动力电源时，持续非零命令必须被拒绝，左右目标和 PWM 必须保持 `0`。
3. 打开正常 2S 动力电源，等待 3 个独立样本；此时应为 `needs_zero_rearm`，非零命令仍不得启动。
4. 发送一条全零命令，确认 `motion_allowed=true`；再发送短时低速命令，确认两轮方向和主动零速停车正常。
5. 架空低速运动时关闭动力电源，从 MCU 观察到低压样本起，目标/PWM 应在一个 `20 ms` 控制周期内归零。保持非零发布到电压恢复也不得自动重启。
6. 不要为了验证上限给真实 2S 电池施加超过 `8.6 V`；过压边界只在单元测试或受控 ADC 注入中验证。
