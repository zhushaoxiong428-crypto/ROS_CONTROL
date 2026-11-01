# leap_mavlink_bridge on the robot, 2026-10-31

Laptop (Ubuntu 22.04) ↔ CH340 USB-serial ↔ ESP32-S3 UART0, firmware in `uart_mavlink`
mode, robot standing still, listen only (`leap_mavlink_bridge -t 60 --csv ...`).
Summaries from `python3 tools/bridge_log_stats.py`.

| | 230400 baud | 921600 baud | 921600, no periodic logs |
| --- | --- | --- | --- |
| Throughput | 23041 B/s = 100 % of the line | 23240 B/s ≈ 25 % | 23020 B/s ≈ 25 % |
| ODOMETRY rate | 49.35 Hz | 49.90 Hz | 50.00 Hz |
| Firmware publish gaps p50 / p95 / p99 / max | 20 / 50 / 60 / 160 ms | 20 / 20 / 20 / 40 ms | 20 / 20 / 20 / 20 ms |
| Host arrival gaps p50 / p95 / p99 / max | 13.9 / 54.2 / 57.0 / 137.7 ms | 19.9 / 22.1 / 22.4 / 40.4 ms | 19.9 / 22.1 / 22.4 / 22.9 ms |
| Lost frames / CRC errors in 60 s | 17 / 14 | 11 / 9 | 0 / 0 |

At 230400 the 50 Hz messages alone (ODOMETRY 245 B, ATTITUDE 40 B,
ATTITUDE_QUATERNION 60 B, SCALED_IMU 36 B with MAVLink 2 framing) need ~19 KB/s of
the 23 KB/s, so the UART TX buffer filled, `uart_write_bytes` blocked the publish task
and frames left the MCU late and in bursts (firmware timestamps already uneven).

The CRC errors left at 921600 come every 10.0 s of firmware time (28.2, 38.2, 48.2,
58.2, 68.3, 78.3 s), the period of the control-loop and rt_stats log reports: log text
written to UART0 by another task lands inside MAVLink frames. With those two reports
skipped in the UART modes (`Uart0CarriesBinaryLink`), 3050 frames in 61 s arrived with
no loss and no CRC error. Other, one-off log lines (warnings) still go to UART0.

## Driving and faults (921600, periodic logs off), wheels lifted

- **H2, drive 0.1 m/s for 60 s** (`h2_drive_lifted_0.1.*`): 3003 ODOMETRY frames at
  50.00 Hz, 0 lost, 0 CRC errors, host gaps max 22.7 ms. Speed after 5 s: mean
  0.1000 m/s, SD 0.0038, range 0.091–0.106. The zero command sent on exit took effect at
  once (`h2_after_exit.*`: speed already falling in the first frame, ~0 within ~0.3 s), with
  no 500 ms plateau of waiting for the watchdog.
- **H3, USB unplugged while driving** (`h3_unplug.*`): 0 lost until the unplug at
  11.4 s; the bridge saw EPOLLHUP, printed `device gone`, failed to send the stop command
  (EIO, the cable was out) and exited with code 1. The wheels stopped by eye "almost at
  once" (the watchdog's ~0.5 s); the MCU did not reset (uptime continuous).
- **H4, MCU reset button pressed twice while driving** (`h4_mcu_reset.*`): the bridge
  kept running through both reboots (warned after 3 s without frames) and resumed with
  the first frame after each. **The robot started driving again on its own after each
  reboot**, because the host keeps streaming commands and the firmware obeys them as
  soon as it is up; after the second reboot it overshot to 0.139 m/s before settling at
  0.1. The first reboot resumed publishing at firmware time 170 ms, without the 3 s IMU
  calibration (the second took the normal 3.2 s): possibly the IMU failing to
  initialize after a reset mid-I2C-transfer; not confirmed, no boot log was captured.
  `lost` (331) is inflated: a reboot restarts the sequence number, which the bridge
  counts as loss; the extra CRC errors are boot log printed at 230400 read at 921600.

- `listen_idle_60s.csv/.txt`: 230400 baud
- `listen_idle_60s_921600.csv/.txt`: 921600 baud, after the baud change
- `listen_idle_60s_921600_nolog.csv/.txt`: 921600 baud, periodic log reports off
