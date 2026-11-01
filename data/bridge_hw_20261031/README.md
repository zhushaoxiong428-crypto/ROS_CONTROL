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

- `listen_idle_60s.csv/.txt`: 230400 baud
- `listen_idle_60s_921600.csv/.txt`: 921600 baud, after the baud change
- `listen_idle_60s_921600_nolog.csv/.txt`: 921600 baud, periodic log reports off
