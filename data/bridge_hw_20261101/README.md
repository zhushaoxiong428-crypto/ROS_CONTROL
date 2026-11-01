# Command-loss stop on the ground (H5), 2026-11-01

What happens when the host stops sending velocity commands with no stop command:
only the firmware's 500 ms command watchdog (`CommandWatchdog`) can stop the robot.

The robot had to run untethered, so this was done over micro-ROS / Wi-Fi with
`tools/cmd_timeout_test.py --scenarios stream` (publish `/cmd_vel` at 10 Hz, then go
silent), not over the serial bridge. The firmware path after the last command is the
same. Stop time is from the last published command to the first `/odom` sample below
the threshold, so it includes Wi-Fi latency and ~130 ms of `/odom` velocity filter lag.
Distance is `/odom` speed integrated after the last command.

| Ground, 3 runs each | Stopped, \|vx\| < 0.01 / < 0.005 m/s | Travel after the last command |
| --- | --- | --- |
| 0.1 m/s (runs 1–6) | 714–757 / 778–804 ms | 63–66 mm |
| 0.2 m/s (runs 1–3) | 792–803 / 846–874 ms | 133–135 mm |

Checked against a tape measure over each batch (rear axle, start mark to rest):

| Batch | Odometry | Tape | Difference |
| --- | --- | --- | --- |
| 0.1 m/s, runs 4–6 | 1079 mm | 1080 mm | −0.1 % |
| 0.2 m/s, runs 1–3 | 1471 mm | 1435 mm | +2.5 % |

At 0.1 m/s odometry and tape agree, so the wheels did not slip while braking. At
0.2 m/s odometry reads 2.5 % long, i.e. the wheels turned more than the robot moved:
slip, most likely on the unramped 0.2 m/s start; the data cannot place it in the start
or the stop. Stop distances at 0.2 m/s carry that uncertainty.

0.2 m/s run 3 started ~0.3 s late (a 339 ms `/odom` gap over Wi-Fi at the start, against
58–71 ms in runs 1–2), which explains its shorter total (449 mm against 509–513 mm);
its stop phase is unaffected.

The stop time grows little with speed (~70 ms) while the distance doubles: during the
500 ms timeout the robot keeps its speed, so that part of the distance is speed × 0.5 s.

Files: `h5_ground_<speed>_run<n>.csv` (`scenario,t_since_last_cmd_s,vx_mps`).
`h5_dryrun_lifted_0.1.*` is the serial-bridge version of the same test
(`tools/bridge_kill_stop_test.sh`, bridge killed with SIGKILL), wheels lifted: stopped
742 ms after the kill, 59 mm. `h5_wifi_dryrun_lifted_0.1.csv`: the Wi-Fi version, lifted.
