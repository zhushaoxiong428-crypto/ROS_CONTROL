#!/usr/bin/env bash
# Command-loss stop test (H5): drive with leap_mavlink_bridge, SIGKILL it so no
# stop command is sent, and record the stop with a second bridge in listen mode.
# Only the firmware's 500 ms command watchdog can stop the robot.
#
# Usage: tools/bridge_kill_stop_test.sh SPEED_MPS DRIVE_S OUT_PREFIX [DEVICE]
#   tools/bridge_kill_stop_test.sh 0.2 2 data/bridge_hw_20261101/h5_0.2_run1
set -euo pipefail

speed=$1
drive_s=$2
out=$3
device=${4:-/dev/ttyUSB0}
bridge=build-bridge/leap_mavlink_bridge

"$bridge" -d "$device" --vx "$speed" --csv "$out.drive.csv" > "$out.drive.txt" 2>&1 &
pid=$!
sleep "$drive_s"
kill_s=$(date +%s.%N)
kill -9 "$pid"
wait "$pid" 2>/dev/null || true
listen_s=$(date +%s.%N)
"$bridge" -d "$device" -t 3 --csv "$out.stop.csv" > "$out.stop.txt" 2>&1
echo "$kill_s $listen_s" > "$out.times"

python3 - "$out" <<'EOF'
import csv, sys
out = sys.argv[1]
kill_s, listen_s = map(float, open(out + ".times").read().split())
drive = list(csv.DictReader(open(out + ".drive.csv")))
stop = list(csv.DictReader(open(out + ".stop.csv")))
# The listener starts a few ms after the kill, so its first frame stands for the
# robot at the moment the command stream stopped. Its host_s counts from its own
# start, about listen_s.
gap = listen_s - kill_s
first = stop[0]
still = next((r for r in stop if abs(float(r["vx_mps"])) < 0.005), None)
x0, y0 = float(first["x_m"]), float(first["y_m"])
xs, ys = float(stop[-1]["x_m"]), float(stop[-1]["y_m"])
dist = ((xs - x0) ** 2 + (ys - y0) ** 2) ** 0.5
total = ((xs - float(drive[0]["x_m"])) ** 2 + (ys - float(drive[0]["y_m"])) ** 2) ** 0.5
print(f"{out}: listener up {gap * 1000:.0f} ms after the kill, speed then {float(first['vx_mps']):.3f} m/s")
if still:
    print(f"  stopped (|vx| < 0.005 m/s) {(gap + float(still['host_s'])) * 1000:.0f} ms after the kill")
else:
    print("  did not stop within the 3 s recording")
print(f"  odometry distance from the kill to rest: {dist * 1000:.0f} mm; whole run {total * 1000:.0f} mm")
EOF
