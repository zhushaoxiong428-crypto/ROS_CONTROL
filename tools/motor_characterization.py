#!/usr/bin/env python3
"""Measure each wheel's steady speed against open-loop PWM.

Needs the diagnostic firmware (CONFIG_LEAP_MOTOR_CHARACTERIZATION), which
reads /cmd_vel linear.z / angular.x as raw left / right motor PWM:

  idf.py -B build-diag -D SDKCONFIG=build-diag/sdkconfig \\
      -D "SDKCONFIG_DEFAULTS=sdkconfig;sdkconfig.motor_characterization" \\
      -p /dev/ttyUSB0 flash

For each PWM level both wheels are driven forward, stopped, then driven in
reverse, so the robot ends near where it started; the largest excursion is
one hold at the highest level. Wheel speeds come from /odom:
v_left = vx - wz * W / 2, v_right = vx + wz * W / 2 (W = track width).

Reports, per wheel and direction, the lowest PWM that moves the wheel from
rest and a line v = k * (pwm - p0) fitted above it.

Usage:
  source /opt/ros/humble/setup.bash
  python3 tools/motor_characterization.py --csv /tmp/motor.csv
"""

import argparse
import csv
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState

DEFAULT_LEVELS = "60,80,100,110,120,130,140,150,160,170,180,190,200,220,240,255"


class Characterizer(Node):
    def __init__(self, track_width_m):
        super().__init__("motor_characterization")
        self.half_track = track_width_m / 2.0
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(BatteryState, "battery_state", self.on_battery, qos_profile_sensor_data)
        self.samples = []  # (monotonic time, v_left, v_right)
        self.voltage = float("nan")

    def on_odom(self, msg):
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        self.samples.append((time.monotonic(), vx - wz * self.half_track, vx + wz * self.half_track))

    def on_battery(self, msg):
        self.voltage = msg.voltage

    def send_pwm(self, left, right):
        cmd = Twist()
        cmd.linear.z = float(left)
        cmd.angular.x = float(right)
        self.pub.publish(cmd)

    def send_stop(self):
        self.pub.publish(Twist())

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def hold(self, left, right, seconds, rate):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if left == 0 and right == 0:
                self.send_stop()
            else:
                self.send_pwm(left, right)
            self.spin_for(1.0 / rate)


def fit_line(points):
    """Least squares v = a * pwm + b; returns (k, p0) with v = k * (pwm - p0)."""
    if len(points) < 2:
        return float("nan"), float("nan")
    xs, ys = zip(*points)
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return float("nan"), float("nan")
    a = sum((x - mx) * (y - my) for x, y in points) / sxx
    b = my - a * mx
    return a, (-b / a if a else float("nan"))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--levels", default=DEFAULT_LEVELS, help="comma-separated PWM magnitudes")
    parser.add_argument("--hold-s", type=float, default=1.2, help="time at each PWM")
    parser.add_argument("--steady-s", type=float, default=0.5, help="window at the end of each hold")
    parser.add_argument("--rest-s", type=float, default=0.6, help="stop between holds")
    parser.add_argument("--rate", type=float, default=20.0, help="publish rate in Hz")
    parser.add_argument("--track-width", type=float, default=0.1317, help="metres, kTrackWidthMm / 1000")
    parser.add_argument("--moving", type=float, default=0.01, help="|v| above this counts as moving (m/s)")
    parser.add_argument("--csv", help="per-level results")
    parser.add_argument("--samples-csv", help="every odom sample")
    args = parser.parse_args()

    levels = [int(x) for x in args.levels.split(",")]
    rclpy.init()
    node = Characterizer(args.track_width)
    results, raw = [], []
    try:
        node.spin_for(1.0)
        if not node.samples:
            print("no /odom received; is the robot connected to the agent?", file=sys.stderr)
            return 2
        for pwm in levels:
            for sign in (1, -1):
                node.hold(0, 0, args.rest_s, args.rate)
                node.samples.clear()
                t0 = time.monotonic()
                node.hold(sign * pwm, sign * pwm, args.hold_s, args.rate)
                samples = list(node.samples)
                t_end = time.monotonic()
                steady = [s for s in samples if s[0] >= t_end - args.steady_s]
                v_left = statistics.fmean(s[1] for s in steady) if steady else float("nan")
                v_right = statistics.fmean(s[2] for s in steady) if steady else float("nan")
                results.append((sign * pwm, v_left, v_right, node.voltage))
                raw += [(sign * pwm, f"{t - t0:.4f}", f"{vl:.5f}", f"{vr:.5f}") for t, vl, vr in samples]
                print(f"pwm {sign * pwm:+4d}: left {v_left:+.3f} m/s, right {v_right:+.3f} m/s, "
                      f"battery {node.voltage:.2f} V")
    finally:
        for _ in range(3):
            node.send_stop()
            node.spin_for(0.05)
        node.destroy_node()
        rclpy.shutdown()

    if results and max(max(abs(r[1]), abs(r[2])) for r in results) < args.moving:
        print("no wheel moved at any PWM; is the diagnostic firmware flashed?", file=sys.stderr)
        return 1

    print("\nwheel  dir  start pwm  k (m/s per pwm)  p0     fit points")
    for wheel, col in (("left", 1), ("right", 2)):
        for direction, sign in (("fwd", 1), ("rev", -1)):
            rows = sorted((abs(r[0]), sign * r[col]) for r in results if r[0] * sign > 0)
            start = next((p for p, v in rows if v > args.moving), None)
            k, p0 = fit_line([(p, v) for p, v in rows if v > args.moving])
            print(f"{wheel:<6} {direction:<4} {str(start):>9}  {k:>15.5f}  {p0:>5.1f}  "
                  f"{sum(1 for _, v in rows if v > args.moving)}")

    if args.csv:
        with open(args.csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["pwm", "v_left_mps", "v_right_mps", "battery_v"])
            writer.writerows((p, f"{vl:.5f}", f"{vr:.5f}", f"{b:.3f}") for p, vl, vr, b in results)
        print(f"results written to {args.csv}")
    if args.samples_csv:
        with open(args.samples_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["pwm", "t_s", "v_left_mps", "v_right_mps"])
            writer.writerows(raw)
    return 0


if __name__ == "__main__":
    sys.exit(main())
