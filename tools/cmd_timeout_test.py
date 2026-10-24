#!/usr/bin/env python3
"""Hardware test for the firmware's cmd_vel timeout (kCommandTimeoutMs).

Drives the robot through /cmd_vel, then goes silent without sending a zero
command, and measures from /odom how long the robot keeps moving after the
last command. Lift the wheels or leave clear space in front of the robot.

Scenarios:
  stream  publish at --rate Hz for --drive-s seconds, then stop publishing
  once    publish a single command, like `ros2 topic pub --once`

Usage:
  source /opt/ros/humble/setup.bash
  python3 tools/cmd_timeout_test.py --speed 0.1 --csv /tmp/cmd_timeout.csv
"""

import argparse
import csv
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class TimeoutTester(Node):
    def __init__(self):
        super().__init__("cmd_timeout_test")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, qos_profile_sensor_data)
        self.samples = []  # (monotonic time, vx m/s)

    def on_odom(self, msg):
        self.samples.append((time.monotonic(), msg.twist.twist.linear.x))

    def send(self, vx):
        cmd = Twist()
        cmd.linear.x = vx
        self.pub.publish(cmd)
        return time.monotonic()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.005)


def stop_latency(samples, t_last_cmd, threshold, settle_s):
    """Time from the last command until |vx| drops below threshold for good.

    Returns (latency_s, moved): moved is False if the robot never exceeded the
    threshold after the last command (nothing to measure).
    """
    after = [(t, v) for t, v in samples if t >= t_last_cmd]
    if not after or after[-1][0] - t_last_cmd < settle_s:
        return None, False
    moved = any(abs(v) >= threshold for _, v in after)
    t_stop = t_last_cmd
    for t, v in after:
        if abs(v) >= threshold:
            t_stop = t
    return t_stop - t_last_cmd, moved


def run_scenario(node, name, speed, rate, drive_s, observe_s):
    node.samples.clear()
    node.spin_for(0.5)  # baseline, robot at rest
    if name == "stream":
        period = 1.0 / rate
        t_end = time.monotonic() + drive_s
        t_last = node.send(speed)
        while time.monotonic() < t_end:
            node.spin_for(period)
            t_last = node.send(speed)
    else:
        t_last = node.send(speed)
    node.spin_for(observe_s)
    return t_last, list(node.samples)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--speed", type=float, default=0.1, help="linear.x in m/s")
    parser.add_argument("--rate", type=float, default=10.0, help="stream publish rate in Hz")
    parser.add_argument("--drive-s", type=float, default=2.0, help="stream duration in s")
    parser.add_argument("--observe-s", type=float, default=3.0, help="observation after the last command")
    parser.add_argument("--timeout-ms", type=float, default=500.0, help="firmware kCommandTimeoutMs")
    parser.add_argument("--margin-ms", type=float, default=400.0,
                        help="allowed braking, filter lag and network delay on top of the timeout")
    parser.add_argument("--threshold", type=float, default=0.01, help="|vx| below this counts as stopped (m/s)")
    parser.add_argument("--scenarios", default="stream,once")
    parser.add_argument("--csv", help="write every odom sample to this file")
    args = parser.parse_args()

    rclpy.init()
    node = TimeoutTester()
    rows = []
    failed = False
    try:
        node.spin_for(1.0)  # let discovery match publisher and subscriber
        if not node.samples:
            print("no /odom received; is the robot connected to the agent?", file=sys.stderr)
            return 2
        limit_s = (args.timeout_ms + args.margin_ms) / 1000.0
        for name in args.scenarios.split(","):
            t_last, samples = run_scenario(node, name, args.speed, args.rate, args.drive_s, args.observe_s)
            peak = max((abs(v) for t, v in samples if t >= t_last - args.drive_s), default=0.0)
            latency, moved = stop_latency(samples, t_last, args.threshold, settle_s=args.observe_s * 0.5)
            ok = moved and latency is not None and latency <= limit_s
            failed |= not ok
            if latency is None:
                verdict = "no data"
            elif not moved:
                verdict = "robot did not move"
            else:
                verdict = f"stopped {latency * 1000:.0f} ms after last command"
            print(f"[{'PASS' if ok else 'FAIL'}] {name:6s} peak |vx|={peak:.3f} m/s, {verdict} "
                  f"(limit {limit_s * 1000:.0f} ms)")
            rows += [(name, f"{t - t_last:.4f}", f"{v:.5f}") for t, v in samples]
            node.spin_for(1.0)
    finally:
        for _ in range(3):
            node.send(0.0)
            node.spin_for(0.05)
        node.destroy_node()
        rclpy.shutdown()

    if args.csv:
        with open(args.csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["scenario", "t_since_last_cmd_s", "vx_mps"])
            writer.writerows(rows)
        print(f"samples written to {args.csv}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
