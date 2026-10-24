#!/usr/bin/env python3
"""Velocity step-response test through /cmd_vel and /odom.

For each target speed, streams linear.x at --rate Hz for --hold-s seconds,
then sends zero and waits for the robot to settle. Reports rise time,
overshoot, steady-state error and ripple from the /odom body velocity.
Lift the wheels or leave clear space for the longest run.

Usage:
  source /opt/ros/humble/setup.bash
  python3 tools/speed_step_test.py --speeds 0.05,0.1,0.2,-0.1 --csv /tmp/steps.csv
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


class StepTester(Node):
    def __init__(self):
        super().__init__("speed_step_test")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, qos_profile_sensor_data)
        self.samples = []  # (monotonic time, vx m/s)

    def on_odom(self, msg):
        self.samples.append((time.monotonic(), msg.twist.twist.linear.x))

    def send(self, vx):
        cmd = Twist()
        cmd.linear.x = vx
        self.pub.publish(cmd)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def hold(self, vx, seconds, rate):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.send(vx)
            self.spin_for(1.0 / rate)


def step_metrics(samples, t0, target, steady_window_s):
    """Rise time (10-90 %), overshoot and steady-state stats for one step."""
    resp = [(t - t0, v) for t, v in samples if t >= t0]
    sign = 1.0 if target >= 0 else -1.0
    mag = abs(target)

    def first_cross(level):
        return next((t for t, v in resp if sign * v >= level * mag), None)

    t10, t90 = first_cross(0.1), first_cross(0.9)
    peak = max((sign * v for _, v in resp), default=0.0)
    t_end = resp[-1][0] if resp else 0.0
    steady = [sign * v for t, v in resp if t >= t_end - steady_window_s]
    mean = statistics.fmean(steady) if steady else float("nan")
    std = statistics.pstdev(steady) if len(steady) > 1 else float("nan")
    return {
        "target": target,
        "rise_ms": (t90 - t10) * 1000 if t10 is not None and t90 is not None else float("nan"),
        "t90_ms": t90 * 1000 if t90 is not None else float("nan"),
        "overshoot_pct": max(0.0, (peak - mag) / mag * 100),
        "ss_error_pct": (mean - mag) / mag * 100,
        "ripple_pct": std / mag * 100,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--speeds", default="0.05,0.1,0.2,-0.1", help="comma-separated linear.x targets in m/s")
    parser.add_argument("--hold-s", type=float, default=4.0, help="time at each target")
    parser.add_argument("--rest-s", type=float, default=1.5, help="zero command between steps")
    parser.add_argument("--steady-s", type=float, default=2.0, help="window at the end of each hold for steady state")
    parser.add_argument("--rate", type=float, default=20.0, help="cmd_vel publish rate in Hz")
    parser.add_argument("--csv", help="write every odom sample to this file")
    args = parser.parse_args()

    rclpy.init()
    node = StepTester()
    results, rows = [], []
    try:
        node.spin_for(1.0)
        if not node.samples:
            print("no /odom received; is the robot connected to the agent?", file=sys.stderr)
            return 2
        for target in (float(s) for s in args.speeds.split(",")):
            node.hold(0.0, args.rest_s, args.rate)
            node.samples.clear()
            t0 = time.monotonic()
            node.hold(target, args.hold_s, args.rate)
            samples = list(node.samples)
            results.append(step_metrics(samples, t0, target, args.steady_s))
            rows += [(target, f"{t - t0:.4f}", f"{v:.5f}") for t, v in samples]
    finally:
        for _ in range(3):
            node.send(0.0)
            node.spin_for(0.05)
        node.destroy_node()
        rclpy.shutdown()

    print(f"{'target m/s':>10} {'t90 ms':>8} {'rise ms':>8} {'overshoot':>10} {'ss error':>9} {'ripple':>8}")
    for r in results:
        print(f"{r['target']:>10.3f} {r['t90_ms']:>8.0f} {r['rise_ms']:>8.0f} {r['overshoot_pct']:>9.1f}% "
              f"{r['ss_error_pct']:>+8.1f}% {r['ripple_pct']:>7.1f}%")
    if args.csv:
        with open(args.csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["target_mps", "t_since_step_s", "vx_mps"])
            writer.writerows(rows)
        print(f"samples written to {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
