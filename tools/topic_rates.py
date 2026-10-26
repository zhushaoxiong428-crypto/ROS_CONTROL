#!/usr/bin/env python3
"""Measure the arrival rate and gaps of the robot's ROS topics on the PC.

Subscribes to /odom, /imu, /scan, /battery_state and /ultrasonic for
--duration seconds and reports, per topic, the message count, mean rate and
inter-arrival statistics (median, p99, max). Times are PC receive times, so
they include Wi-Fi and agent delays: this is the end-to-end view a ROS node
on the PC sees. Run it alone for an idle baseline, or alongside a motion
test to see whether driving changes it.

Usage:
  source /opt/ros/humble/setup.bash
  python3 tools/topic_rates.py --duration 60 --csv /tmp/rates.csv
"""

import argparse
import csv
import statistics
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu, LaserScan, Range

TOPICS = {
    "odom": Odometry,
    "imu": Imu,
    "scan": LaserScan,
    "battery_state": BatteryState,
    "ultrasonic": Range,
}


class RateRecorder(Node):
    def __init__(self, topics):
        super().__init__("topic_rates")
        self.arrivals = {name: [] for name in topics}
        for name in topics:
            self.create_subscription(TOPICS[name], name, self._callback(name), qos_profile_sensor_data)

    def _callback(self, name):
        def record(_msg):
            self.arrivals[name].append(time.monotonic())
        return record


def percentile(sorted_values, fraction):
    if not sorted_values:
        return float("nan")
    index = min(len(sorted_values) - 1, int(round(fraction * (len(sorted_values) - 1))))
    return sorted_values[index]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--duration", type=float, default=60.0, help="seconds")
    parser.add_argument("--topics", default=",".join(TOPICS), help="comma-separated subset of " + ",".join(TOPICS))
    parser.add_argument("--csv", help="write every arrival time to this file")
    args = parser.parse_args()

    topics = [t for t in args.topics.split(",") if t]
    unknown = [t for t in topics if t not in TOPICS]
    if unknown:
        print(f"unknown topics: {unknown}", file=sys.stderr)
        return 2

    rclpy.init()
    node = RateRecorder(topics)
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.01)
    finally:
        elapsed = time.monotonic() - start
        node.destroy_node()
        rclpy.shutdown()

    print(f"{'topic':<14} {'count':>6} {'rate Hz':>8} {'median ms':>10} {'p99 ms':>8} {'max gap ms':>11}")
    for name in topics:
        times = node.arrivals[name]
        gaps = sorted((b - a) * 1000.0 for a, b in zip(times, times[1:]))
        rate = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.0
        print(f"{name:<14} {len(times):>6} {rate:>8.2f} {percentile(gaps, 0.5):>10.1f} "
              f"{percentile(gaps, 0.99):>8.1f} {max(gaps) if gaps else float('nan'):>11.1f}")
    print(f"window {elapsed:.1f} s")

    if args.csv:
        with open(args.csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["topic", "t_s"])
            for name in topics:
                writer.writerows((name, f"{t - start:.4f}") for t in node.arrivals[name])
        print(f"arrivals written to {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
