#!/usr/bin/env python3
"""Compare the IMU yaw rate and heading with odometry, phase by phase.

Records /imu (angular_velocity.z and orientation yaw) and /odom (twist
angular.z and pose yaw) through four phases: standing still, driving
straight, turning left in place and turning right in place. Odometry is
the reference (calibrated to < 0.2 % on the ground). For each phase it
prints the mean yaw rate from both sources, their difference (gyro bias
under those conditions) and ratio (gyro scale), and the heading change
each one integrates.

Needs about 1 m of clear floor in front of the robot; do not touch it
during the still phase.

Usage:
  source /opt/ros/humble/setup.bash
  python3 tools/imu_heading_test.py --csv /tmp/imu_heading.csv
"""

import argparse
import csv
import math
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def quat_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def unwrapped_change(yaws):
    total = 0.0
    for a, b in zip(yaws, yaws[1:]):
        total += math.atan2(math.sin(b - a), math.cos(b - a))
    return total


class Recorder(Node):
    def __init__(self):
        super().__init__("imu_heading_test")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, "imu", self.on_imu, qos_profile_sensor_data)
        self.phase = "settle"
        self.odom = []  # (time, phase, wz, yaw)
        self.imu = []   # (time, phase, gz, yaw)

    def on_odom(self, msg):
        self.odom.append((time.monotonic(), self.phase, msg.twist.twist.angular.z,
                          quat_yaw(msg.pose.pose.orientation)))

    def on_imu(self, msg):
        self.imu.append((time.monotonic(), self.phase, msg.angular_velocity.z, quat_yaw(msg.orientation)))

    def send(self, vx=0.0, wz=0.0):
        cmd = Twist()
        cmd.linear.x = vx
        cmd.angular.z = wz
        self.pub.publish(cmd)

    def run_phase(self, name, vx, wz, seconds, rate=20.0):
        self.phase = name
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if vx or wz:
                self.send(vx, wz)
            rclpy.spin_once(self, timeout_sec=1.0 / rate)
        self.phase = "settle"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--still-s", type=float, default=20.0)
    parser.add_argument("--move-s", type=float, default=8.0, help="duration of each moving phase")
    parser.add_argument("--speed", type=float, default=0.1, help="straight phase, m/s")
    parser.add_argument("--wz", type=float, default=0.5, help="turning phases, rad/s")
    parser.add_argument("--skip-s", type=float, default=1.0, help="ignore the start of each moving phase")
    parser.add_argument("--csv", help="write every sample to this file")
    args = parser.parse_args()

    phases = [("still", 0.0, 0.0, args.still_s),
              ("straight", args.speed, 0.0, args.move_s),
              ("left", 0.0, abs(args.wz), args.move_s),
              ("right", 0.0, -abs(args.wz), args.move_s)]

    rclpy.init()
    node = Recorder()
    try:
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not node.odom or not node.imu:
            print("no /odom or /imu received; is the robot connected to the agent?", file=sys.stderr)
            return 2
        for name, vx, wz, seconds in phases:
            print(f"phase {name} ({seconds:.0f} s)...", flush=True)
            node.run_phase(name, vx, wz, seconds)
            for _ in range(3):
                node.send()
                rclpy.spin_once(node, timeout_sec=0.05)
            settle_end = time.monotonic() + 1.5
            while time.monotonic() < settle_end:
                rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        for _ in range(3):
            node.send()
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()

    print(f"\n{'phase':<9} {'odom wz':>9} {'gyro gz':>9} {'gyro-odom':>10} {'gyro/odom':>10} "
          f"{'gyro std':>9} {'odom dyaw':>10} {'imu dyaw':>9}")
    print(f"{'':<9} {'deg/s':>9} {'deg/s':>9} {'deg/s':>10} {'':>10} {'deg/s':>9} {'deg':>10} {'deg':>9}")
    for name, vx, wz, seconds in phases:
        odom = [s for s in node.odom if s[1] == name]
        imu = [s for s in node.imu if s[1] == name]
        if not odom or not imu:
            continue
        skip = 0.0 if name == "still" else args.skip_s
        t0 = odom[0][0]
        o_rate = [math.degrees(s[2]) for s in odom if s[0] >= t0 + skip]
        g_rate = [math.degrees(s[2]) for s in imu if s[0] >= t0 + skip]
        o_mean, g_mean = statistics.fmean(o_rate), statistics.fmean(g_rate)
        ratio = g_mean / o_mean if abs(o_mean) > 1.0 else float("nan")
        print(f"{name:<9} {o_mean:>+9.3f} {g_mean:>+9.3f} {g_mean - o_mean:>+10.3f} {ratio:>10.4f} "
              f"{statistics.pstdev(g_rate):>9.3f} "
              f"{math.degrees(unwrapped_change([s[3] for s in odom])):>+10.2f} "
              f"{math.degrees(unwrapped_change([s[3] for s in imu])):>+9.2f}")

    if args.csv:
        with open(args.csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["source", "t_s", "phase", "yaw_rate_rad_s", "yaw_rad"])
            t0 = min(node.odom[0][0], node.imu[0][0])
            writer.writerows(("odom", f"{t - t0:.4f}", p, f"{r:.6f}", f"{y:.6f}") for t, p, r, y in node.odom)
            writer.writerows(("imu", f"{t - t0:.4f}", p, f"{r:.6f}", f"{y:.6f}") for t, p, r, y in node.imu)
        print(f"samples written to {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
