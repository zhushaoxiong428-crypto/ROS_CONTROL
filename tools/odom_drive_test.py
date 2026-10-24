#!/usr/bin/env python3
"""Ground test of odometry: drive a fixed distance or turn, then compare.

The robot drives until /odom says the target is reached, stops, and reports
what odometry and the IMU heading (/imu orientation) measured. Measure the
real distance, lateral offset and heading by hand and compare.

  straight  drive --distance metres at --speed
  spin      turn --turns full turns in place at --wz

Usage:
  source /opt/ros/humble/setup.bash
  python3 tools/odom_drive_test.py straight --distance 1.0 --speed 0.1
  python3 tools/odom_drive_test.py spin --turns 1 --wz 0.8
"""

import argparse
import math
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


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class UnwrappedYaw:
    """Accumulates a heading across the +/-pi wrap."""

    def __init__(self):
        self.last = None
        self.total = 0.0

    def update(self, yaw):
        if self.last is not None:
            self.total += wrap(yaw - self.last)
        self.last = yaw


class DriveTester(Node):
    def __init__(self):
        super().__init__("odom_drive_test")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, "imu", self.on_imu, qos_profile_sensor_data)
        self.x = self.y = None
        self.odom_yaw = UnwrappedYaw()
        self.imu_yaw = UnwrappedYaw()

    def on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.odom_yaw.update(quat_yaw(msg.pose.pose.orientation))

    def on_imu(self, msg):
        self.imu_yaw.update(quat_yaw(msg.orientation))

    def send(self, vx=0.0, wz=0.0):
        cmd = Twist()
        cmd.linear.x = vx
        cmd.angular.z = wz
        self.pub.publish(cmd)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def snapshot(self):
        return self.x, self.y, self.odom_yaw.total, self.imu_yaw.total


def drive_until(node, vx, wz, done, max_s, rate=20.0):
    end = time.monotonic() + max_s
    while time.monotonic() < end:
        if done():
            return True
        node.send(vx, wz)
        node.spin_for(1.0 / rate)
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="mode", required=True)
    straight = sub.add_parser("straight")
    straight.add_argument("--distance", type=float, default=1.0, help="metres")
    straight.add_argument("--speed", type=float, default=0.1, help="m/s")
    spin = sub.add_parser("spin")
    spin.add_argument("--turns", type=float, default=1.0)
    spin.add_argument("--wz", type=float, default=0.8, help="rad/s, positive turns left")
    parser.add_argument("--settle-s", type=float, default=1.5, help="wait after stopping before reading")
    args = parser.parse_args()

    rclpy.init()
    node = DriveTester()
    try:
        node.spin_for(1.0)
        if node.x is None or node.imu_yaw.last is None:
            print("no /odom or /imu received; is the robot connected to the agent?", file=sys.stderr)
            return 2
        x0, y0, odom0, imu0 = node.snapshot()
        heading0 = node.odom_yaw.last

        if args.mode == "straight":
            def travelled():
                return math.hypot(node.x - x0, node.y - y0)
            reached = drive_until(node, args.speed, 0.0, lambda: travelled() >= args.distance,
                                  max_s=args.distance / abs(args.speed) * 2 + 3)
        else:
            target = 2 * math.pi * args.turns
            reached = drive_until(node, 0.0, args.wz,
                                  lambda: abs(node.odom_yaw.total - odom0) >= target,
                                  max_s=target / abs(args.wz) * 2 + 3)
        for _ in range(3):
            node.send()
            node.spin_for(0.05)
        node.spin_for(args.settle_s)

        x1, y1, odom1, imu1 = node.snapshot()
        # Express the displacement in the start heading: forward and to the left.
        dx, dy = x1 - x0, y1 - y0
        forward = dx * math.cos(heading0) + dy * math.sin(heading0)
        left = -dx * math.sin(heading0) + dy * math.cos(heading0)
        odom_turn = math.degrees(odom1 - odom0)
        imu_turn = math.degrees(imu1 - imu0)

        if not reached:
            print("WARNING: target not reached before the time limit")
        print(f"mode: {args.mode}")
        print(f"odom  forward {forward:+.3f} m, left {left:+.3f} m, heading change {odom_turn:+.1f} deg")
        print(f"imu   heading change {imu_turn:+.1f} deg (odom - imu = {odom_turn - imu_turn:+.1f} deg)")
        if args.mode == "straight":
            print("Now measure: actual forward distance and lateral offset (left positive).")
        else:
            print("Now measure: actual heading change against a mark on the floor.")
    finally:
        for _ in range(3):
            node.send()
            node.spin_for(0.05)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
