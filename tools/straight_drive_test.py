#!/usr/bin/env python3
"""落地直行偏航测试：发送直行 /cmd_vel，同时记录 /odom 与 /imu，判断偏航来源。

/odom 的航向由编码器推算，/imu 的航向由陀螺仪积分得到：
- 两者偏航接近：左右轮转速确实不同（控制问题），编码器看得到；
- IMU 偏航明显而编码器几乎为 0：两轮转速一样但车仍然转了，
  来自轮径差、打滑等机械因素，只有 IMU 航向闭环能纠正。

用法（车放在地上，前方留出足够空间，随时准备急停）：
    python3 tools/straight_drive_test.py --speed 0.1 --distance 1.0 --confirm-ground
结果 CSV 写入 data/，文件名带时间戳。
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

STRAIGHT_TOLERANCE_DEG = 2.0
TRACK_WIDTH_M = 0.1317  # 与固件 kTrackWidth 一致
# 打滑检测：每前进 SLIP_WINDOW_M，编码器航向与 IMU 航向之差的变化超过该角度即视为打滑/卡顿。
# 固定的比例差按行程线性累积（本车未标定时约 2.3 deg / 0.1 m），打滑是局部突变。
SLIP_WINDOW_M = 0.1
SLIP_THRESHOLD_DEG = 5.0


@dataclass
class Sample:
    t: float
    odom_x: float
    odom_y: float
    odom_yaw: float
    imu_yaw: float


@dataclass
class DriftReport:
    duration_s: float
    distance_m: float
    lateral_offset_m: float
    encoder_yaw_deg: float
    imu_yaw_deg: float
    # 让编码器航向与 IMU 航向一致所需的右/左行程比（乘以当前固件的比值即为新的标定值）
    implied_travel_ratio: float | None
    # 检测到的打滑/卡顿事件：(起点前进距离 m, 该段编码器-IMU 航向差变化 deg)
    slip_events: list[tuple[float, float]]
    diagnosis: str


def wrap(angle: float) -> float:
    return math.remainder(angle, 2.0 * math.pi)


def yaw_from_quaternion(w: float, x: float, y: float, z: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _forward_and_divergence(samples: list[Sample]) -> tuple[list[float], list[float]]:
    """每个采样点的前进距离（起点航向坐标系）和累计的"编码器航向 - IMU 航向"（deg，已展开）。"""
    first = samples[0]
    cos0, sin0 = math.cos(first.odom_yaw), math.sin(first.odom_yaw)
    forward, divergence = [], []
    enc = imu = 0.0
    for i, s in enumerate(samples):
        if i > 0:
            enc += wrap(s.odom_yaw - samples[i - 1].odom_yaw)
            imu += wrap(s.imu_yaw - samples[i - 1].imu_yaw)
        forward.append(cos0 * (s.odom_x - first.odom_x) + sin0 * (s.odom_y - first.odom_y))
        divergence.append(math.degrees(enc - imu))
    return forward, divergence


def find_slip_events(samples: list[Sample]) -> list[tuple[float, float]]:
    forward, divergence = _forward_and_divergence(samples)
    # 1) 标出所有"前进 SLIP_WINDOW_M 内航向差变化超过阈值"的窗口 [i, j]
    windows: list[tuple[int, int]] = []
    j = 0
    for i in range(len(samples)):
        while j < len(samples) and forward[j] - forward[i] < SLIP_WINDOW_M:
            j += 1
        if j >= len(samples):
            break
        if abs(divergence[j] - divergence[i]) > SLIP_THRESHOLD_DEG:
            windows.append((i, j))
    # 2) 合并重叠或相邻的窗口，每段报告起点距离和整段的航向差变化
    events: list[tuple[float, float]] = []
    start = end = None
    for i, j in windows:
        if start is not None and i <= end:
            end = max(end, j)
            continue
        if start is not None:
            events.append((forward[start], divergence[end] - divergence[start]))
        start, end = i, j
    if start is not None:
        events.append((forward[start], divergence[end] - divergence[start]))
    return events


def analyze(samples: list[Sample]) -> DriftReport:
    """samples 为直行段内（含起步）按时间排序的采样。偏航以逆时针（向左）为正。"""
    if len(samples) < 2:
        raise ValueError("need at least two samples inside the drive window")
    first, last = samples[0], samples[-1]

    dx = last.odom_x - first.odom_x
    dy = last.odom_y - first.odom_y
    # 转到起点航向坐标系：前进方向为 x，左侧为 y。
    cos0, sin0 = math.cos(first.odom_yaw), math.sin(first.odom_yaw)
    forward = cos0 * dx + sin0 * dy
    lateral = -sin0 * dx + cos0 * dy

    encoder_yaw = math.degrees(wrap(last.odom_yaw - first.odom_yaw))
    imu_yaw = math.degrees(wrap(last.imu_yaw - first.imu_yaw))

    # 编码器推算的左右行程差 = (编码器航向 - 真实航向) × 轮距；
    # 真实左右行程应满足 IMU 航向，由此求出右/左每脉冲行程之比。
    # 比例只用第一次打滑之前的数据推算，避免局部打滑污染标定结果。
    slip_events = find_slip_events(samples)
    ratio_samples = samples
    if slip_events:
        fwd_list, _ = _forward_and_divergence(samples)
        cut = next(i for i, f in enumerate(fwd_list) if f >= slip_events[0][0])
        ratio_samples = samples[:cut + 1]
    implied_ratio = None
    r_fwd, r_div = _forward_and_divergence(ratio_samples)
    if len(ratio_samples) >= 2 and r_fwd[-1] > 0.2:
        excess = math.radians(-r_div[-1]) * TRACK_WIDTH_M  # 左 - 右
        left, right = r_fwd[-1] + excess / 2.0, r_fwd[-1] - excess / 2.0
        implied_ratio = left / right

    if abs(imu_yaw) < STRAIGHT_TOLERANCE_DEG:
        diagnosis = "straight: IMU heading change within tolerance"
        mismatch = encoder_yaw - imu_yaw
        if abs(mismatch) >= STRAIGHT_TOLERANCE_DEG and not slip_events:
            # 车实际走直，但编码器推算出转弯：两轮每个脉冲对应的行程不同。
            # 左右轮行程差 = 航向差 × 轮距，据此给出右/左比例。
            side = "right" if mismatch < 0 else "left"
            diagnosis += (f"; but encoders report a {abs(mismatch):.1f} deg {side} turn: "
                          "per-wheel scale mismatch (gear ratio / wheel diameter), "
                          "odometry is wrong")
    else:
        side = "left" if imu_yaw > 0 else "right"
        ratio = encoder_yaw / imu_yaw
        if ratio >= 0.7:
            diagnosis = (f"drifts {side}; encoders see most of it "
                         f"({ratio:.0%}): wheel speeds really differ (control)")
        elif abs(encoder_yaw) < STRAIGHT_TOLERANCE_DEG:
            diagnosis = (f"drifts {side}; encoders see almost none of it: "
                         "wheel diameter mismatch or slip (mechanical)")
        else:
            diagnosis = (f"drifts {side}; encoders see {ratio:.0%} of it: "
                         "mixed control and mechanical causes")

    if slip_events:
        diagnosis = (f"{len(slip_events)} slip/stall event(s) (see below); "
                     "whole-run encoder yaw and odometry are unreliable. " + diagnosis)

    return DriftReport(
        duration_s=last.t - first.t,
        distance_m=forward,  # /odom 位置单位为 m
        lateral_offset_m=lateral,
        encoder_yaw_deg=encoder_yaw,
        imu_yaw_deg=imu_yaw,
        implied_travel_ratio=implied_ratio,
        slip_events=slip_events,
        diagnosis=diagnosis,
    )


def format_report(report: DriftReport) -> str:
    return "\n".join((
        f"duration        : {report.duration_s:.2f} s",
        f"distance (odom) : {report.distance_m:.3f} m",
        f"lateral (odom)  : {report.lateral_offset_m * 100:+.1f} cm  (+ = left)",
        f"encoder yaw     : {report.encoder_yaw_deg:+.2f} deg  (+ = left)",
        f"IMU yaw         : {report.imu_yaw_deg:+.2f} deg  (+ = left)",
        "right/left ratio: " + (f"{report.implied_travel_ratio:.4f}  (x current "
                                "LEAP_RIGHT_LEFT_TRAVEL_RATIO = new calibration"
                                + (", from data before the first slip)" if report.slip_events else ")")
                                if report.implied_travel_ratio is not None else "n/a"),
        "slip events     : " + ("none" if not report.slip_events else "; ".join(
            f"at {fwd:.2f} m encoders diverged {change:+.1f} deg from IMU"
            for fwd, change in report.slip_events)),
        f"diagnosis       : {report.diagnosis}",
    ))


def run(args: argparse.Namespace) -> int:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu

    class StraightDriveNode(Node):
        def __init__(self) -> None:
            super().__init__("straight_drive_test")
            self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
            self.odom = None
            self.imu_yaw = None
            # 遥测可能是 Best Effort，sensor_data QoS 与两种发布端都兼容。
            self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
            self.create_subscription(Imu, "/imu", self.on_imu, qos_profile_sensor_data)

        def on_odom(self, msg: Odometry) -> None:
            q = msg.pose.pose.orientation
            self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                         yaw_from_quaternion(q.w, q.x, q.y, q.z))

        def on_imu(self, msg: Imu) -> None:
            q = msg.orientation
            self.imu_yaw = yaw_from_quaternion(q.w, q.x, q.y, q.z)

        def send(self, vx: float) -> None:
            twist = Twist()
            twist.linear.x = vx
            self.cmd.publish(twist)

    rclpy.init()
    node = StraightDriveNode()
    period = 1.0 / args.rate
    samples: list[Sample] = []

    def spin_for(seconds: float, vx: float, record: bool, start: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            node.send(vx)
            rclpy.spin_once(node, timeout_sec=period)
            if record and node.odom is not None and node.imu_yaw is not None:
                samples.append(Sample(time.monotonic() - start, *node.odom, node.imu_yaw))

    try:
        deadline = time.monotonic() + 5.0
        while (node.odom is None or node.imu_yaw is None) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.odom is None or node.imu_yaw is None:
            print("no /odom or /imu received; is the micro-ROS agent connected?")
            return 2

        start = time.monotonic()
        # 连续零速命令：重新武装电池互锁 / 急停释放后的零速要求，并让车静止。
        spin_for(0.5, 0.0, False, start)
        drive_s = args.distance / args.speed
        print(f"driving {args.distance:.2f} m at {args.speed:.2f} m/s ({drive_s:.1f} s)...")
        spin_for(drive_s, args.speed, True, start)
    finally:
        # 无论成功与否都连续发零速，停车交给 MCU 的主动制动和 500 ms 看门狗兜底。
        for _ in range(int(args.rate)):
            node.send(0.0)
            time.sleep(period)
        node.destroy_node()
        rclpy.shutdown()

    out = Path(args.output) if args.output else Path("data") / (
        f"straight_drive_{args.speed:.2f}mps_{datetime.now():%Y%m%d_%H%M%S}.csv")
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(("time_s", "odom_x", "odom_y", "odom_yaw_rad", "imu_yaw_rad"))
        for s in samples:
            writer.writerow((f"{s.t:.4f}", s.odom_x, s.odom_y, s.odom_yaw, s.imu_yaw))

    print(format_report(analyze(samples)))
    print(f"samples saved to {out}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--speed", type=float, default=0.1, help="forward speed, m/s")
    parser.add_argument("--distance", type=float, default=1.0, help="drive distance, m")
    parser.add_argument("--rate", type=float, default=20.0, help="/cmd_vel rate, Hz")
    parser.add_argument("--output", help="CSV path (default: data/straight_drive_*.csv)")
    parser.add_argument("--confirm-ground", action="store_true",
                        help="required: robot is on the floor with clear space ahead")
    args = parser.parse_args()
    if not args.confirm_ground:
        parser.error("this sends real motion commands; pass --confirm-ground when safe")
    if not 0.0 < args.speed <= 0.3 or not 0.0 < args.distance <= 3.0:
        parser.error("speed must be in (0, 0.3] m/s and distance in (0, 3] m")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
