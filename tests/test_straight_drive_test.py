"""Unit tests for the drift analysis in tools/straight_drive_test.py (no ROS needed)."""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from straight_drive_test import Sample, analyze, wrap, yaw_from_quaternion  # noqa: E402


def arc(heading_rate_deg_s: float, encoder_fraction: float, speed=0.1, duration=10.0, dt=0.05,
        start_yaw=0.0):
    """Robot turning at heading_rate; encoders observe encoder_fraction of that turn."""
    samples = []
    x = y = 0.0  # odometry integrates with the encoder heading, like the firmware
    true_yaw = odom_yaw = start_yaw
    rate = math.radians(heading_rate_deg_s)
    for i in range(int(duration / dt) + 1):
        samples.append(Sample(i * dt, x, y, wrap(odom_yaw), wrap(true_yaw)))
        x += speed * math.cos(odom_yaw) * dt
        y += speed * math.sin(odom_yaw) * dt
        true_yaw += rate * dt
        odom_yaw += rate * encoder_fraction * dt
    return samples


class AnalyzeTest(unittest.TestCase):
    def test_straight(self):
        report = analyze(arc(0.0, 1.0))
        self.assertAlmostEqual(report.distance_m, 1.0, places=2)
        self.assertAlmostEqual(report.lateral_offset_m, 0.0, places=6)
        self.assertTrue(report.diagnosis.startswith("straight"))

    def test_left_drift_seen_by_encoders_is_control(self):
        report = analyze(arc(1.0, 1.0))
        self.assertGreater(report.imu_yaw_deg, 9.0)
        self.assertGreater(report.lateral_offset_m, 0.05)
        self.assertIn("drifts left", report.diagnosis)
        self.assertIn("control", report.diagnosis)

    def test_drift_invisible_to_encoders_is_mechanical(self):
        report = analyze(arc(1.0, 0.0))
        self.assertAlmostEqual(report.encoder_yaw_deg, 0.0, places=6)
        self.assertIn("mechanical", report.diagnosis)
        self.assertAlmostEqual(report.lateral_offset_m, 0.0, places=6)  # odom sees straight

    def test_straight_but_encoders_disagree_is_scale_mismatch(self):
        # 航向保持把车拉直了，编码器却推算出右转（本次实测的情形）。
        samples = arc(0.0, 1.0)
        for i, sample in enumerate(samples):
            sample.odom_yaw = -math.radians(20.0) * i / (len(samples) - 1)
        report = analyze(samples)
        self.assertTrue(report.diagnosis.startswith("straight"))
        self.assertIn("scale mismatch", report.diagnosis)
        self.assertIn("right turn", report.diagnosis)
        # 编码器多报 20 deg 右转：左轮编码器行程更长，右轮每脉冲实际走得更远。
        self.assertGreater(report.implied_travel_ratio, 1.03)

    def test_steady_scale_mismatch_is_not_slip(self):
        # 未标定时的固定比例差（约 2.3 deg / 0.1 m）按行程线性累积，不应被当成打滑。
        samples = arc(0.0, 1.0)
        for i, sample in enumerate(samples):
            sample.odom_yaw = -math.radians(23.0) * i / (len(samples) - 1)
        self.assertEqual(analyze(samples).slip_events, [])

    def test_local_slip_is_detected_and_excluded_from_ratio(self):
        # 前 0.6 m 编码器与 IMU 一致，随后 0.2 m 内编码器凭空多出 20 deg 右转（实测 2026-09-29 的情形）。
        samples = arc(0.0, 1.0)
        n = len(samples)
        for i, sample in enumerate(samples):
            progress = i / (n - 1)
            sample.odom_yaw = -math.radians(20.0) * min(max((progress - 0.6) / 0.2, 0.0), 1.0)
        report = analyze(samples)
        self.assertEqual(len(report.slip_events), 1)
        start, change = report.slip_events[0]
        self.assertAlmostEqual(start, 0.55, delta=0.08)
        self.assertAlmostEqual(change, -20.0, delta=1.5)
        self.assertAlmostEqual(report.implied_travel_ratio, 1.0, delta=0.01)
        self.assertIn("slip", report.diagnosis)
        self.assertNotIn("scale mismatch", report.diagnosis)

    def test_right_drift_and_mixed(self):
        report = analyze(arc(-1.0, 0.4))
        self.assertIn("drifts right", report.diagnosis)
        self.assertIn("mixed", report.diagnosis)

    def test_lateral_measured_in_start_frame(self):
        report = analyze(arc(1.0, 1.0, start_yaw=math.radians(170.0)))
        self.assertGreater(report.lateral_offset_m, 0.05)
        self.assertGreater(report.distance_m, 0.9)

    def test_quaternion_yaw(self):
        half = math.radians(30.0) / 2.0
        self.assertAlmostEqual(yaw_from_quaternion(math.cos(half), 0.0, 0.0, math.sin(half)),
                               math.radians(30.0))

    def test_needs_samples(self):
        with self.assertRaises(ValueError):
            analyze([])


if __name__ == "__main__":
    unittest.main()
