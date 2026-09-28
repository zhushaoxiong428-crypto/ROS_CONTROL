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
