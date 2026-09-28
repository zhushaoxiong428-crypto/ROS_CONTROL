#!/usr/bin/env python3
"""Offline checks for the trial recorder; never starts ROS or motors."""

import math
import io
import sys
import unittest
from contextlib import redirect_stderr
from pathlib import Path
from unittest.mock import patch


sys.path.insert(0, str(Path(__file__).parents[1] / "tools"))
from record_motor_debug import (  # noqa: E402
    DEFAULT_MOTOR_DEBUG_QOS_DEPTH,
    FIELDS,
    LEGACY_FIELDS,
    motor_debug_values,
)
from run_motor_startup_trial import (  # noqa: E402
    BATTERY_FRESH_NS,
    BATTERY_STABLE_NS,
    INTERLOCK_REARM_ZERO_SECONDS,
    TRIAL_BATTERY_MAX_V,
    TRIAL_BATTERY_MIN_V,
    TrialSettings,
    battery_message_error,
    battery_readiness_error,
    battery_summary,
    battery_voltage_error,
    motion_command_safety_error,
    next_battery_valid_since_ns,
    parse_args,
    startup_fault_rearm_error,
    stop_verification_error,
)


class MotorStartupRecordingTest(unittest.TestCase):
    def test_exact_telemetry_schema(self) -> None:
        self.assertEqual(len(LEGACY_FIELDS), 13)
        self.assertEqual(len(FIELDS), 19)
        self.assertEqual(motor_debug_values(range(13)), tuple(float(x) for x in range(13)))
        timed = [0.0] * 13 + [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        self.assertEqual(motor_debug_values(timed, require_timing=True), tuple(timed))
        with self.assertRaisesRegex(ValueError, "needs 19 values"):
            motor_debug_values([0.0] * 13, require_timing=True)

    def test_short_or_nonfinite_telemetry_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "needs 13 or 19 values"):
            motor_debug_values(range(8))
        for length in (14, 18, 20):
            with self.subTest(length=length), self.assertRaisesRegex(ValueError, "needs 13 or 19 values"):
                motor_debug_values([0.0] * length)
        with self.assertRaisesRegex(ValueError, "non-finite"):
            motor_debug_values([0.0] * 12 + [math.nan])
        with self.assertRaisesRegex(ValueError, "uint16"):
            motor_debug_values([0.0] * 13 + [1.5] + [0.0] * 5)

    def test_default_trial_is_bounded(self) -> None:
        TrialSettings().validate()
        self.assertEqual(TrialSettings().speed_mps, 0.1)
        self.assertEqual(TrialSettings().rate_hz, 10.0)
        self.assertGreaterEqual(TrialSettings().zero_seconds, 1.0)
        self.assertGreaterEqual(INTERLOCK_REARM_ZERO_SECONDS, 0.3)

    def test_default_debug_queue_keeps_active_trial_view_fresh(self) -> None:
        self.assertEqual(DEFAULT_MOTOR_DEBUG_QOS_DEPTH, 10)

    def test_unsafe_trial_settings_are_rejected(self) -> None:
        for settings in (
            TrialSettings(speed_mps=0.2),
            TrialSettings(speed_mps=0.0),
            TrialSettings(drive_seconds=11.0),
            TrialSettings(zero_seconds=0.5),
            TrialSettings(rate_hz=1.0),
            TrialSettings(wait_seconds=0.0),
        ):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                settings.validate()

    def test_explicit_suspended_confirmation_is_required(self) -> None:
        with patch.object(sys, "argv", ["run_motor_startup_trial.py", "data/trial", "--firmware-id", "A"]):
            with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as result:
                parse_args()
        self.assertEqual(result.exception.code, 2)

        with patch.object(sys, "argv", [
            "run_motor_startup_trial.py", "data/trial", "--firmware-id", "A", "--wheels-suspended",
        ]):
            self.assertEqual(parse_args().firmware_id, "A")

    def test_stop_needs_fresh_post_zero_motor_confirmation(self) -> None:
        stopped = (0.0,) * 13
        self.assertIsNone(stop_verification_error(
            stopped, 2_000_000_000, 900_000_000, 1_000_000_000,
            1_100_000_000, 2_100_000_000, 1, 1,
        ))
        self.assertIsNotNone(stop_verification_error(
            stopped, 2_000_000_000, 900_000_000, 1_000_000_000,
            None, 2_100_000_000, 1, 1,
        ))
        self.assertIsNotNone(stop_verification_error(
            stopped, 1_000_000_000, 900_000_000, 1_000_000_000,
            1_100_000_000, 2_100_000_000, 1, 1,
        ))
        self.assertIsNotNone(stop_verification_error(
            stopped, 2_000_000_000, 900_000_000, 1_000_000_000,
            1_100_000_000, 2_100_000_000, 0, 1,
        ))
        moving = list(stopped)
        moving[3] = 145.0
        self.assertIsNotNone(stop_verification_error(
            tuple(moving), 2_000_000_000, 900_000_000, 1_000_000_000,
            1_100_000_000, 2_100_000_000, 1, 1,
        ))
        still_spinning = list(stopped)
        still_spinning[1] = 1.1
        self.assertIsNotNone(stop_verification_error(
            tuple(still_spinning), 2_000_000_000, 900_000_000, 1_000_000_000,
            1_100_000_000, 2_100_000_000, 1, 1,
        ))
        self.assertIsNotNone(stop_verification_error(
            stopped, 2_000_000_000, 900_000_000, 1_000_000_000,
            1_450_000_000, 2_100_000_000, 1, 1,
        ))
        self.assertIsNotNone(stop_verification_error(
            stopped, 2_000_000_000, 900_000_000, 1_000_000_000,
            1_100_000_000, 2_100_000_000, 1, 2,
        ))

    def test_battery_summary_uses_drive_interval(self) -> None:
        readings = [(100, 8.01), (200, 7.91), (250, 7.85), (400, 7.99)]
        self.assertEqual(battery_summary(readings, 150, 300), {
            "battery_before_drive_v": 8.01,
            "battery_min_during_drive_v": 7.85,
            "battery_after_stop_v": 7.99,
        })

    def test_motion_trial_rejects_implausible_2s_battery_voltage(self) -> None:
        self.assertIsNotNone(battery_voltage_error(math.nan))
        self.assertIsNotNone(battery_voltage_error(math.inf))
        self.assertIsNotNone(battery_voltage_error(TRIAL_BATTERY_MIN_V - 0.001))
        self.assertIsNone(battery_voltage_error(TRIAL_BATTERY_MIN_V))
        self.assertIsNotNone(battery_voltage_error(3.63))
        self.assertIsNone(battery_voltage_error(7.4))
        self.assertIsNone(battery_voltage_error(TRIAL_BATTERY_MAX_V))
        self.assertIsNotNone(battery_voltage_error(TRIAL_BATTERY_MAX_V + 0.01))

    def test_battery_message_requires_a_fresh_valid_mcu_sample(self) -> None:
        self.assertIsNone(battery_message_error(8.0, True))
        self.assertIsNotNone(battery_message_error(8.0, False))
        self.assertIsNotNone(battery_message_error(math.nan, True))

    def test_rearm_requires_fresh_fault_free_motor_telemetry(self) -> None:
        start_ns = 1_000_000_000
        self.assertIsNotNone(startup_fault_rearm_error(start_ns, None, False))
        self.assertIsNotNone(startup_fault_rearm_error(start_ns, start_ns, False))
        self.assertIsNotNone(startup_fault_rearm_error(start_ns, start_ns + 1, True))
        self.assertIsNone(startup_fault_rearm_error(start_ns, start_ns + 1, False))

    def test_motion_trial_requires_one_second_of_in_range_battery_voltage(self) -> None:
        start_ns = 1_000_000_000
        self.assertIsNotNone(battery_readiness_error(None, None, None, start_ns))
        self.assertIsNotNone(battery_readiness_error(3.63, None, None, start_ns))
        self.assertIsNotNone(battery_readiness_error(
            8.0, start_ns, start_ns, start_ns,
        ))
        self.assertIsNotNone(battery_readiness_error(
            8.0, start_ns, start_ns + BATTERY_STABLE_NS - 1,
            start_ns + BATTERY_STABLE_NS - 1,
        ))
        self.assertIsNone(battery_readiness_error(
            8.0, start_ns, start_ns + BATTERY_STABLE_NS,
            start_ns + BATTERY_STABLE_NS,
        ))
        self.assertIsNotNone(battery_readiness_error(
            8.0,
            start_ns,
            start_ns,
            start_ns + BATTERY_FRESH_NS + 1,
        ))

    def test_battery_stability_restarts_after_error_or_long_gap(self) -> None:
        start_ns = 1_000_000_000
        self.assertEqual(next_battery_valid_since_ns(
            8.0, None, None, start_ns,
        ), start_ns)
        self.assertEqual(next_battery_valid_since_ns(
            8.0, start_ns, start_ns + 100, start_ns + 200,
        ), start_ns)
        self.assertIsNone(next_battery_valid_since_ns(
            3.63, start_ns, start_ns + 100, start_ns + 200,
        ))
        after_gap_ns = start_ns + BATTERY_FRESH_NS + 1
        self.assertEqual(next_battery_valid_since_ns(
            8.0, start_ns, start_ns, after_gap_ns,
        ), after_gap_ns)

    def test_nonzero_commands_cannot_bypass_health_gate(self) -> None:
        self.assertIsNone(motion_command_safety_error(0.05, None))
        self.assertEqual(
            motion_command_safety_error(0.05, "battery not ready"),
            "battery not ready",
        )
        self.assertEqual(
            motion_command_safety_error(-0.05, "battery not ready"),
            "battery not ready",
        )
        # A zero command must remain publishable while unhealthy so the abort
        # path can actively stop the robot.
        self.assertIsNone(motion_command_safety_error(0.0, "battery not ready"))
        self.assertIsNotNone(motion_command_safety_error(math.nan, None))


if __name__ == "__main__":
    unittest.main()
