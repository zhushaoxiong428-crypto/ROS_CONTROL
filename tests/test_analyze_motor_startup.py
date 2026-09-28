#!/usr/bin/env python3

import importlib.util
import io
import json
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "tools" / "analyze_motor_startup.py"
SPEC = importlib.util.spec_from_file_location("analyze_motor_startup", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
ANALYSER = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = ANALYSER
SPEC.loader.exec_module(ANALYSER)


class AnalyzeMotorStartupTest(unittest.TestCase):
    MCU_HEADER = (
        "time_s,left_target_rpm,left_actual_rpm,left_raw_pwm,left_final_pwm,"
        "right_target_rpm,right_actual_rpm,right_raw_pwm,right_final_pwm,"
        "control_seq_mod65536,control_time_ms_hi16,control_time_ms_lo16,"
        "publish_seq_mod65536,publish_time_ms_hi16,publish_time_ms_lo16\n"
    )

    @staticmethod
    def mcu_row(host_time, target, actual, control_seq, control_time_ms, publish_seq):
        control_hi, control_lo = divmod(control_time_ms, 1 << 16)
        publish_time_ms = control_time_ms + 3
        publish_hi, publish_lo = divmod(publish_time_ms % (1 << 32), 1 << 16)
        return (
            f"{host_time},{target},{actual},1,145,"
            f"{target},{actual},1,145,"
            f"{control_seq},{control_hi},{control_lo},"
            f"{publish_seq},{publish_hi},{publish_lo}\n"
        )

    @staticmethod
    def make_run(duration_s=3.4, plateau_from_s=0.3, omit=None):
        samples = []
        for index in range(round(duration_s / 0.02) + 1):
            time_s = round(index * 0.02, 3)
            if omit is not None and omit[0] < time_s < omit[1]:
                continue
            target = 30.0 if time_s >= plateau_from_s else 2.0 + 28.0 * time_s / plateau_from_s
            actual = 30.0 if time_s >= 0.5 else 0.0
            samples.append(
                ANALYSER.Sample(time_s, target, actual, 145.0, 145.0,
                                target, actual, 145.0, 145.0)
            )
        return samples

    @staticmethod
    def timed_sample(time_s, left_actual, right_actual, sequence, gap_before=False):
        target = 14.691225
        return ANALYSER.Sample(
            time_s=time_s,
            left_target_rpm=target,
            left_actual_rpm=left_actual,
            left_raw_pwm=145.0,
            left_final_pwm=145.0,
            right_target_rpm=target,
            right_actual_rpm=right_actual,
            right_raw_pwm=145.0,
            right_final_pwm=145.0,
            control_seq=sequence,
            control_time_ms=round(time_s * 1000.0),
            gap_before=gap_before,
        )

    def test_parses_idf_log_and_splits_run(self) -> None:
        lines = []
        for index in range(12):
            time_ms = 1000 + index * 20
            target = min(20.0, 2.0 + index * 2.0)
            actual = max(0.0, target - 2.0)
            lines.append(
                f"I ({time_ms}) MOTOR_DEBUG: send "
                f"L:({target:.2f} {actual:.2f} 1.00 145.00) "
                f"R:({target:.2f} {actual:.2f} 1.00 151.00)\n"
            )

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "monitor.log"
            path.write_text("".join(lines), encoding="utf-8")
            samples = ANALYSER.load_samples(path)

        self.assertEqual(len(samples), 12)
        runs = ANALYSER.split_straight_runs(samples)
        self.assertEqual(len(runs), 1)
        self.assertAlmostEqual(runs[0][0].time_s, 1.0)

    def test_csv_parser_keeps_first_eight_fields(self) -> None:
        header = (
            "time_s,left_target_rpm,left_actual_rpm,left_raw_pwm,left_final_pwm,"
            "right_target_rpm,right_actual_rpm,right_raw_pwm,right_final_pwm,"
            "sync_error_rpm,sync_correction_pwm,sync_active,startup_active\n"
        )
        row = "0.02,2,0,2.2,145,2,0,2.2,145,0,0,1,1\n"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "motor.csv"
            path.write_text(header + row, encoding="utf-8")
            samples = ANALYSER.load_samples(path)

        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0].left_final_pwm, 145.0)
        self.assertEqual(samples[0].right_final_pwm, 145.0)

    def test_mcu_csv_uses_control_time_and_deduplicates(self) -> None:
        start_ms = (2 << 16) + 65516
        rows = (
            self.mcu_row(0.10, 2, 0, 10, start_ms, 100),
            self.mcu_row(0.10, 4, 1, 11, start_ms + 20, 101),
            self.mcu_row(0.60, 4, 1, 11, start_ms + 20, 102),
            self.mcu_row(0.61, 6, 2, 12, start_ms + 40, 103),
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mcu.csv"
            path.write_text(self.MCU_HEADER + "".join(rows), encoding="utf-8")
            loaded = ANALYSER.load_samples_with_diagnostics(path)

        self.assertEqual(len(loaded.samples), 3)
        self.assertAlmostEqual(loaded.samples[1].time_s - loaded.samples[0].time_s, 0.02)
        self.assertAlmostEqual(loaded.samples[2].time_s - loaded.samples[1].time_s, 0.02)
        self.assertEqual(loaded.diagnostics.time_source, "mcu_control")
        self.assertEqual(loaded.diagnostics.duplicate_control_snapshots, 1)
        self.assertEqual(loaded.diagnostics.missing_control_snapshots, 0)
        self.assertFalse(loaded.samples[1].gap_before)

    def test_mcu_csv_marks_missing_control_snapshot(self) -> None:
        rows = (
            self.mcu_row(0.0, 2, 0, 10, 1000, 100),
            self.mcu_row(0.01, 6, 3, 12, 1040, 101),
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "gap.csv"
            path.write_text(self.MCU_HEADER + "".join(rows), encoding="utf-8")
            loaded = ANALYSER.load_samples_with_diagnostics(path)

        self.assertEqual(loaded.diagnostics.missing_control_snapshots, 1)
        self.assertTrue(loaded.samples[1].gap_before)
        elapsed = [
            (sample.time_s - loaded.samples[0].time_s, sample)
            for sample in loaded.samples
        ]
        self.assertIsNone(ANALYSER.first_crossing_ms(elapsed, 1.0, "left", 1.0))
        self.assertIsNone(
            ANALYSER.complete_prefix_window(elapsed, 0.04, 0.1)
        )

    def test_candidate_crossing_brackets_missing_control_ticks(self) -> None:
        threshold = 13.0
        samples = [
            self.timed_sample(0.0, 0.0, 0.0, 10),
            self.timed_sample(0.04, 14.0, 0.0, 12, gap_before=True),
        ]
        elapsed = [(sample.time_s, sample) for sample in samples]

        self.assertIsNone(
            ANALYSER.first_crossing_ms(elapsed, threshold, "left", 1.0)
        )
        candidate = ANALYSER.crossing_candidate_interval_ms(
            elapsed, threshold, "left", 1.0, 20.0
        )
        self.assertIsNotNone(candidate)
        self.assertAlmostEqual(candidate.lower, 20.0)
        self.assertAlmostEqual(candidate.upper, 40.0)

        right = ANALYSER.CandidateInterval(240.0, 260.0)
        lead = ANALYSER.subtract_candidate_intervals(
            ANALYSER.CandidateInterval(700.0, 720.0), right
        )
        self.assertEqual(lead, ANALYSER.CandidateInterval(440.0, 480.0))

    def test_candidate_crossing_remains_local_and_requires_valid_mcu_cadence(self) -> None:
        threshold = 13.0
        earlier_gap = [
            self.timed_sample(0.0, 0.0, 0.0, 10),
            self.timed_sample(0.04, 0.0, 0.0, 12, gap_before=True),
            self.timed_sample(0.06, 14.0, 0.0, 13),
        ]
        elapsed = [(sample.time_s, sample) for sample in earlier_gap]
        self.assertIsNone(
            ANALYSER.first_crossing_ms(elapsed, threshold, "left", 1.0)
        )
        self.assertEqual(
            ANALYSER.crossing_candidate_interval_ms(
                elapsed, threshold, "left", 1.0, 20.0
            ),
            ANALYSER.CandidateInterval(60.0, 60.0),
        )

        bad_cadence = [
            self.timed_sample(0.0, 0.0, 0.0, 20),
            self.timed_sample(0.06, 14.0, 0.0, 22, gap_before=True),
        ]
        self.assertIsNone(
            ANALYSER.crossing_candidate_interval_ms(
                [(sample.time_s, sample) for sample in bad_cadence],
                threshold, "left", 1.0, 20.0,
            )
        )

        host_samples = [
            ANALYSER.Sample(0.0, 14.7, 0.0, 1.0, 145.0,
                            14.7, 0.0, 1.0, 145.0),
            ANALYSER.Sample(0.02, 14.7, 14.0, 1.0, 145.0,
                            14.7, 0.0, 1.0, 145.0),
        ]
        self.assertIsNone(
            ANALYSER.crossing_candidate_interval_ms(
                [(sample.time_s, sample) for sample in host_samples],
                threshold, "left", 1.0, 20.0,
            )
        )

    def test_observed_peak_lower_bound_survives_gap_and_stops_at_500ms(self) -> None:
        samples = []
        for index in range(61):
            if index == 10:
                continue
            time_s = index * 0.02
            difference = 5.0 if index == 25 else 10.0 if index == 26 else 0.0
            samples.append(self.timed_sample(
                time_s, 0.0, difference, 100 + index,
                gap_before=index == 11,
            ))

        metrics = ANALYSER.analyse_run(1, samples, 65.0, 131.7)
        self.assertIsNone(metrics.peak_speed_difference_500ms_rpm)
        self.assertAlmostEqual(
            metrics.observed_peak_speed_difference_500ms_lower_bound_rpm,
            5.0,
        )

    def test_real_smoke_logs_reproduce_candidate_evidence(self) -> None:
        expected = (
            ((680.0, 700.0), (260.0, 260.0), (420.0, 440.0), 4.995032),
            ((700.0, 720.0), (240.0, 260.0), (440.0, 480.0), 5.170067),
            ((660.0, 660.0), (240.0, 240.0), (420.0, 420.0), 5.807579),
        )
        repository = Path(__file__).parents[1]
        for run_number, values in enumerate(expected, start=1):
            path = repository / "data" / f"trial_scan320_suspended_smoke_0{run_number}" / "motor_debug.csv"
            loaded = ANALYSER.load_samples_with_diagnostics(path)
            run = ANALYSER.split_straight_runs(loaded.samples)[0]
            metrics = ANALYSER.analyse_run(run_number, run, 65.0, 131.7)
            intervals = (
                metrics.left_t90_candidate_interval_ms,
                metrics.right_t90_candidate_interval_ms,
                metrics.right_t90_lead_candidate_interval_ms,
            )
            for interval, bounds in zip(intervals, values[:3]):
                self.assertIsNotNone(interval)
                self.assertAlmostEqual(interval.lower, bounds[0], places=5)
                self.assertAlmostEqual(interval.upper, bounds[1], places=5)
            self.assertAlmostEqual(
                metrics.observed_peak_speed_difference_500ms_lower_bound_rpm,
                values[3],
                places=5,
            )

    def test_mcu_csv_unwraps_sequence_and_timestamp(self) -> None:
        rows = (
            self.mcu_row(0.0, 2, 0, 65535, 0xfffffff0, 200),
            self.mcu_row(0.1, 4, 1, 0, 4, 201),
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "wrap.csv"
            path.write_text(self.MCU_HEADER + "".join(rows), encoding="utf-8")
            loaded = ANALYSER.load_samples_with_diagnostics(path)

        self.assertEqual(loaded.samples[1].control_seq, 65536)
        self.assertEqual(loaded.diagnostics.control_seq_wraps, 1)
        self.assertEqual(loaded.diagnostics.control_time_wraps, 1)
        self.assertAlmostEqual(loaded.samples[1].time_s - loaded.samples[0].time_s, 0.02)
        self.assertFalse(loaded.samples[1].gap_before)

    def test_mcu_csv_rejects_conflicting_duplicate(self) -> None:
        rows = (
            self.mcu_row(0.0, 2, 0, 10, 1000, 100),
            self.mcu_row(0.1, 4, 0, 10, 1000, 101),
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad_duplicate.csv"
            path.write_text(self.MCU_HEADER + "".join(rows), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "different control payload"):
                ANALYSER.load_samples_with_diagnostics(path)

    def test_mcu_csv_rejects_partial_timing_row(self) -> None:
        row = self.mcu_row(0.0, 2, 0, 10, 1000, 100).rstrip("\n").split(",")
        row[-1] = ""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "partial.csv"
            path.write_text(self.MCU_HEADER + ",".join(row) + "\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "partially populated"):
                ANALYSER.load_samples_with_diagnostics(path)

    def test_short_run_uses_only_last_second_after_target_plateau(self) -> None:
        metrics = ANALYSER.analyse_run(1, self.make_run(), 65.0, 131.7)
        self.assertAlmostEqual(metrics.steady_left_mean_rpm, 30.0)
        self.assertAlmostEqual(metrics.steady_right_mean_rpm, 30.0)
        self.assertAlmostEqual(metrics.steady_left_rmse_rpm, 0.0)

    def test_short_plateau_is_unavailable_and_check_fails(self) -> None:
        samples = self.make_run(plateau_from_s=3.0)
        metrics = ANALYSER.analyse_run(1, samples, 65.0, 131.7)
        self.assertIsNone(metrics.steady_left_mean_rpm)
        self.assertIsNone(metrics.steady_right_rmse_rpm)
        self.assertIn("steady window unavailable", " ".join(ANALYSER.check_metrics(metrics)))

        header = (
            "time_s,left_target_rpm,left_actual_rpm,left_raw_pwm,left_final_pwm,"
            "right_target_rpm,right_actual_rpm,right_raw_pwm,right_final_pwm\n"
        )
        rows = (
            f"{s.time_s},{s.left_target_rpm},{s.left_actual_rpm},145,145,"
            f"{s.right_target_rpm},{s.right_actual_rpm},145,145\n"
            for s in samples
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "short.csv"
            path.write_text(header + "".join(rows), encoding="utf-8")
            output, errors = io.StringIO(), io.StringIO()
            with redirect_stdout(output), redirect_stderr(errors):
                status = ANALYSER.main([str(path), "--check"])
            json_output = io.StringIO()
            with redirect_stdout(json_output), redirect_stderr(io.StringIO()):
                json_status = ANALYSER.main([str(path), "--json", "--check"])
        self.assertEqual(status, 1)
        self.assertIn("N/A", output.getvalue())
        self.assertIn("steady window unavailable", errors.getvalue())
        self.assertEqual(json_status, 1)
        self.assertIsNone(json.loads(json_output.getvalue())[0]["steady_left_mean_rpm"])

    def test_large_sample_gap_cannot_create_false_settle_or_steady(self) -> None:
        samples = self.make_run(omit=(2.6, 2.9))
        metrics = ANALYSER.analyse_run(1, samples, 65.0, 131.7)
        self.assertIsNone(metrics.steady_left_mean_rpm)

        gapped = [(t, samples[0]) for t in (0.0, 0.02, 0.04, 0.30, 0.32, 0.34)]
        self.assertIsNone(
            ANALYSER.first_continuous_window_ms(gapped, lambda _: True)
        )

    def test_short_pulse_has_500ms_but_not_first_second_metrics(self) -> None:
        samples = self.make_run(duration_s=0.54)
        metrics = ANALYSER.analyse_run(1, samples, 65.0, 131.7)
        self.assertIsNotNone(metrics.peak_speed_difference_500ms_rpm)
        self.assertIsNone(metrics.mean_right_minus_left_1s_rpm)
        self.assertIsNone(metrics.wheel_distance_difference_1s_mm)
        self.assertIsNone(metrics.predicted_yaw_1s_deg)
        self.assertIn("complete first-1-s window unavailable", ANALYSER.check_metrics(metrics))

        header = (
            "time_s,left_target_rpm,left_actual_rpm,left_raw_pwm,left_final_pwm,"
            "right_target_rpm,right_actual_rpm,right_raw_pwm,right_final_pwm\n"
        )
        rows = (
            f"{s.time_s},{s.left_target_rpm},{s.left_actual_rpm},145,145,"
            f"{s.right_target_rpm},{s.right_actual_rpm},145,145\n"
            for s in samples
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pulse.csv"
            path.write_text(header + "".join(rows), encoding="utf-8")
            output, errors = io.StringIO(), io.StringIO()
            with redirect_stdout(output), redirect_stderr(errors):
                status = ANALYSER.main([str(path), "--check"])
            json_output = io.StringIO()
            with redirect_stdout(json_output), redirect_stderr(io.StringIO()):
                json_status = ANALYSER.main([str(path), "--json", "--check"])
        self.assertEqual(status, 1)
        self.assertEqual(json_status, 1)
        self.assertIn("first 1 s mean R-L=N/A", output.getvalue())
        self.assertIn("complete first-1-s window unavailable", errors.getvalue())
        self.assertIsNone(json.loads(json_output.getvalue())[0]["predicted_yaw_1s_deg"])

    def test_prefix_gaps_invalidate_only_affected_windows(self) -> None:
        early = ANALYSER.analyse_run(
            1, self.make_run(omit=(0.3, 0.48)), 65.0, 131.7
        )
        self.assertIsNone(early.peak_speed_difference_500ms_rpm)
        self.assertIsNone(early.mean_right_minus_left_1s_rpm)
        self.assertIsNone(early.wheel_distance_difference_1s_mm)
        self.assertIn("complete first-500-ms window unavailable", ANALYSER.check_metrics(early))

        late = ANALYSER.analyse_run(
            1, self.make_run(omit=(0.6, 0.8)), 65.0, 131.7
        )
        self.assertIsNotNone(late.peak_speed_difference_500ms_rpm)
        self.assertIsNone(late.predicted_yaw_1s_deg)

    def test_long_run_preserves_complete_startup_windows(self) -> None:
        metrics = ANALYSER.analyse_run(1, self.make_run(duration_s=6.0), 65.0, 131.7)
        self.assertIsNotNone(metrics.peak_speed_difference_500ms_rpm)
        self.assertIsNotNone(metrics.mean_right_minus_left_1s_rpm)
        self.assertIsNotNone(metrics.wheel_distance_difference_1s_mm)
        self.assertIsNotNone(metrics.predicted_yaw_1s_deg)


if __name__ == "__main__":
    unittest.main()
