#!/usr/bin/env python3
"""Offline checks for the 19-field /motor_debug timing analyzer."""

import csv
import importlib.util
import io
import json
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "tools" / "analyze_motor_timing.py"
SPEC = importlib.util.spec_from_file_location("analyze_motor_timing", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
ANALYZER = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = ANALYZER
SPEC.loader.exec_module(ANALYZER)


def sample(row, host_s, control_seq, control_ms, publish_seq, publish_ms):
    return ANALYZER.TimingSample(
        row, host_s, control_seq, control_ms, publish_seq, publish_ms,
    )


def interval(before, after):
    return ANALYZER.analyze([before, after])[0]


def csv_text(samples, fields=None):
    fields = list(fields or ("time_s", *ANALYZER.MOTOR_FIELDS, *ANALYZER.TIMING_FIELDS))
    output = io.StringIO()
    writer = csv.DictWriter(output, fieldnames=fields)
    writer.writeheader()
    for item in samples:
        row = {name: "0" for name in ANALYZER.MOTOR_FIELDS}
        row.update({
            "time_s": item.host_time_s,
            "control_seq_mod65536": item.control_seq,
            "control_time_ms_hi16": item.control_time_ms >> 16,
            "control_time_ms_lo16": item.control_time_ms & 0xffff,
            "publish_seq_mod65536": item.publish_seq,
            "publish_time_ms_hi16": item.publish_time_ms >> 16,
            "publish_time_ms_lo16": item.publish_time_ms & 0xffff,
        })
        writer.writerow({field: row[field] for field in fields})
    return output.getvalue()


class AnalyzeMotorTimingTest(unittest.TestCase):
    def test_nominal_and_simultaneous_counter_timestamp_wrap(self):
        before = sample(2, 0.1, 65535, 0xfffffff6, 65535, 0xfffffff6)
        after = sample(3, 0.12, 0, 10, 0, 10)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "motor_debug.csv"
            path.write_text(csv_text([before, after]), encoding="utf-8")
            loaded = ANALYZER.load_samples(path)
        gap = interval(*loaded)
        self.assertEqual(gap.control_seq_delta, 1)
        self.assertEqual(gap.publish_seq_delta, 1)
        self.assertEqual(gap.control_time_delta_ms, 20)
        self.assertEqual(gap.publish_time_delta_ms, 20)
        self.assertEqual(gap.classification, "nominal")

    def test_missing_publish_attempts_are_not_called_network_loss(self):
        gap = interval(sample(2, .1, 10, 100, 10, 100),
                       sample(3, .16, 13, 160, 13, 160))
        self.assertEqual(gap.classification, "publish_attempts_not_observed")
        self.assertIn("2 intermediate", gap.explanation)
        self.assertIn("cannot be separated", gap.explanation)

    def test_mcu_snapshot_and_publisher_gaps_are_distinguished(self):
        before = sample(2, .1, 10, 100, 10, 100)
        control_gap = interval(before, sample(3, .24, 11, 240, 11, 240))
        publish_gap = interval(before, sample(3, .24, 17, 240, 11, 240))
        self.assertEqual(control_gap.classification, "mcu_control_snapshot_gap")
        self.assertEqual(publish_gap.classification, "mcu_publish_attempt_gap")

    def test_host_gap_is_downstream_evidence_not_unique_cause(self):
        gap = interval(sample(2, .1, 10, 100, 10, 100),
                       sample(3, .24, 11, 120, 11, 120))
        self.assertEqual(gap.classification, "host_arrival_gap")
        self.assertIn("not isolated", gap.explanation)

    def test_control_snapshot_jump_and_repeat_are_reported_cautiously(self):
        before = sample(2, .1, 10, 100, 10, 100)
        jump = interval(before, sample(3, .12, 12, 120, 11, 120))
        repeat = interval(before, sample(3, .12, 10, 100, 11, 120))
        self.assertEqual(jump.classification, "control_snapshots_not_observed_by_publisher")
        self.assertEqual(repeat.classification, "repeated_control_snapshot")
        self.assertIn("phase jitter", jump.explanation)
        self.assertIn("phase jitter", repeat.explanation)

    def test_duplicate_and_ambiguous_counters(self):
        before = sample(2, .1, 10, 100, 10, 100)
        duplicate = interval(before, sample(3, .12, 10, 100, 10, 100))
        ambiguous = interval(before, sample(3, .12, 11, 120, 9, 120))
        self.assertEqual(duplicate.classification, "duplicate_publish_metadata")
        self.assertEqual(ambiguous.classification, "ambiguous_counter_jump")

    def test_invalid_schema_and_nonintegral_field_are_rejected(self):
        first = sample(2, .1, 10, 100, 10, 100)
        second = sample(3, .12, 11, 120, 11, 120)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "motor_debug.csv"
            fields = ("time_s", *ANALYZER.MOTOR_FIELDS)
            path.write_text(csv_text([first, second], fields=fields), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "19-value"):
                ANALYZER.load_samples(path)
            broken = csv_text([first, second]).replace(",10,0,100,10,0,100", ",10.5,0,100,10,0,100", 1)
            path.write_text(broken, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "exact integer"):
                ANALYZER.load_samples(path)
            legacy_padded = csv_text([first, second]).replace(",10,0,100,10,0,100", ",,,,,,", 1)
            path.write_text(legacy_padded, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "old 13-value firmware"):
                ANALYZER.load_samples(path)

    def test_json_cli_reports_only_diagnostics_by_default(self):
        rows = [sample(2, .1, 10, 100, 10, 100),
                sample(3, .12, 11, 120, 11, 120),
                sample(4, .26, 12, 140, 12, 140)]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "motor_debug.csv"
            path.write_text(csv_text(rows), encoding="utf-8")
            output = io.StringIO()
            with redirect_stdout(output), redirect_stderr(io.StringIO()):
                result = ANALYZER.main([str(path), "--json"])
        document = json.loads(output.getvalue())
        self.assertEqual(result, 0)
        self.assertEqual(document["sample_count"], 3)
        self.assertEqual(document["interval_count"], 2)
        self.assertEqual(document["diagnostic_count"], 1)
        self.assertEqual(document["reported_interval_count"], 1)
        self.assertEqual(document["gaps"][0]["classification"], "host_arrival_gap")

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "motor_debug.csv"
            path.write_text(csv_text(rows), encoding="utf-8")
            output = io.StringIO()
            with redirect_stdout(output), redirect_stderr(io.StringIO()):
                result = ANALYZER.main([str(path), "--json", "--all"])
        document = json.loads(output.getvalue())
        self.assertEqual(result, 0)
        self.assertEqual(document["diagnostic_count"], 1)
        self.assertEqual(document["reported_interval_count"], 2)


if __name__ == "__main__":
    unittest.main()
