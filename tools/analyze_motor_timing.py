#!/usr/bin/env python3
"""Locate /motor_debug timing gaps using MCU counters and host receive times.

The publish counter records MCU *attempts* to publish, not delivery ACKs. This
tool describes evidence in the CSV; it cannot uniquely identify a network,
agent, DDS, or host scheduling fault.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path


MOTOR_FIELDS = (
    "left_target_rpm", "left_actual_rpm", "left_raw_pwm", "left_final_pwm",
    "right_target_rpm", "right_actual_rpm", "right_raw_pwm", "right_final_pwm",
    "sync_error_rpm", "sync_correction_pwm", "sync_active", "startup_active",
    "startup_fault",
)
TIMING_FIELDS = (
    "control_seq_mod65536",
    "control_time_ms_hi16",
    "control_time_ms_lo16",
    "publish_seq_mod65536",
    "publish_time_ms_hi16",
    "publish_time_ms_lo16",
)
COUNTER_MODULUS = 1 << 16
TIME_MODULUS = 1 << 32


@dataclass(frozen=True)
class TimingSample:
    row: int
    host_time_s: float
    control_seq: int
    control_time_ms: int
    publish_seq: int
    publish_time_ms: int


@dataclass(frozen=True)
class TimingGap:
    from_row: int
    to_row: int
    host_gap_ms: float
    control_seq_delta: int
    control_time_delta_ms: int
    publish_seq_delta: int
    publish_time_delta_ms: int
    classification: str
    explanation: str


def _uint16(value: str, row: int, field: str) -> int:
    if value is None or value == "":
        raise ValueError(
            f"row {row}: {field} is missing; this row may come from old 13-value firmware"
        )
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"row {row}: {field} is not a number: {value!r}") from error
    if not math.isfinite(number) or not number.is_integer() or not 0 <= number < COUNTER_MODULUS:
        raise ValueError(f"row {row}: {field} must be an exact integer in 0..65535")
    return int(number)


def load_samples(path: Path) -> list[TimingSample]:
    samples: list[TimingSample] = []
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError("empty CSV: expected time_s and the six timing fields")
        required = ("time_s", *MOTOR_FIELDS, *TIMING_FIELDS)
        missing = [field for field in required if field not in reader.fieldnames]
        if missing:
            raise ValueError(
                "CSV lacks timing fields " + ", ".join(missing) +
                "; this tool needs the 19-value /motor_debug schema, not an older 13-value log"
            )
        for row_number, data in enumerate(reader, start=2):
            if None in data:
                raise ValueError(f"row {row_number}: more values than CSV header")
            try:
                host_time_s = float(data["time_s"])
            except (TypeError, ValueError) as error:
                raise ValueError(f"row {row_number}: time_s is not a number") from error
            if not math.isfinite(host_time_s) or host_time_s < 0:
                raise ValueError(f"row {row_number}: time_s must be finite and non-negative")
            parts = {field: _uint16(data[field], row_number, field) for field in TIMING_FIELDS}
            if samples and host_time_s <= samples[-1].host_time_s:
                raise ValueError(f"row {row_number}: host time_s must strictly increase")
            samples.append(TimingSample(
                row=row_number,
                host_time_s=host_time_s,
                control_seq=parts["control_seq_mod65536"],
                control_time_ms=(parts["control_time_ms_hi16"] << 16) |
                                parts["control_time_ms_lo16"],
                publish_seq=parts["publish_seq_mod65536"],
                publish_time_ms=(parts["publish_time_ms_hi16"] << 16) |
                                parts["publish_time_ms_lo16"],
            ))
    if len(samples) < 2:
        raise ValueError("need at least two timing samples to compare intervals")
    return samples


def classify_gap(
    control_seq_delta: int,
    control_time_delta_ms: int,
    publish_seq_delta: int,
    publish_time_delta_ms: int,
    host_gap_ms: float,
    expected_ms: float,
    gap_factor: float,
) -> tuple[str, str]:
    """Classify observable evidence, never claim a unique physical root cause."""
    large_host_gap = host_gap_ms > expected_ms * gap_factor
    large_publish_gap = publish_time_delta_ms > expected_ms * gap_factor
    large_control_gap = control_time_delta_ms > expected_ms * gap_factor

    # A half-range jump could be wrap, reboot, or an enormous unobserved span.
    if (control_seq_delta >= COUNTER_MODULUS // 2 or
            publish_seq_delta >= COUNTER_MODULUS // 2 or
            control_time_delta_ms >= TIME_MODULUS // 2 or
            publish_time_delta_ms >= TIME_MODULUS // 2):
        return ("ambiguous_counter_jump",
                "Counter/time jump exceeds half its modulus; reset, wrap, or a long gap cannot be distinguished.")
    if publish_seq_delta == 0:
        if publish_time_delta_ms == 0:
            return ("duplicate_publish_metadata",
                    "Same publish attempt metadata appeared twice; this may be a duplicate sample or repeated payload.")
        return ("inconsistent_publish_metadata",
                "Publish timestamp changed without advancing the publish-attempt counter; inspect instrumentation/reset behavior.")
    if publish_time_delta_ms == 0:
        return ("inconsistent_publish_metadata",
                "Publish-attempt counter advanced but its millisecond timestamp did not; timing resolution or instrumentation may be insufficient.")
    if publish_seq_delta > 1:
        return ("publish_attempts_not_observed",
                f"Assuming no counter reset, {publish_seq_delta - 1} intermediate MCU publish "
                "attempt(s) are absent from this host CSV; loss, delayed delivery, or "
                "recorder behavior cannot be separated here.")
    if large_publish_gap:
        if control_seq_delta == 1 and large_control_gap:
            return ("mcu_control_snapshot_gap",
                    "Consecutive publish attempts contain consecutive control snapshots separated by a long MCU interval; "
                    "the control task/timestamp path needs inspection, but a task stall is not proven.")
        return ("mcu_publish_attempt_gap",
                "Consecutive observed publish attempts are far apart on the MCU clock; "
                "check publisher scheduling. Control snapshots may have continued meanwhile.")
    if control_seq_delta > 1:
        return ("control_snapshots_not_observed_by_publisher",
                f"Assuming no counter reset, {control_seq_delta - 1} intermediate control "
                "snapshot(s) were not represented in this pair of consecutive publish attempts. "
                "One such jump may arise from normal phase jitter between the two periodic tasks."
                + (" Host arrival was also delayed." if large_host_gap else ""))
    if control_seq_delta == 0:
        return ("repeated_control_snapshot",
                "Consecutive publish attempts reported the same control snapshot. "
                "An isolated repeat may arise from normal phase jitter between the two periodic tasks."
                + (" Host arrival was also delayed." if large_host_gap else ""))
    if large_host_gap:
        return ("host_arrival_gap",
                "Assuming no counter reset, MCU publish attempts/timestamps are consecutive "
                "and near their nominal spacing, "
                "but host receive times are far apart; delivery or host scheduling is implicated, not isolated.")
    return ("nominal", "No large interval or missing publish attempt is evident in this pair.")


def analyze(samples: list[TimingSample], expected_ms: float = 20.0,
            gap_factor: float = 2.5) -> list[TimingGap]:
    if not math.isfinite(expected_ms) or expected_ms <= 0:
        raise ValueError("expected_ms must be finite and positive")
    if not math.isfinite(gap_factor) or gap_factor <= 1:
        raise ValueError("gap_factor must be finite and greater than one")
    gaps: list[TimingGap] = []
    for previous, current in zip(samples, samples[1:]):
        control_seq_delta = (current.control_seq - previous.control_seq) % COUNTER_MODULUS
        publish_seq_delta = (current.publish_seq - previous.publish_seq) % COUNTER_MODULUS
        control_time_delta_ms = (current.control_time_ms - previous.control_time_ms) % TIME_MODULUS
        publish_time_delta_ms = (current.publish_time_ms - previous.publish_time_ms) % TIME_MODULUS
        host_gap_ms = (current.host_time_s - previous.host_time_s) * 1000.0
        classification, explanation = classify_gap(
            control_seq_delta, control_time_delta_ms, publish_seq_delta,
            publish_time_delta_ms, host_gap_ms, expected_ms, gap_factor,
        )
        gaps.append(TimingGap(
            from_row=previous.row, to_row=current.row,
            host_gap_ms=host_gap_ms,
            control_seq_delta=control_seq_delta,
            control_time_delta_ms=control_time_delta_ms,
            publish_seq_delta=publish_seq_delta,
            publish_time_delta_ms=publish_time_delta_ms,
            classification=classification, explanation=explanation,
        ))
    return gaps


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", type=Path, help="new-schema motor_debug.csv")
    parser.add_argument("--expected-ms", type=float, default=20.0,
                        help="nominal MCU publish interval (default: 20 ms)")
    parser.add_argument("--gap-factor", type=float, default=2.5,
                        help="interval greater than this times nominal is large (default: 2.5)")
    parser.add_argument("--all", action="store_true", help="print nominal intervals too")
    parser.add_argument("--json", action="store_true", help="emit machine-readable diagnostics")
    args = parser.parse_args(argv)
    try:
        samples = load_samples(args.csv_path)
        gaps = analyze(samples, args.expected_ms, args.gap_factor)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    diagnostics = [gap for gap in gaps if gap.classification != "nominal"]
    selected = gaps if args.all else diagnostics
    if args.json:
        print(json.dumps({
            "sample_count": len(samples),
            "interval_count": len(gaps),
            "diagnostic_count": len(diagnostics),
            "reported_interval_count": len(selected),
            "expected_ms": args.expected_ms,
            "gap_factor": args.gap_factor,
            "gaps": [asdict(gap) for gap in selected],
        }, indent=2))
    else:
        print(f"{len(samples)} samples, {len(gaps)} intervals, "
              f"{len(diagnostics)} diagnostic interval(s), {len(selected)} reported")
        for gap in selected:
            print(
                f"rows {gap.from_row}->{gap.to_row}: host {gap.host_gap_ms:.1f} ms; "
                f"control Δseq={gap.control_seq_delta}, Δt={gap.control_time_delta_ms} ms; "
                f"publish Δseq={gap.publish_seq_delta}, Δt={gap.publish_time_delta_ms} ms; "
                f"{gap.classification}\n  {gap.explanation}"
            )
        if not diagnostics:
            print("No diagnostic gaps at this threshold; this does not prove zero packet loss or real-time behavior.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
