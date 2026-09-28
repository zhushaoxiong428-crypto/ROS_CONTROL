#!/usr/bin/env python3
"""Quantify left/right wheel startup symmetry from firmware or ROS CSV logs."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
import sys
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Iterable, Sequence


IDF_LINE = re.compile(
    r"\((?P<time_ms>\d+)\).*?MOTOR_DEBUG: send "
    r"L:\((?P<lt>-?[\d.]+) (?P<la>-?[\d.]+) "
    r"(?P<lp>-?[\d.]+) (?P<lo>-?[\d.]+)\) "
    r"R:\((?P<rt>-?[\d.]+) (?P<ra>-?[\d.]+) "
    r"(?P<rp>-?[\d.]+) (?P<ro>-?[\d.]+)\)"
)

MCU_TIMING_FIELDS = (
    "control_seq_mod65536",
    "control_time_ms_hi16",
    "control_time_ms_lo16",
    "publish_seq_mod65536",
    "publish_time_ms_hi16",
    "publish_time_ms_lo16",
)
CONTROL_PAYLOAD_FIELDS = (
    "left_target_rpm",
    "left_actual_rpm",
    "left_raw_pwm",
    "left_final_pwm",
    "right_target_rpm",
    "right_actual_rpm",
    "right_raw_pwm",
    "right_final_pwm",
    "sync_error_rpm",
    "sync_correction_pwm",
    "sync_active",
    "startup_active",
    "startup_fault",
)
UINT16_MODULUS = 1 << 16
UINT32_MODULUS = 1 << 32
MAX_CONTIGUOUS_CONTROL_INTERVAL_MS = 30
CONTROL_CADENCE_TOLERANCE_MS = 1.0


@dataclass(frozen=True)
class Sample:
    time_s: float
    left_target_rpm: float
    left_actual_rpm: float
    left_raw_pwm: float
    left_final_pwm: float
    right_target_rpm: float
    right_actual_rpm: float
    right_raw_pwm: float
    right_final_pwm: float
    control_seq: int | None = None
    control_time_ms: int | None = None
    gap_before: bool = False


@dataclass(frozen=True)
class LoadDiagnostics:
    time_source: str
    input_samples: int
    output_samples: int
    duplicate_control_snapshots: int = 0
    missing_control_snapshots: int = 0
    control_gap_count: int = 0
    control_seq_wraps: int = 0
    control_time_wraps: int = 0


@dataclass(frozen=True)
class LoadResult:
    samples: list[Sample]
    diagnostics: LoadDiagnostics


@dataclass(frozen=True)
class CandidateInterval:
    lower: float
    upper: float


@dataclass(frozen=True)
class StartupMetrics:
    run: int
    target_rpm: float
    sample_period_ms: float
    left_t10_ms: float | None
    right_t10_ms: float | None
    left_t90_ms: float | None
    right_t90_ms: float | None
    t90_difference_ms: float | None
    peak_speed_difference_500ms_rpm: float | None
    mean_right_minus_left_1s_rpm: float | None
    wheel_distance_difference_1s_mm: float | None
    predicted_yaw_1s_deg: float | None
    tracking_settle_ms: float | None
    sync_settle_ms: float | None
    steady_left_mean_rpm: float | None
    steady_right_mean_rpm: float | None
    steady_mean_difference_rpm: float | None
    steady_difference_p95_rpm: float | None
    steady_left_rmse_rpm: float | None
    steady_right_rmse_rpm: float | None
    left_t90_candidate_interval_ms: CandidateInterval | None
    right_t90_candidate_interval_ms: CandidateInterval | None
    right_t90_lead_candidate_interval_ms: CandidateInterval | None
    observed_peak_speed_difference_500ms_lower_bound_rpm: float | None
    time_source: str
    control_gap_count: int


def parse_idf_log(path: Path) -> list[Sample]:
    samples: list[Sample] = []
    with path.open("r", encoding="utf-8", errors="ignore") as stream:
        for line in stream:
            match = IDF_LINE.search(line)
            if not match:
                continue
            values = {key: float(value) for key, value in match.groupdict().items()}
            samples.append(
                Sample(
                    time_s=values["time_ms"] / 1000.0,
                    left_target_rpm=values["lt"],
                    left_actual_rpm=values["la"],
                    left_raw_pwm=values["lp"],
                    left_final_pwm=values["lo"],
                    right_target_rpm=values["rt"],
                    right_actual_rpm=values["ra"],
                    right_raw_pwm=values["rp"],
                    right_final_pwm=values["ro"],
                )
            )
    return samples


def _parse_uint(row: dict[str, str], field: str, bits: int) -> int:
    text = row.get(field)
    if text is None or text.strip() == "":
        raise ValueError(f"new-schema CSV field {field!r} is blank")
    value = float(text)
    limit = 1 << bits
    if not math.isfinite(value) or not value.is_integer() or not 0.0 <= value < limit:
        raise ValueError(f"new-schema CSV field {field!r} must be uint{bits}, got {text!r}")
    return int(value)


def _sample_from_csv_row(row: dict[str, str], time_s: float) -> Sample:
    return Sample(
        time_s=time_s,
        left_target_rpm=float(row["left_target_rpm"]),
        left_actual_rpm=float(row["left_actual_rpm"]),
        left_raw_pwm=float(row["left_raw_pwm"]),
        left_final_pwm=float(row["left_final_pwm"]),
        right_target_rpm=float(row["right_target_rpm"]),
        right_actual_rpm=float(row["right_actual_rpm"]),
        right_raw_pwm=float(row["right_raw_pwm"]),
        right_final_pwm=float(row["right_final_pwm"]),
    )


def _control_payload(row: dict[str, str]) -> tuple[float, ...]:
    values: list[float] = []
    for field in CONTROL_PAYLOAD_FIELDS:
        text = row.get(field)
        if text is not None and text.strip() != "":
            value = float(text)
            if not math.isfinite(value):
                raise ValueError(f"CSV field {field!r} is not finite")
            values.append(value)
    return tuple(values)


def parse_csv_log_with_diagnostics(path: Path) -> LoadResult:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        fieldnames = set(reader.fieldnames or ())
        rows = list(reader)

    timing_columns_present = [field in fieldnames for field in MCU_TIMING_FIELDS]
    if any(timing_columns_present) and not all(timing_columns_present):
        missing = [
            field for field, present in zip(MCU_TIMING_FIELDS, timing_columns_present)
            if not present
        ]
        raise ValueError(f"CSV has only part of the MCU timing schema; missing {missing}")

    if not all(timing_columns_present):
        samples = [_sample_from_csv_row(row, float(row["time_s"])) for row in rows]
        return LoadResult(
            samples,
            LoadDiagnostics("host", len(rows), len(samples)),
        )

    row_timing_states = []
    for row in rows:
        populated = [bool((row.get(field) or "").strip()) for field in MCU_TIMING_FIELDS]
        if any(populated) and not all(populated):
            raise ValueError("CSV row has a partially populated MCU timing schema")
        row_timing_states.append(all(populated))

    if not any(row_timing_states):
        samples = [_sample_from_csv_row(row, float(row["time_s"])) for row in rows]
        return LoadResult(
            samples,
            LoadDiagnostics("host", len(rows), len(samples)),
        )
    if not all(row_timing_states):
        raise ValueError("CSV mixes MCU-timed rows with rows that have blank timing fields")

    samples: list[Sample] = []
    duplicate_count = 0
    missing_count = 0
    gap_count = 0
    seq_wraps = 0
    time_wraps = 0
    previous_raw_seq: int | None = None
    previous_raw_time: int | None = None
    previous_payload: tuple[float, ...] | None = None
    unwrapped_time_ms: int | None = None

    for row in rows:
        raw_seq = _parse_uint(row, "control_seq_mod65536", 16)
        time_hi = _parse_uint(row, "control_time_ms_hi16", 16)
        time_lo = _parse_uint(row, "control_time_ms_lo16", 16)
        # Validate the publish fields too: partial/corrupt metadata must not be
        # mistaken for trustworthy MCU-timed evidence.
        _parse_uint(row, "publish_seq_mod65536", 16)
        _parse_uint(row, "publish_time_ms_hi16", 16)
        _parse_uint(row, "publish_time_ms_lo16", 16)
        raw_time = (time_hi << 16) | time_lo
        payload = _control_payload(row)

        if previous_raw_seq is None:
            unwrapped_time_ms = raw_time
            samples.append(replace(
                _sample_from_csv_row(row, raw_time / 1000.0),
                control_seq=raw_seq,
                control_time_ms=raw_time,
                gap_before=True,
            ))
            gap_count += 1
        else:
            assert previous_raw_time is not None
            assert previous_payload is not None
            assert unwrapped_time_ms is not None
            seq_delta = (raw_seq - previous_raw_seq) % UINT16_MODULUS
            time_delta = (raw_time - previous_raw_time) % UINT32_MODULUS

            if seq_delta == 0 and time_delta == 0:
                if payload != previous_payload:
                    raise ValueError("duplicate MCU seq/time carries different control payload")
                duplicate_count += 1
                continue
            if seq_delta == 0 or time_delta == 0:
                raise ValueError("MCU control sequence and timestamp did not advance together")
            if seq_delta >= UINT16_MODULUS // 2:
                raise ValueError("MCU control sequence moved backward or reset")
            if time_delta >= UINT32_MODULUS // 2:
                raise ValueError("MCU control timestamp moved backward or reset")

            if raw_seq < previous_raw_seq:
                seq_wraps += 1
            if raw_time < previous_raw_time:
                time_wraps += 1
            unwrapped_time_ms += time_delta
            gap_before = (
                seq_delta != 1 or time_delta > MAX_CONTIGUOUS_CONTROL_INTERVAL_MS
            )
            if seq_delta > 1:
                missing_count += seq_delta - 1
            if gap_before:
                gap_count += 1
            samples.append(replace(
                _sample_from_csv_row(row, unwrapped_time_ms / 1000.0),
                control_seq=(samples[-1].control_seq or 0) + seq_delta,
                control_time_ms=unwrapped_time_ms,
                gap_before=gap_before,
            ))

        previous_raw_seq = raw_seq
        previous_raw_time = raw_time
        previous_payload = payload

    return LoadResult(
        samples,
        LoadDiagnostics(
            time_source="mcu_control",
            input_samples=len(rows),
            output_samples=len(samples),
            duplicate_control_snapshots=duplicate_count,
            missing_control_snapshots=missing_count,
            control_gap_count=gap_count,
            control_seq_wraps=seq_wraps,
            control_time_wraps=time_wraps,
        ),
    )


def parse_csv_log(path: Path) -> list[Sample]:
    return parse_csv_log_with_diagnostics(path).samples


def load_samples_with_diagnostics(path: Path) -> LoadResult:
    if path.suffix.lower() == ".csv":
        return parse_csv_log_with_diagnostics(path)
    samples = parse_idf_log(path)
    return LoadResult(samples, LoadDiagnostics("idf_log", len(samples), len(samples)))


def load_samples(path: Path) -> list[Sample]:
    return load_samples_with_diagnostics(path).samples


def split_straight_runs(samples: Sequence[Sample]) -> list[list[Sample]]:
    runs: list[list[Sample]] = []
    current: list[Sample] = []
    for sample in samples:
        target_scale = max(
            abs(sample.left_target_rpm), abs(sample.right_target_rpm), 1.0
        )
        is_straight = (
            abs(sample.left_target_rpm) >= 0.5
            and sample.left_target_rpm * sample.right_target_rpm > 0.0
            and abs(sample.left_target_rpm - sample.right_target_rpm)
            <= max(0.05, target_scale * 0.01)
        )
        if is_straight:
            current.append(sample)
        elif current:
            if len(current) >= 10:
                runs.append(current)
            current = []
    if len(current) >= 10:
        runs.append(current)
    return runs


def percentile(values: Sequence[float], probability: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return math.nan
    index = max(0, math.ceil(probability * len(ordered)) - 1)
    return ordered[index]


def first_crossing_ms(
    elapsed_samples: Sequence[tuple[float, Sample]],
    threshold_rpm: float,
    side: str,
    direction: float,
) -> float | None:
    attribute = f"{side}_actual_rpm"
    continuous_prefix = True
    for elapsed, sample in elapsed_samples:
        if sample.control_seq is not None and sample.gap_before:
            continuous_prefix = False
        if direction * getattr(sample, attribute) >= threshold_rpm:
            return elapsed * 1000.0 if continuous_prefix else None
    return None


def crossing_candidate_interval_ms(
    elapsed_samples: Sequence[tuple[float, Sample]],
    threshold_rpm: float,
    side: str,
    direction: float,
    nominal_period_ms: float,
) -> CandidateInterval | None:
    """Bracket a local observed MCU crossing without claiming it was first.

    Earlier missing samples can hide a crossing followed by a drop, so this is
    deliberately separate from ``first_crossing_ms`` and is never used by the
    strict acceptance check. When snapshots are missing immediately before the
    first received over-threshold sample, a fixed-cadence MCU sequence can only
    narrow the candidate tick to an interval.
    """
    if not math.isfinite(nominal_period_ms) or nominal_period_ms <= 0.0:
        return None

    attribute = f"{side}_actual_rpm"
    previous: tuple[float, Sample] | None = None
    for elapsed, sample in elapsed_samples:
        if direction * getattr(sample, attribute) < threshold_rpm:
            previous = (elapsed, sample)
            continue

        if previous is None:
            return None
        previous_elapsed, previous_sample = previous
        if previous_sample.control_seq is None or sample.control_seq is None:
            return None

        sequence_delta = sample.control_seq - previous_sample.control_seq
        if sequence_delta <= 0 or elapsed <= previous_elapsed:
            return None
        if sequence_delta == 1:
            crossing_ms = elapsed * 1000.0
            return CandidateInterval(crossing_ms, crossing_ms)

        elapsed_per_tick_ms = (
            (elapsed - previous_elapsed) * 1000.0 / sequence_delta
        )
        if abs(elapsed_per_tick_ms - nominal_period_ms) > CONTROL_CADENCE_TOLERANCE_MS:
            return None
        earliest_ms = previous_elapsed * 1000.0 + nominal_period_ms
        latest_ms = elapsed * 1000.0
        return CandidateInterval(earliest_ms, latest_ms)

    return None


def subtract_candidate_intervals(
    left: CandidateInterval | None,
    right: CandidateInterval | None,
) -> CandidateInterval | None:
    if left is None or right is None:
        return None
    return CandidateInterval(
        left.lower - right.upper,
        left.upper - right.lower,
    )


def first_continuous_window_ms(
    elapsed_samples: Sequence[tuple[float, Sample]],
    predicate,
    duration_s: float = 0.2,
) -> float | None:
    # Once evidence is missing, a later in-band sample cannot prove when the
    # *first* continuous settling window began. Only search the complete prefix.
    first_gap_index = next(
        (
            index for index, (_, sample) in enumerate(elapsed_samples)
            if sample.control_seq is not None and sample.gap_before
        ),
        len(elapsed_samples),
    )
    elapsed_samples = elapsed_samples[:first_gap_index]
    max_gap_s = maximum_sample_gap_s(elapsed_samples)
    for start_index, (start_time, _) in enumerate(elapsed_samples):
        end_time = start_time + duration_s
        valid = True
        previous_time = None
        for elapsed, sample in elapsed_samples[start_index:]:
            if previous_time is not None and elapsed - previous_time > max_gap_s:
                valid = False
                break
            if not predicate(sample):
                valid = False
                break
            if elapsed >= end_time:
                return start_time * 1000.0
            previous_time = elapsed
        if not valid:
            continue
    return None


def maximum_sample_gap_s(elapsed_samples: Sequence[tuple[float, Sample]]) -> float:
    """Allow normal recorder jitter, but never bridge a large missing interval."""
    intervals = [
        current[0] - previous[0]
        for previous, current in zip(elapsed_samples, elapsed_samples[1:])
        if current[0] > previous[0] and not current[1].gap_before
    ]
    if not intervals:
        return 0.0
    return min(0.1, max(0.05, 3.5 * statistics.median(intervals)))


def complete_prefix_window(
    elapsed_samples: Sequence[tuple[float, Sample]],
    horizon_s: float,
    max_gap_s: float,
) -> list[tuple[float, Sample]] | None:
    """Return 0..horizon, clipping the last sample only across a small gap."""
    if not elapsed_samples or elapsed_samples[0][0] != 0.0:
        return None
    if elapsed_samples[0][1].control_seq is not None and elapsed_samples[0][1].gap_before:
        return None
    selected = [elapsed_samples[0]]
    for (previous_time, previous), (time_s, current) in zip(
        elapsed_samples, elapsed_samples[1:]
    ):
        dt = time_s - previous_time
        if current.gap_before or dt <= 0.0 or dt > max_gap_s:
            return None
        if time_s >= horizon_s:
            if time_s == horizon_s:
                selected.append((time_s, current))
            else:
                fraction = (horizon_s - previous_time) / dt
                endpoint = replace(
                    previous,
                    time_s=previous.time_s + horizon_s - previous_time,
                    left_actual_rpm=previous.left_actual_rpm + fraction * (
                        current.left_actual_rpm - previous.left_actual_rpm
                    ),
                    right_actual_rpm=previous.right_actual_rpm + fraction * (
                        current.right_actual_rpm - previous.right_actual_rpm
                    ),
                )
                selected.append((horizon_s, endpoint))
            return selected
        selected.append((time_s, current))
    return None


def steady_plateau_window(
    elapsed_samples: Sequence[tuple[float, Sample]],
    target_rpm: float,
    direction: float,
    max_gap_s: float,
) -> list[Sample] | None:
    """Return a complete last second entirely after the final target plateau."""
    last_time = elapsed_samples[-1][0]
    window_start = last_time - 1.0
    if window_start < 0.0:
        return None

    target_tolerance = max(0.05, target_rpm * 0.01)
    plateau = lambda sample: (
        abs(direction * sample.left_target_rpm - target_rpm) <= target_tolerance
        and abs(direction * sample.right_target_rpm - target_rpm) <= target_tolerance
    )
    # A sample just before the boundary and one just after it establish that
    # the requested second is covered, rather than merely containing samples.
    before = [item for item in elapsed_samples if item[0] <= window_start]
    after = [item for item in elapsed_samples if item[0] >= window_start]
    if not before or not after:
        return None
    boundary = before[-1]
    steady = [boundary, *(item for item in after if item[0] > boundary[0])]
    if not all(plateau(sample) for _, sample in steady):
        return None
    if any(
        current[1].gap_before or
        current[0] - previous[0] > max_gap_s or current[0] <= previous[0]
        for previous, current in zip(steady, steady[1:])
    ):
        return None
    return [sample for _, sample in after]


def integrate_distance_difference_mm(
    elapsed_samples: Sequence[tuple[float, Sample]],
    direction: float,
    wheel_diameter_mm: float,
    horizon_s: float,
) -> float:
    selected = [(t, sample) for t, sample in elapsed_samples if t <= horizon_s]
    return (
        integrate_speed_difference_rpm_s(selected, direction)
        * math.pi * wheel_diameter_mm / 60.0
    )


def integrate_speed_difference_rpm_s(
    elapsed_samples: Sequence[tuple[float, Sample]],
    direction: float,
) -> float:
    signed_difference_rpm_s = 0.0
    for (previous_time, previous), (time_s, current) in zip(
        elapsed_samples, elapsed_samples[1:]
    ):
        dt = time_s - previous_time
        previous_difference = direction * (
            previous.right_actual_rpm - previous.left_actual_rpm
        )
        current_difference = direction * (
            current.right_actual_rpm - current.left_actual_rpm
        )
        mean_difference_rpm = (previous_difference + current_difference) * 0.5
        signed_difference_rpm_s += mean_difference_rpm * dt
    return signed_difference_rpm_s


def analyse_run(
    run_number: int,
    samples: Sequence[Sample],
    wheel_diameter_mm: float,
    track_width_mm: float,
) -> StartupMetrics:
    start_time = samples[0].time_s
    elapsed_samples = [(sample.time_s - start_time, sample) for sample in samples]
    direction = 1.0 if samples[0].left_target_rpm > 0.0 else -1.0

    plateau_candidates = [
        direction * sample.left_target_rpm
        for elapsed, sample in elapsed_samples
        if elapsed >= max(0.0, elapsed_samples[-1][0] - 0.5)
    ]
    target_rpm = statistics.median(plateau_candidates)
    sample_periods = [
        current.time_s - previous.time_s
        for previous, current in zip(samples, samples[1:])
        if current.time_s > previous.time_s and not current.gap_before
    ]
    sample_period_ms = (
        statistics.median(sample_periods) * 1000.0
        if sample_periods else math.nan
    )
    time_source = "mcu_control" if any(
        sample.control_seq is not None for sample in samples
    ) else "host"
    control_gap_count = sum(
        sample.control_seq is not None and sample.gap_before for sample in samples
    )

    left_t10 = first_crossing_ms(elapsed_samples, target_rpm * 0.10, "left", direction)
    right_t10 = first_crossing_ms(elapsed_samples, target_rpm * 0.10, "right", direction)
    left_t90 = first_crossing_ms(elapsed_samples, target_rpm * 0.90, "left", direction)
    right_t90 = first_crossing_ms(elapsed_samples, target_rpm * 0.90, "right", direction)
    t90_difference = (
        abs(left_t90 - right_t90)
        if left_t90 is not None and right_t90 is not None
        else None
    )
    left_t90_candidate = crossing_candidate_interval_ms(
        elapsed_samples, target_rpm * 0.90, "left", direction, sample_period_ms
    )
    right_t90_candidate = crossing_candidate_interval_ms(
        elapsed_samples, target_rpm * 0.90, "right", direction, sample_period_ms
    )
    right_t90_lead_candidate = subtract_candidate_intervals(
        left_t90_candidate, right_t90_candidate
    )

    max_gap_s = maximum_sample_gap_s(elapsed_samples)
    first_500ms = complete_prefix_window(elapsed_samples, 0.5, max_gap_s)
    first_1s = complete_prefix_window(elapsed_samples, 1.0, max_gap_s)
    peak_difference_500ms = (
        max(abs(sample.right_actual_rpm - sample.left_actual_rpm)
            for _, sample in first_500ms)
        if first_500ms else None
    )
    observed_first_500ms = [
        sample for elapsed, sample in elapsed_samples
        if 0.0 <= elapsed <= 0.5 + 1e-9
    ]
    observed_peak_difference_500ms_lower_bound = (
        max(abs(sample.right_actual_rpm - sample.left_actual_rpm)
            for sample in observed_first_500ms)
        if observed_first_500ms else None
    )
    # The complete window is exactly 1 s, so its rpm·s integral is mean rpm.
    mean_difference_1s = (
        integrate_speed_difference_rpm_s(first_1s, direction)
        if first_1s else None
    )
    distance_difference_1s = (
        integrate_distance_difference_mm(first_1s, direction, wheel_diameter_mm, 1.0)
        if first_1s else None
    )
    predicted_yaw_1s_deg = (
        math.degrees(distance_difference_1s / track_width_mm)
        if distance_difference_1s is not None else None
    )

    tracking_band = target_rpm * 0.05
    tracking_settle = first_continuous_window_ms(
        elapsed_samples,
        lambda sample: (
            abs(direction * sample.left_actual_rpm - target_rpm) <= tracking_band
            and abs(direction * sample.right_actual_rpm - target_rpm) <= tracking_band
        ),
    )
    sync_settle = first_continuous_window_ms(
        elapsed_samples,
        lambda sample: abs(sample.left_actual_rpm - sample.right_actual_rpm)
        <= tracking_band,
    )

    steady = steady_plateau_window(
        elapsed_samples, target_rpm, direction,
        max_gap_s,
    )
    steady_left = [direction * sample.left_actual_rpm for sample in steady or []]
    steady_right = [direction * sample.right_actual_rpm for sample in steady or []]
    steady_differences = [abs(right - left) for left, right in zip(steady_left, steady_right)]

    return StartupMetrics(
        run=run_number,
        target_rpm=target_rpm,
        sample_period_ms=sample_period_ms,
        left_t10_ms=left_t10,
        right_t10_ms=right_t10,
        left_t90_ms=left_t90,
        right_t90_ms=right_t90,
        t90_difference_ms=t90_difference,
        peak_speed_difference_500ms_rpm=peak_difference_500ms,
        mean_right_minus_left_1s_rpm=mean_difference_1s,
        wheel_distance_difference_1s_mm=distance_difference_1s,
        predicted_yaw_1s_deg=predicted_yaw_1s_deg,
        tracking_settle_ms=tracking_settle,
        sync_settle_ms=sync_settle,
        steady_left_mean_rpm=statistics.fmean(steady_left) if steady else None,
        steady_right_mean_rpm=statistics.fmean(steady_right) if steady else None,
        steady_mean_difference_rpm=(
            abs(statistics.fmean(steady_right) - statistics.fmean(steady_left))
            if steady else None
        ),
        steady_difference_p95_rpm=percentile(steady_differences, 0.95) if steady else None,
        steady_left_rmse_rpm=(
            math.sqrt(statistics.fmean((value - target_rpm) ** 2 for value in steady_left))
            if steady else None
        ),
        steady_right_rmse_rpm=(
            math.sqrt(statistics.fmean((value - target_rpm) ** 2 for value in steady_right))
            if steady else None
        ),
        left_t90_candidate_interval_ms=left_t90_candidate,
        right_t90_candidate_interval_ms=right_t90_candidate,
        right_t90_lead_candidate_interval_ms=right_t90_lead_candidate,
        observed_peak_speed_difference_500ms_lower_bound_rpm=(
            observed_peak_difference_500ms_lower_bound
        ),
        time_source=time_source,
        control_gap_count=control_gap_count,
    )


def check_metrics(metrics: StartupMetrics) -> list[str]:
    failures: list[str] = []
    target = metrics.target_rpm
    steady_available = metrics.steady_left_mean_rpm is not None
    checks = (
        (metrics.t90_difference_ms is not None and metrics.t90_difference_ms <= 20.0,
         "t90 difference > 20 ms"),
        (metrics.tracking_settle_ms is not None and metrics.tracking_settle_ms <= 500.0,
         "tracking settle time > 500 ms"),
    )
    for passed, reason in checks:
        if not passed:
            failures.append(reason)
    if metrics.peak_speed_difference_500ms_rpm is None:
        failures.append("complete first-500-ms window unavailable")
    elif metrics.peak_speed_difference_500ms_rpm > target * 0.10:
        failures.append("peak first-500-ms speed difference > 10% target")
    if metrics.predicted_yaw_1s_deg is None:
        failures.append("complete first-1-s window unavailable")
    elif abs(metrics.predicted_yaw_1s_deg) > 1.5:
        failures.append("predicted first-1-s yaw > 1.5 deg")
    if not steady_available:
        failures.append("complete 1-s target-plateau steady window unavailable")
    else:
        assert metrics.steady_mean_difference_rpm is not None
        assert metrics.steady_difference_p95_rpm is not None
        assert metrics.steady_left_rmse_rpm is not None
        assert metrics.steady_right_rmse_rpm is not None
        steady_checks = (
            (metrics.steady_mean_difference_rpm <= target * 0.03,
             "steady mean wheel difference > 3% target"),
            (metrics.steady_difference_p95_rpm <= target * 0.05,
             "steady wheel-difference p95 > 5% target"),
            (metrics.steady_left_rmse_rpm <= 1.0,
             "left steady RMSE > 1 rpm"),
            (metrics.steady_right_rmse_rpm <= 1.0,
             "right steady RMSE > 1 rpm"),
        )
        for passed, reason in steady_checks:
            if not passed:
                failures.append(reason)
    return failures


def format_optional(value: float | None, digits: int = 1) -> str:
    return "N/A" if value is None else f"{value:.{digits}f}"


def format_candidate_interval(
    interval: CandidateInterval | None,
    digits: int = 1,
) -> str:
    if interval is None:
        return "N/A"
    if math.isclose(interval.lower, interval.upper, abs_tol=1e-6):
        return f"{interval.lower:.{digits}f}"
    return f"[{interval.lower:.{digits}f}, {interval.upper:.{digits}f}]"


def print_report(metrics: Iterable[StartupMetrics]) -> None:
    for item in metrics:
        print(
            f"Run {item.run}: target={item.target_rpm:.2f} rpm, "
            f"dt={item.sample_period_ms:.1f} ms, time={item.time_source}, "
            f"control_gaps={item.control_gap_count}"
        )
        print(
            "  t10 L/R="
            f"{format_optional(item.left_t10_ms)}/{format_optional(item.right_t10_ms)} ms, "
            "t90 L/R="
            f"{format_optional(item.left_t90_ms)}/{format_optional(item.right_t90_ms)} ms"
        )
        print(
            "  first 500 ms peak |R-L|="
            f"{format_optional(item.peak_speed_difference_500ms_rpm, 2)} rpm"
        )
        print(
            "  observed-only evidence (not strict first-crossing proof): "
            "t90 candidate L/R="
            f"{format_candidate_interval(item.left_t90_candidate_interval_ms)}/"
            f"{format_candidate_interval(item.right_t90_candidate_interval_ms)} ms, "
            "right-lead candidate="
            f"{format_candidate_interval(item.right_t90_lead_candidate_interval_ms)} ms, "
            "first-500-ms peak lower bound >="
            f"{format_optional(item.observed_peak_speed_difference_500ms_lower_bound_rpm, 3)} rpm"
        )
        print(
            "  first 1 s mean R-L="
            f"{format_optional(item.mean_right_minus_left_1s_rpm, 2)} rpm, "
            f"wheel delta={format_optional(item.wheel_distance_difference_1s_mm, 2)} mm, "
            f"predicted yaw={format_optional(item.predicted_yaw_1s_deg, 2)} deg"
        )
        print(
            "  settle tracking/sync="
            f"{format_optional(item.tracking_settle_ms)}/"
            f"{format_optional(item.sync_settle_ms)} ms"
        )
        print(
            "  steady (last 1 s after target plateau) L/R="
            f"{format_optional(item.steady_left_mean_rpm, 2)}/"
            f"{format_optional(item.steady_right_mean_rpm, 2)} rpm, "
            f"mean delta={format_optional(item.steady_mean_difference_rpm, 2)}, "
            f"p95 delta={format_optional(item.steady_difference_p95_rpm, 2)} rpm, "
            f"RMSE L/R={format_optional(item.steady_left_rmse_rpm, 2)}/"
            f"{format_optional(item.steady_right_rmse_rpm, 2)} rpm"
        )


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, help="IDF monitor log or recorder CSV")
    parser.add_argument("--wheel-diameter-mm", type=float, default=65.0)
    parser.add_argument("--track-width-mm", type=float, default=131.7)
    parser.add_argument("--json", action="store_true", help="emit JSON")
    parser.add_argument(
        "--check",
        action="store_true",
        help="return non-zero when a run misses the documented acceptance limits",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        loaded = load_samples_with_diagnostics(args.log)
    except (KeyError, TypeError, ValueError) as error:
        print(f"Invalid startup log: {error}", file=sys.stderr)
        return 2
    samples = loaded.samples
    diagnostics = loaded.diagnostics
    if diagnostics.time_source == "mcu_control":
        print(
            "MCU timing: "
            f"input={diagnostics.input_samples}, unique={diagnostics.output_samples}, "
            f"duplicates={diagnostics.duplicate_control_snapshots}, "
            f"missing={diagnostics.missing_control_snapshots}, "
            f"gaps={diagnostics.control_gap_count}, "
            f"seq_wraps={diagnostics.control_seq_wraps}, "
            f"time_wraps={diagnostics.control_time_wraps}",
            file=sys.stderr,
        )
    runs = split_straight_runs(samples)
    if not runs:
        print("No straight-line startup runs found", file=sys.stderr)
        return 2

    metrics = [
        analyse_run(index, run, args.wheel_diameter_mm, args.track_width_mm)
        for index, run in enumerate(runs, start=1)
    ]
    if args.json:
        print(json.dumps([asdict(item) for item in metrics], indent=2))
    else:
        print_report(metrics)

    if args.check:
        failed = False
        for item in metrics:
            failures = check_metrics(item)
            if failures:
                failed = True
                print(f"Run {item.run} FAILED: {', '.join(failures)}", file=sys.stderr)
        return 1 if failed else 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
