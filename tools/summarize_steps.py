#!/usr/bin/env python3
"""Summarize speed_step_test.py CSV files, averaging repeated runs.

Files are grouped by name with a trailing _r<N> run suffix removed, so
kp1.2_ki6.0_r1.csv and kp1.2_ki6.0_r2.csv form one group. For each group
it prints per-target mean overshoot, steady-state error, ripple and t90, and
a score line used to pick PID gains:

  ok     every target has |steady error| < --max-ss-error and t90 < --max-t90
  low_os mean overshoot over targets with |v| <= --low-speed (both directions)

Usage:
  python3 tools/summarize_steps.py data/pid_grid/*.csv
"""

import argparse
import csv
import math
import os
import re
import statistics
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from step_metrics import step_metrics  # noqa: E402


def load(path, steady_s):
    by_target = defaultdict(list)
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            by_target[float(row["target_mps"])].append((float(row["t_since_step_s"]), float(row["vx_mps"])))
    return {target: step_metrics(samples, 0.0, target, steady_s) for target, samples in by_target.items()}


def mean(values):
    values = [v for v in values if not math.isnan(v)]
    return statistics.fmean(values) if values else float("nan")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", nargs="+")
    parser.add_argument("--steady-s", type=float, default=1.5)
    parser.add_argument("--max-ss-error", type=float, default=1.0, help="percent")
    parser.add_argument("--max-t90", type=float, default=250.0, help="ms")
    parser.add_argument("--low-speed", type=float, default=0.1, help="m/s, targets scored for overshoot")
    args = parser.parse_args()

    groups = defaultdict(list)
    for path in args.csv:
        name = re.sub(r"_r\d+$", "", os.path.splitext(os.path.basename(path))[0])
        groups[name].append(load(path, args.steady_s))

    scores = []
    for name, runs in sorted(groups.items()):
        targets = sorted(runs[0], key=lambda t: (abs(t), -t))
        print(f"== {name} ({len(runs)} run{'s' if len(runs) > 1 else ''})")
        print(f"{'target':>8} {'overshoot':>10} {'ss error':>9} {'ripple':>7} {'t90 ms':>7}")
        ok = True
        for target in targets:
            os_ = mean([r[target]["overshoot_pct"] for r in runs])
            ss = mean([r[target]["ss_error_pct"] for r in runs])
            rip = mean([r[target]["ripple_pct"] for r in runs])
            t90 = mean([r[target]["t90_ms"] for r in runs])
            ok &= abs(ss) < args.max_ss_error and not math.isnan(t90) and t90 < args.max_t90
            print(f"{target:>+8.2f} {os_:>9.1f}% {ss:>+8.1f}% {rip:>6.1f}% {t90:>7.0f}")
        low_os = mean([r[t]["overshoot_pct"] for r in runs for t in targets if abs(t) <= args.low_speed])
        scores.append((not ok, low_os, name))
        print(f"   ok={ok} low_os={low_os:.1f}%")

    print("\nranking (meets limits first, then lowest low-speed overshoot):")
    for rank, (not_ok, low_os, name) in enumerate(sorted(scores), 1):
        print(f"{rank:>2}. {name:<20} low_os={low_os:5.1f}%  {'ok' if not not_ok else 'fails limits'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
