#!/usr/bin/env python3
"""Summarize a leap_mavlink_bridge --csv log.

Separates the firmware's publish timing (mcu_ms, the firmware tick count, so it
moves in tick steps) from what the PC saw (host_s), so a late frame can be put
on the MCU or on the link.

Usage:
  python3 tools/bridge_log_stats.py data/bridge_hw_*/listen_idle.csv [...]
"""

import argparse
import csv


def percentile(sorted_values, p):
    i = round(p * (len(sorted_values) - 1))
    return sorted_values[min(i, len(sorted_values) - 1)]


def gaps(values, scale):
    return sorted((b - a) * scale for a, b in zip(values, values[1:]))


def describe(name, g):
    return (f"{name}: p50 {percentile(g, 0.50):.1f}  p95 {percentile(g, 0.95):.1f}  "
            f"p99 {percentile(g, 0.99):.1f}  max {g[-1]:.1f} ms")


def summarize(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if len(rows) < 2:
        print(f"{path}: fewer than two odometry frames")
        return
    host = [float(r["host_s"]) for r in rows]
    mcu = [int(r["mcu_ms"]) for r in rows]
    last = rows[-1]
    duration = host[-1] - host[0]
    print(f"{path}")
    print(f"  {len(rows)} odometry frames over {duration:.1f} s = {(len(rows) - 1) / duration:.2f} Hz; "
          f"link frames {last['frames']}, lost {last['lost']}, crc errors {last['crc']}")
    print("  " + describe("host arrival gaps", gaps(host, 1000.0)))
    print("  " + describe("MCU publish gaps ", gaps(mcu, 1.0)))
    # A firmware reset restarts its clock; flag it rather than hide it in the gaps.
    resets = sum(1 for a, b in zip(mcu, mcu[1:]) if b < a)
    if resets:
        print(f"  firmware clock went backwards {resets} time(s): MCU reset during the run")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("logs", nargs="+")
    for path in parser.parse_args().logs:
        summarize(path)


if __name__ == "__main__":
    main()
