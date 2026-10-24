"""Step-response metrics shared by speed_step_test.py and summarize_steps.py."""

import statistics


def step_metrics(samples, t0, target, steady_window_s):
    """Rise time (10-90 %), overshoot and steady-state stats for one step."""
    resp = [(t - t0, v) for t, v in samples if t >= t0]
    sign = 1.0 if target >= 0 else -1.0
    mag = abs(target)

    def first_cross(level):
        return next((t for t, v in resp if sign * v >= level * mag), None)

    t10, t90 = first_cross(0.1), first_cross(0.9)
    peak = max((sign * v for _, v in resp), default=0.0)
    t_end = resp[-1][0] if resp else 0.0
    steady = [sign * v for t, v in resp if t >= t_end - steady_window_s]
    mean = statistics.fmean(steady) if steady else float("nan")
    std = statistics.pstdev(steady) if len(steady) > 1 else float("nan")
    return {
        "target": target,
        "rise_ms": (t90 - t10) * 1000 if t10 is not None and t90 is not None else float("nan"),
        "t90_ms": t90 * 1000 if t90 is not None else float("nan"),
        "overshoot_pct": max(0.0, (peak - mag) / mag * 100),
        "ss_error_pct": (mean - mag) / mag * 100,
        "ripple_pct": std / mag * 100,
    }
