"""Step-response metrics shared by speed_step_test.py and summarize_steps.py."""

import statistics


def step_metrics(samples, t0, target, steady_window_s):
    """Rise time (10-90 %), overshoot and steady-state stats for one step.

    Overshoot is the peak before the steady window, so it is not inflated by
    measurement noise: at 0.05 m/s one encoder count per 20 ms cycle is ~4 %
    of the speed, and the largest of a few hundred noisy samples says nothing
    about the step. noise_pct is the steady window's own peak above its
    mean, the floor an overshoot has to clear to be real.
    """
    resp = [(t - t0, v) for t, v in samples if t >= t0]
    sign = 1.0 if target >= 0 else -1.0
    mag = abs(target)

    def first_cross(level):
        return next((t for t, v in resp if sign * v >= level * mag), None)

    t10, t90 = first_cross(0.1), first_cross(0.9)
    t_end = resp[-1][0] if resp else 0.0
    t_steady = t_end - steady_window_s
    peak = max((sign * v for t, v in resp if t < t_steady), default=0.0)
    steady = [sign * v for t, v in resp if t >= t_steady]
    mean = statistics.fmean(steady) if steady else float("nan")
    std = statistics.pstdev(steady) if len(steady) > 1 else float("nan")
    noise = max(steady) - mean if steady else float("nan")
    return {
        "target": target,
        "rise_ms": (t90 - t10) * 1000 if t10 is not None and t90 is not None else float("nan"),
        "t90_ms": t90 * 1000 if t90 is not None else float("nan"),
        "overshoot_pct": max(0.0, (peak - mag) / mag * 100),
        "ss_error_pct": (mean - mag) / mag * 100,
        "ripple_pct": std / mag * 100,
        "noise_pct": noise / mag * 100,
    }
