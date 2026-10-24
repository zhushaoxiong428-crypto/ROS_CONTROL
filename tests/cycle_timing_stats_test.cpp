#include "cycle_timing_stats.h"

#include <cassert>
#include <cmath>

static void ExpectNear(float actual, float expected, float tolerance) {
    assert(std::fabs(actual - expected) <= tolerance);
}

int main() {
    CycleTimingStats stats(20000, 2000);
    assert(stats.Get().count == 0);

    // Periods 19, 20, 21 and 25 ms with busy 0.3-0.9 ms.
    stats.Add(19000, 300);
    stats.Add(20000, 500);
    stats.Add(21000, 900);
    stats.Add(25000, 400);
    CycleTimingStats::Summary s = stats.Get();
    assert(s.count == 4);
    ExpectNear(s.period_mean_us, 21250.0f, 0.5f);
    // Population std of {19, 20, 21, 25} ms is 2.2776 ms.
    ExpectNear(s.period_std_us, 2277.6f, 0.5f);
    assert(s.period_min_us == 19000);
    assert(s.period_max_us == 25000);
    assert(s.max_abs_jitter_us == 5000);
    ExpectNear(s.busy_mean_us, 525.0f, 0.5f);
    assert(s.busy_max_us == 900);
    assert(s.overruns == 1);  // only 25 ms exceeds 22 ms

    // A short period counts as jitter too.
    stats.Add(12000, 100);
    assert(stats.Get().max_abs_jitter_us == 8000);
    assert(stats.Get().period_min_us == 12000);

    stats.Reset();
    assert(stats.Get().count == 0);
    stats.Add(20000, 100);
    s = stats.Get();
    assert(s.period_min_us == 20000 && s.period_max_us == 20000);
    ExpectNear(s.period_std_us, 0.0f, 0.5f);
    assert(s.overruns == 0);

    // dt follows the measured period but is limited to [0.5, 2] x nominal.
    ExpectNear(ClampLoopDtS(20000, 20000), 0.020f, 1e-6f);
    ExpectNear(ClampLoopDtS(23500, 20000), 0.0235f, 1e-6f);
    ExpectNear(ClampLoopDtS(1000, 20000), 0.010f, 1e-6f);
    ExpectNear(ClampLoopDtS(500000, 20000), 0.040f, 1e-6f);
}
