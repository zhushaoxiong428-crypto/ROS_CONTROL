#include "leap_bridge/rate_stats.h"

#include <cassert>
#include <cmath>

using leap_bridge::RateStats;

namespace {

constexpr int64_t kMs = 1000000;

bool Near(double a, double b) { return std::fabs(a - b) < 1e-6; }

}  // namespace

int main() {
    RateStats stats;

    // One arrival is not an interval yet.
    stats.Add(0);
    assert(stats.intervals() == 0);
    assert(stats.rate_hz() == 0.0);

    stats.Add(20 * kMs);
    stats.Add(40 * kMs);
    stats.Add(61 * kMs);
    assert(stats.intervals() == 3);
    assert(Near(stats.min_ms(), 20.0));
    assert(Near(stats.max_ms(), 21.0));
    assert(Near(stats.mean_ms(), 61.0 / 3));
    assert(Near(stats.rate_hz(), 3 / 0.061));

    // The interval that spans the window boundary belongs to the new window.
    stats.ResetWindow();
    assert(stats.intervals() == 0);
    stats.Add(101 * kMs);
    assert(stats.intervals() == 1);
    assert(Near(stats.min_ms(), 40.0));
    assert(Near(stats.max_ms(), 40.0));

    return 0;
}
