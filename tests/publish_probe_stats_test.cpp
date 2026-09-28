#include "publish_probe_stats.h"

#include <cstdlib>

namespace
{
void Require(bool condition)
{
    if (!condition)
    {
        std::abort();
    }
}

void TestEmptyAndReset()
{
    ProbeStats stats;
    Require(stats.count == 0);
    Require(stats.fail == 0);
    Require(stats.total_us == 0);
    Require(stats.max_us == 0);
    Require(stats.max_end_mcu_ms == 0);
    Require(stats.max_debug_publish_seq == 0);
    Require(stats.over_20ms == 0);
    Require(stats.over_50ms == 0);

    stats.Observe(50001, false);
    stats.Reset();
    Require(stats.count == 0);
    Require(stats.fail == 0);
    Require(stats.total_us == 0);
    Require(stats.max_us == 0);
    Require(stats.max_end_mcu_ms == 0);
    Require(stats.max_debug_publish_seq == 0);
    Require(stats.over_20ms == 0);
    Require(stats.over_50ms == 0);
}

void TestDurationsAndThresholdBoundaries()
{
    ProbeStats stats;
    stats.Observe(0, true);
    stats.Observe(20000, true);
    stats.Observe(20001, false, 1234, 9);
    stats.Observe(50000, true);
    stats.Observe(50001, false, 4321, 17);

    Require(stats.count == 5);
    Require(stats.fail == 2);
    Require(stats.total_us == 140002);
    Require(stats.max_us == 50001);
    Require(stats.max_end_mcu_ms == 4321);
    Require(stats.max_debug_publish_seq == 17);
    Require(stats.over_20ms == 3);
    Require(stats.over_50ms == 1);
}
} // namespace

int main()
{
    TestEmptyAndReset();
    TestDurationsAndThresholdBoundaries();
}
