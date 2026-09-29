#include "cycle_timing_stats.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace
{
int g_failures = 0;

void Expect(bool condition, const char *message)
{
  if (!condition)
  {
    std::cerr << "FAIL: " << message << '\n';
    ++g_failures;
  }
}

void TestEmpty()
{
  CycleTimingStats stats(20000, 2000);
  Expect(stats.Snapshot().count == 0, "empty snapshot has no samples");
}

void TestExactPeriod()
{
  CycleTimingStats stats(20000, 2000);
  for (int i = 0; i < 500; ++i)
    stats.AddCycle(20000, 300);
  const CycleTimingSnapshot s = stats.Snapshot();
  Expect(s.count == 500, "count");
  Expect(std::abs(s.period_mean_us - 20000.0f) < 0.01f, "mean equals nominal");
  Expect(s.period_std_us < 0.01f, "no jitter gives zero std");
  Expect(s.max_abs_jitter_us == 0, "zero max jitter");
  Expect(std::abs(s.busy_mean_us - 300.0f) < 0.01f, "busy mean");
  Expect(s.overruns == 0, "no overruns");
}

void TestJitterAndOverrun()
{
  CycleTimingStats stats(20000, 2000);
  // 典型的 tick 对齐：一次晚到 1 ms 被下一次提前补偿。
  stats.AddCycle(21000, 250);
  stats.AddCycle(19000, 900);
  stats.AddCycle(20000, 250);
  stats.AddCycle(23500, 250); // 超限
  const CycleTimingSnapshot s = stats.Snapshot();
  Expect(s.period_min_us == 19000, "min period");
  Expect(s.period_max_us == 23500, "max period");
  Expect(s.max_abs_jitter_us == 3500, "max |jitter|");
  Expect(s.overruns == 1, "one overrun beyond 22 ms");
  Expect(s.busy_max_us == 900, "busy max");
  // 偏差 +1000, -1000, 0, +3500：均值 875，方差 = (1e6+1e6+0+12.25e6)/4 - 875^2
  const float expected_std = std::sqrt(3562500.0f - 875.0f * 875.0f);
  Expect(std::abs(s.period_mean_us - 20875.0f) < 0.01f, "mean period");
  Expect(std::abs(s.period_std_us - expected_std) < 0.5f, "std deviation");
}

void TestReset()
{
  CycleTimingStats stats(20000, 2000);
  stats.AddCycle(25000, 5000);
  stats.Reset();
  stats.AddCycle(20000, 100);
  const CycleTimingSnapshot s = stats.Snapshot();
  Expect(s.count == 1 && s.overruns == 0 && s.busy_max_us == 100 && s.period_max_us == 20000,
         "reset starts a fresh window");
}
} // namespace

int main()
{
  TestEmpty();
  TestExactPeriod();
  TestJitterAndOverrun();
  TestReset();
  if (g_failures != 0)
  {
    std::cerr << g_failures << " failure(s)\n";
    return EXIT_FAILURE;
  }
  std::cout << "cycle_timing_stats_test passed\n";
  return EXIT_SUCCESS;
}
