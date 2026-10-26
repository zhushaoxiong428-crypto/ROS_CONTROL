#include "runtime_stats.h"

#include <cassert>
#include <cmath>
#include <cstdint>

static void ExpectNear(float actual, float expected) {
    assert(std::fabs(actual - expected) < 1e-3f);
}

int main() {
    const TaskRuntimeSample previous[] = {{1, 1000}, {7, 50000}, {9, UINT32_MAX - 99}};
    bool found = false;

    // Known task: plain difference.
    assert(TaskRuntimeDelta(previous, 3, 7, 52500, &found) == 2500);
    assert(found);

    // The 32-bit microsecond counter wrapped since the last snapshot.
    assert(TaskRuntimeDelta(previous, 3, 9, 400, &found) == 500);
    assert(found);

    // Task created since: counted from zero.
    assert(TaskRuntimeDelta(previous, 3, 12, 3000, &found) == 3000);
    assert(!found);
    assert(TaskRuntimeDelta(nullptr, 0, 1, 42, nullptr) == 42);

    // Percent of one core over the window.
    ExpectNear(CorePercent(2500, 10000), 25.0f);
    ExpectNear(CorePercent(10000, 10000), 100.0f);
    ExpectNear(CorePercent(0, 10000), 0.0f);
    ExpectNear(CorePercent(5, 0), 0.0f);
}
