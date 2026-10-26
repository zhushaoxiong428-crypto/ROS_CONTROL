#pragma once

#include <stddef.h>
#include <stdint.h>

// Per-task CPU accounting between two FreeRTOS uxTaskGetSystemState()
// snapshots. Run-time counters are 32-bit microseconds and wrap after ~71
// minutes, so deltas use unsigned subtraction (correct across one wrap).

struct TaskRuntimeSample {
  uint32_t task_number;  // TaskStatus_t::xTaskNumber, unique per task
  uint32_t runtime;      // TaskStatus_t::ulRunTimeCounter
};

// Run time the task used since the previous snapshot. A task that was not in
// the previous snapshot (created since) is counted from its creation, i.e.
// its whole counter. Sets *found accordingly when found is non-null.
inline uint32_t TaskRuntimeDelta(const TaskRuntimeSample* previous, size_t previous_count,
                                 uint32_t task_number, uint32_t runtime_now, bool* found) {
  for (size_t i = 0; i < previous_count; ++i) {
    if (previous[i].task_number == task_number) {
      if (found != nullptr) *found = true;
      return runtime_now - previous[i].runtime;
    }
  }
  if (found != nullptr) *found = false;
  return runtime_now;
}

// Share of one core in percent: a task that ran the whole window on one core
// reports 100. Returns 0 for an empty window.
inline float CorePercent(uint32_t task_delta, uint32_t window) {
  return window == 0 ? 0.0f : 100.0f * static_cast<float>(task_delta) / static_cast<float>(window);
}
