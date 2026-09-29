// 实时性诊断：控制周期抖动、各任务栈余量与 CPU 占用。
//
// 统计输出放在这个最低优先级任务里，而不是在 motion_task 内打印：
// 串口输出一行需要数毫秒，在控制任务里打印会反过来制造周期抖动。

#include "system_globals.h"

#include <algorithm>
#include <cstring>

#include "control/cycle_timing_stats.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#if defined(CONFIG_LEAP_RT_STATS)

static const char *TAG = "RT_STATS";
static constexpr size_t kMaxTasks = 32;

struct TaskSample
{
  TaskHandle_t handle;
  configRUN_TIME_COUNTER_TYPE run_time;
};

static TaskStatus_t s_status[kMaxTasks];
static TaskSample s_prev[kMaxTasks];
static size_t s_prev_count = 0;

static configRUN_TIME_COUNTER_TYPE previous_run_time(TaskHandle_t handle, bool *found)
{
  for (size_t i = 0; i < s_prev_count; ++i)
  {
    if (s_prev[i].handle == handle)
    {
      *found = true;
      return s_prev[i].run_time;
    }
  }
  *found = false;
  return 0;
}

static void log_control_timing()
{
  CycleTimingSnapshot t = {};
  if (q_control_timing == nullptr || xQueuePeek(q_control_timing, &t, 0) != pdTRUE || t.count == 0)
  {
    ESP_LOGI(TAG, "control: no samples yet");
    return;
  }
  ESP_LOGI(TAG,
           "control(20ms) n=%lu period avg=%.3f min=%.3f max=%.3f std=%.3f ms |jitter|max=%.3f ms "
           "busy avg=%.3f max=%.3f ms overruns(>22ms)=%lu",
           static_cast<unsigned long>(t.count),
           t.period_mean_us / 1000.0f,
           t.period_min_us / 1000.0f,
           t.period_max_us / 1000.0f,
           t.period_std_us / 1000.0f,
           t.max_abs_jitter_us / 1000.0f,
           t.busy_mean_us / 1000.0f,
           t.busy_max_us / 1000.0f,
           static_cast<unsigned long>(t.overruns));
}

static void log_tasks(int64_t window_us)
{
  configRUN_TIME_COUNTER_TYPE total_run_time = 0;
  const UBaseType_t n = uxTaskGetSystemState(s_status, kMaxTasks, &total_run_time);
  if (n == 0)
  {
    ESP_LOGW(TAG, "more than %u tasks; increase kMaxTasks", static_cast<unsigned>(kMaxTasks));
    return;
  }

  // 本窗口内各任务占用的 CPU 时间（运行时计数器单位为 us，来自 esp_timer）。
  uint32_t delta_us[kMaxTasks] = {};
  bool have_delta = window_us > 0;
  float idle_pct[2] = {-1.0f, -1.0f};
  for (UBaseType_t i = 0; i < n; ++i)
  {
    bool found = false;
    const configRUN_TIME_COUNTER_TYPE prev = previous_run_time(s_status[i].xHandle, &found);
    delta_us[i] = found ? static_cast<uint32_t>(s_status[i].ulRunTimeCounter - prev) : 0;
    if (have_delta && std::strncmp(s_status[i].pcTaskName, "IDLE", 4) == 0)
    {
      const int core = s_status[i].pcTaskName[4] == '1' ? 1 : 0;
      idle_pct[core] = 100.0f * delta_us[i] / static_cast<float>(window_us);
    }
  }

  if (have_delta && idle_pct[0] >= 0.0f && idle_pct[1] >= 0.0f)
  {
    ESP_LOGI(TAG, "cpu load: core0=%.1f%% core1=%.1f%% (window %.1f s)",
             100.0f - idle_pct[0], 100.0f - idle_pct[1], window_us / 1e6f);
  }

  // 按 CPU 占用从高到低输出；百分比相对单个核心。
  UBaseType_t order[kMaxTasks];
  for (UBaseType_t i = 0; i < n; ++i)
    order[i] = i;
  std::sort(order, order + n, [&](UBaseType_t a, UBaseType_t b) { return delta_us[a] > delta_us[b]; });

  ESP_LOGI(TAG, "%-16s %4s %7s %11s", "task", "prio", "cpu%", "stack_free");
  for (UBaseType_t k = 0; k < n; ++k)
  {
    const TaskStatus_t &s = s_status[order[k]];
    ESP_LOGI(TAG, "%-16s %4u %6.2f%% %9u B",
             s.pcTaskName,
             static_cast<unsigned>(s.uxCurrentPriority),
             have_delta ? 100.0f * delta_us[order[k]] / static_cast<float>(window_us) : 0.0f,
             static_cast<unsigned>(s.usStackHighWaterMark));
  }

  s_prev_count = n;
  for (UBaseType_t i = 0; i < n; ++i)
  {
    s_prev[i] = {s_status[i].xHandle, s_status[i].ulRunTimeCounter};
  }
}

void rt_stats_task(void *)
{
  const TickType_t period = pdMS_TO_TICKS(CONFIG_LEAP_RT_STATS_INTERVAL_S * 1000);
  int64_t last_us = 0;
  TickType_t last_wake = xTaskGetTickCount();
  while (true)
  {
    vTaskDelayUntil(&last_wake, period);
    const int64_t now_us = esp_timer_get_time();
    log_control_timing();
    log_tasks(last_us > 0 ? now_us - last_us : 0);
    last_us = now_us;
  }
}

#endif // CONFIG_LEAP_RT_STATS
