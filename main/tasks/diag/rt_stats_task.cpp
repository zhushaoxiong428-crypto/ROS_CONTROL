#include "system_globals.h"
#include "runtime_stats.h"

#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "RT_STATS";

// Every report window: core load (from each core's idle task), heap, and per
// task priority, core, CPU share and the least free stack seen since start.
// Logs only; started at the lowest priority so it never delays real work.
static constexpr uint32_t kReportPeriodMs = 10000;
// Offset from the motion loop's timing report so the two UART writes do not
// land in the same control cycle.
static constexpr uint32_t kStartDelayMs = 5000;
static constexpr size_t kMaxTasks = 32;

static TaskStatus_t s_status[kMaxTasks];
static TaskRuntimeSample s_previous[kMaxTasks];
static size_t s_previous_count = 0;
static uint32_t s_previous_total = 0;

static float idle_load(const char *idle_name, const UBaseType_t count, uint32_t window) {
    for (UBaseType_t i = 0; i < count; ++i) {
        if (strcmp(s_status[i].pcTaskName, idle_name) == 0) {
            const uint32_t delta = TaskRuntimeDelta(s_previous, s_previous_count, s_status[i].xTaskNumber,
                                                    s_status[i].ulRunTimeCounter, nullptr);
            return 100.0f - CorePercent(delta, window);
        }
    }
    return -1.0f;
}

static void report(UBaseType_t count, uint32_t window) {
    float cpu[kMaxTasks];
    size_t order[kMaxTasks];
    for (UBaseType_t i = 0; i < count; ++i) {
        const uint32_t delta = TaskRuntimeDelta(s_previous, s_previous_count, s_status[i].xTaskNumber,
                                                s_status[i].ulRunTimeCounter, nullptr);
        cpu[i] = CorePercent(delta, window);
        order[i] = i;
    }
    // Busiest first (insertion sort, at most kMaxTasks entries).
    for (size_t i = 1; i < count; ++i) {
        const size_t current = order[i];
        size_t j = i;
        while (j > 0 && cpu[order[j - 1]] < cpu[current]) {
            order[j] = order[j - 1];
            --j;
        }
        order[j] = current;
    }

    ESP_LOGI(TAG, "window %.1f s  core0 load %.1f%%  core1 load %.1f%%  heap free %u KiB (min %u KiB)  tasks %u",
             window / 1e6f, idle_load("IDLE0", count, window), idle_load("IDLE1", count, window),
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_8BIT) / 1024),
             static_cast<unsigned>(heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT) / 1024),
             static_cast<unsigned>(count));
    for (size_t k = 0; k < count; ++k) {
        const TaskStatus_t &task = s_status[order[k]];
        char core[12];
        if (task.xCoreID == tskNO_AFFINITY) {
            strcpy(core, "any");
        } else {
            snprintf(core, sizeof(core), "%d", static_cast<int>(task.xCoreID));
        }
        ESP_LOGI(TAG, "  %-16s prio %2u  core %-3s  cpu %6.2f%%  stack_free %5u B",
                 task.pcTaskName, static_cast<unsigned>(task.uxCurrentPriority), core, cpu[order[k]],
                 static_cast<unsigned>(task.usStackHighWaterMark));
    }
}

void rt_stats_task(void *p) {
    (void)p;
    vTaskDelay(pdMS_TO_TICKS(kStartDelayMs));
    while (1) {
        uint32_t total = 0;
        const UBaseType_t count = uxTaskGetSystemState(s_status, kMaxTasks, &total);
        const uint32_t window = total - s_previous_total;
        if (count == 0) {
            // uxTaskGetSystemState returns 0 when the array is too small.
            ESP_LOGW(TAG, "more than %u tasks, raise kMaxTasks", static_cast<unsigned>(kMaxTasks));
        }
        // UART0 carries the XRCE-DDS stream in micro-ROS UART mode.
        if (s_previous_count > 0 && count > 0 &&
            system_globals().runtime.wifi_comm_mode != WifiCommMode::kMicroRosUart) {
            report(count, window);
        }
        for (UBaseType_t i = 0; i < count; ++i) {
            s_previous[i] = {s_status[i].xTaskNumber, s_status[i].ulRunTimeCounter};
        }
        s_previous_count = count;
        s_previous_total = total;
        vTaskDelay(pdMS_TO_TICKS(kReportPeriodMs));
    }
}
