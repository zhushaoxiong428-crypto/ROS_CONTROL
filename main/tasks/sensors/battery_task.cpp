#include "system_globals.h"
#include "board.h"
#include "msg/battery_msg.h"

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "BATTERY_TASK";
static constexpr TickType_t kBatterySampleTicks = pdMS_TO_TICKS(500);

void battery_task(void *p) {
    (void)p;

    if (battery_adc.Init() != ESP_OK) {
        ESP_LOGE(TAG, "battery ADC init failed");
        vTaskDelete(NULL);
        return;
    }

    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t sample_seq = 0;
    while (1) {
        BatteryAdcReading reading = battery_adc.Read();
        BatteryMsg msg = {};
        msg.adc_voltage_v = reading.adc_voltage_v;
        msg.voltage_v = reading.battery_voltage_v;
        msg.percentage = reading.percentage;
        msg.raw = reading.raw;
        msg.valid = reading.valid;
        msg.sample_seq = ++sample_seq;
        msg.sample_time_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
        if (q_battery_state != nullptr) {
            xQueueOverwrite(q_battery_state, &msg);
        }
        vTaskDelayUntil(&last_wake_time, kBatterySampleTicks);
    }
}
