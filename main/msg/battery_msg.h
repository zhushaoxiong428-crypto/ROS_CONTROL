#pragma once

#include <stdint.h>

struct BatteryMsg {
    float adc_voltage_v = 0.0f;
    float voltage_v = 0.0f;
    uint8_t percentage = 0;
    int raw = 0;
    bool valid = false;
    // Incremented for every ADC acquisition attempt, including invalid ones.
    // Consumers must use this instead of counting repeated queue peeks.
    uint32_t sample_seq = 0;
    uint32_t sample_time_ms = 0;
};
