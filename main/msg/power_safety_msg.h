#pragma once

#include <stdint.h>

struct PowerSafetyMsg
{
    uint8_t reason = 0;
    uint8_t last_trip_reason = 0;
    bool has_sample = false;
    bool adc_valid = false;
    bool sample_fresh = false;
    bool voltage_ready = false;
    bool armed = false;
    bool motion_allowed = false;
    float voltage_v = 0.0f;
    uint8_t recovery_sample_count = 0;
    uint32_t sample_seq = 0;
    uint32_t sample_age_ms = 0;
    uint32_t trip_count = 0;
};

