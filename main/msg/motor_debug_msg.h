#pragma once

#include <cstdint>

struct MotorDebugMsg
{
    float left_target_rpm = 0.0f;
    float left_actual_rpm = 0.0f;
    float left_raw_pwm = 0.0f;
    float left_final_pwm = 0.0f;

    float right_target_rpm = 0.0f;
    float right_actual_rpm = 0.0f;
    float right_raw_pwm = 0.0f;
    float right_final_pwm = 0.0f;

    float sync_error_rpm = 0.0f;
    float sync_correction_pwm = 0.0f;
    bool sync_active = false;
    bool startup_active = false;
    bool startup_fault = false;

    // Captured with this control snapshot, before the one-slot queue can overwrite it.
    uint16_t control_seq = 0;
    uint32_t control_time_ms = 0;
};
