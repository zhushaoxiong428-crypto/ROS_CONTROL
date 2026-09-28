#pragma once

#include <stdint.h>

// Debounces transient Agent health-ping failures without hiding a sustained
// outage. The caller remains responsible for executing all RMW operations in
// one task; this type only tracks consecutive results.
struct AgentPingHealth
{
    uint8_t consecutive_failures = 0;
    uint32_t first_failure_tick = 0;
    bool failure_window_active = false;

    bool KeepSession(bool ping_succeeded,
                     uint32_t now_tick,
                     uint32_t failure_window_ticks,
                     uint8_t minimum_failures)
    {
        if (ping_succeeded)
        {
            Reset();
            return true;
        }

        if (!failure_window_active)
        {
            failure_window_active = true;
            first_failure_tick = now_tick;
        }
        if (consecutive_failures < UINT8_MAX)
        {
            ++consecutive_failures;
        }

        // Unsigned subtraction intentionally handles the FreeRTOS tick wrap.
        const uint32_t elapsed_ticks = now_tick - first_failure_tick;
        return consecutive_failures < minimum_failures ||
               elapsed_ticks < failure_window_ticks;
    }

    uint32_t FailureAgeTicks(uint32_t now_tick) const
    {
        return failure_window_active ? now_tick - first_failure_tick : 0;
    }

    void Reset()
    {
        consecutive_failures = 0;
        first_failure_tick = 0;
        failure_window_active = false;
    }
};
