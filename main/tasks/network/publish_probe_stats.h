#pragma once

#include <stdint.h>

// Pure in-memory timing counters. The caller measures and records only actual
// attempts; skipped publications must not be counted as zero-duration calls.
struct ProbeStats
{
    uint64_t count = 0;
    uint64_t fail = 0;
    uint64_t total_us = 0;
    uint64_t max_us = 0;
    uint32_t max_end_mcu_ms = 0;
    uint16_t max_debug_publish_seq = 0;
    uint64_t over_20ms = 0;
    uint64_t over_50ms = 0;

    void Observe(uint64_t duration_us, bool success,
                 uint32_t end_mcu_ms = 0, uint16_t debug_publish_seq = 0)
    {
        ++count;
        if (!success)
        {
            ++fail;
        }
        total_us += duration_us;
        if (duration_us > max_us)
        {
            max_us = duration_us;
            max_end_mcu_ms = end_mcu_ms;
            max_debug_publish_seq = debug_publish_seq;
        }
        if (duration_us > 20000)
        {
            ++over_20ms;
        }
        if (duration_us > 50000)
        {
            ++over_50ms;
        }
    }

    void Reset()
    {
        *this = ProbeStats{};
    }
};
