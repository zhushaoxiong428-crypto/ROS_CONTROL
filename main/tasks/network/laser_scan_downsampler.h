#pragma once

#include <cstddef>
#include <cstdint>

// Merge fixed-angle source bins into fewer output bins while preserving the
// closest valid obstacle in every bin. Zero remains the "no valid return"
// marker used by the lidar task and the existing LaserScan publisher.
inline bool DownsampleLaserScanMin(
    const uint16_t *source_mm,
    size_t source_count,
    float *output_m,
    size_t output_count)
{
    if (source_mm == nullptr || output_m == nullptr ||
        source_count == 0 || output_count == 0 || output_count > source_count)
    {
        return false;
    }

    for (size_t output_index = 0; output_index < output_count; ++output_index)
    {
        output_m[output_index] = 0.0f;
    }

    for (size_t source_index = 0; source_index < source_count; ++source_index)
    {
        const uint16_t distance_mm = source_mm[source_index];
        if (distance_mm == 0)
        {
            continue;
        }

        const size_t output_index = source_index * output_count / source_count;
        const float distance_m = static_cast<float>(distance_mm) / 1000.0f;
        if (output_m[output_index] == 0.0f || distance_m < output_m[output_index])
        {
            output_m[output_index] = distance_m;
        }
    }
    return true;
}
