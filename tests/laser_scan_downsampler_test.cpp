#include "laser_scan_downsampler.h"

#include <cmath>
#include <cstdlib>
#include <cstdint>

namespace
{
void Require(bool condition)
{
    if (!condition)
    {
        std::abort();
    }
}

bool Near(float lhs, float rhs, float tolerance = 0.0001f)
{
    return std::abs(lhs - rhs) <= tolerance;
}

void TestClosestObstacleWinsAndCoverageIsPreserved()
{
    uint16_t source[360] = {};
    float output[320] = {};

    // Source degrees 0 and 1 both map to output bin 0 at 320/360.
    source[0] = 1000;
    source[1] = 450;
    source[180] = 2500;
    source[359] = 700;

    Require(DownsampleLaserScanMin(source, 360, output, 320));
    Require(Near(output[0], 0.45f));
    Require(Near(output[160], 2.5f));
    Require(Near(output[319], 0.7f));
    Require(Near(output[1], 0.0f));
}

void TestIdentityMapping()
{
    const uint16_t source[4] = {100, 0, 300, 400};
    float output[4] = {};
    Require(DownsampleLaserScanMin(source, 4, output, 4));
    Require(Near(output[0], 0.1f));
    Require(Near(output[1], 0.0f));
    Require(Near(output[2], 0.3f));
    Require(Near(output[3], 0.4f));
}

void TestInvalidArgumentsFailClosed()
{
    const uint16_t source[2] = {};
    float output[3] = {};
    Require(!DownsampleLaserScanMin(nullptr, 2, output, 2));
    Require(!DownsampleLaserScanMin(source, 2, nullptr, 2));
    Require(!DownsampleLaserScanMin(source, 0, output, 2));
    Require(!DownsampleLaserScanMin(source, 2, output, 0));
    Require(!DownsampleLaserScanMin(source, 2, output, 3));
}
} // namespace

int main()
{
    TestClosestObstacleWinsAndCoverageIsPreserved();
    TestIdentityMapping();
    TestInvalidArgumentsFailClosed();
    return 0;
}
