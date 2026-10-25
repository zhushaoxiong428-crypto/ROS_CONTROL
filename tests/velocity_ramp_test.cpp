#include "velocity_ramp.h"

#include <cassert>
#include <cmath>

static void ExpectNear(float actual, float expected) {
    assert(std::fabs(actual - expected) < 1e-5f);
}

int main() {
    // Rising and falling targets move by one step per call.
    ExpectNear(RampToward(0.0f, 10.0f, 3.0f), 3.0f);
    ExpectNear(RampToward(10.0f, 0.0f, 3.0f), 7.0f);
    ExpectNear(RampToward(-2.0f, -10.0f, 3.0f), -5.0f);

    // The last step lands exactly on the target, never past it.
    ExpectNear(RampToward(8.5f, 10.0f, 3.0f), 10.0f);
    ExpectNear(RampToward(-8.5f, -10.0f, 3.0f), -10.0f);
    ExpectNear(RampToward(10.0f, 10.0f, 3.0f), 10.0f);

    // Reversing crosses zero at the same rate.
    float v = 4.0f;
    for (int i = 0; i < 3; ++i) {
        v = RampToward(v, -4.0f, 3.0f);
    }
    ExpectNear(v, -4.0f);

    // 0 -> 0.1 m/s at 1 m/s^2 with a 20 ms loop takes five cycles.
    v = 0.0f;
    int cycles = 0;
    while (v < 100.0f) {
        v = RampToward(v, 100.0f, 1000.0f * 0.02f);
        ++cycles;
    }
    assert(cycles == 5);
}
