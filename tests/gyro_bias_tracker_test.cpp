#include "gyro_bias_tracker.h"

#include <cassert>
#include <cmath>

static void ExpectNear(float actual, float expected, float tolerance) {
    assert(std::fabs(actual - expected) <= tolerance);
}

int main() {
    const float dt = 0.02f;

    // Boot bias -0.45 deg/s, true bias has drifted to -0.07 (residual +0.38,
    // as measured on the robot). Standing still: output is 0 and the bias
    // converges with the 2 s time constant once the 0.5 s settle has passed.
    GyroBiasTracker tracker(-0.45f, 2.0f, 0.5f, 1.0f);
    for (int i = 0; i < 24; ++i) {  // 0.48 s: still settling
        ExpectNear(tracker.Update(-0.07f, true, dt), 0.0f, 1e-6f);
    }
    ExpectNear(tracker.bias_dps(), -0.45f, 1e-6f);
    for (int i = 0; i < 1000; ++i) {  // 20 s = 10 time constants
        ExpectNear(tracker.Update(-0.07f, true, dt), 0.0f, 1e-6f);
    }
    ExpectNear(tracker.bias_dps(), -0.07f, 1e-3f);

    // Moving: the corrected rate passes through and the bias is frozen.
    ExpectNear(tracker.Update(28.57f, false, dt), 28.64f, 1e-3f);
    ExpectNear(tracker.Update(0.5f, false, dt), 0.57f, 1e-3f);
    ExpectNear(tracker.bias_dps(), -0.07f, 1e-3f);

    // After moving, the settle time starts over before learning again.
    for (int i = 0; i < 20; ++i) {
        tracker.Update(0.3f, true, dt);
    }
    ExpectNear(tracker.bias_dps(), -0.07f, 1e-3f);

    // Turned by hand with the wheels still: reported, not learned.
    GyroBiasTracker hand(0.0f, 2.0f, 0.5f, 1.0f);
    for (int i = 0; i < 100; ++i) {
        ExpectNear(hand.Update(15.0f, true, dt), 15.0f, 1e-6f);
    }
    ExpectNear(hand.bias_dps(), 0.0f, 1e-6f);

    // Reset restores a bias and restarts the settle time.
    hand.Reset(0.2f);
    ExpectNear(hand.Update(0.25f, true, dt), 0.0f, 1e-6f);
    ExpectNear(hand.bias_dps(), 0.2f, 1e-6f);
}
