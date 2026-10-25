#include "wheel_feedforward.h"

#include <cassert>
#include <cmath>

static void ExpectNear(float actual, float expected) {
    assert(std::fabs(actual - expected) < 1e-3f);
}

int main() {
    const WheelFeedforward ff = {142.0f, 5.6f, 8.0f};

    // At the reference voltage: deadband plus speed over slope.
    ExpectNear(FeedforwardPwm(56.0f, ff, 8.0f, 255.0f), 152.0f);
    ExpectNear(FeedforwardPwm(-56.0f, ff, 8.0f, 255.0f), -152.0f);

    // Zero target: no push.
    ExpectNear(FeedforwardPwm(0.0f, ff, 8.0f, 255.0f), 0.0f);

    // A tiny target still gets the deadband, so the wheel is ready to move.
    ExpectNear(FeedforwardPwm(0.5f, ff, 8.0f, 255.0f), 142.0f + 0.5f / 5.6f);

    // Lower battery needs proportionally more duty, higher needs less.
    ExpectNear(FeedforwardPwm(56.0f, ff, 7.0f, 255.0f), 152.0f * 8.0f / 7.0f);
    ExpectNear(FeedforwardPwm(56.0f, ff, 8.4f, 255.0f), 152.0f * 8.0f / 8.4f);

    // Implausible readings (USB power only, unset) skip compensation.
    ExpectNear(FeedforwardPwm(56.0f, ff, 3.45f, 255.0f), 152.0f);
    ExpectNear(FeedforwardPwm(56.0f, ff, 0.0f, 255.0f), 152.0f);
    ExpectNear(FeedforwardPwm(56.0f, ff, 12.0f, 255.0f), 152.0f);

    // Saturates at full duty, keeping the sign.
    ExpectNear(FeedforwardPwm(900.0f, ff, 8.0f, 255.0f), 255.0f);
    ExpectNear(FeedforwardPwm(-900.0f, ff, 6.5f, 255.0f), -255.0f);
}
