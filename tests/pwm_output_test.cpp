#include "pwm_output.h"

#include <cassert>
#include <cmath>

static void ExpectNear(float actual, float expected) {
    assert(std::fabs(actual - expected) < 1e-4f);
}

int main() {
    // The offset follows the sign of the PID output, not of the target.
    ExpectNear(ApplyPwmDeadband(30.0f, 12.0f, 150.0f, 255.0f), 162.0f);
    ExpectNear(ApplyPwmDeadband(-30.0f, -12.0f, 150.0f, 255.0f), -162.0f);
    ExpectNear(ApplyPwmDeadband(30.0f, -5.0f, 150.0f, 255.0f), -155.0f);

    // Fractions are kept; the old int16_t cast truncated 150.6 to 150.
    ExpectNear(ApplyPwmDeadband(10.0f, 0.6f, 150.0f, 255.0f), 150.6f);

    // Saturates at full duty in both directions.
    ExpectNear(ApplyPwmDeadband(100.0f, 200.0f, 150.0f, 255.0f), 255.0f);
    ExpectNear(ApplyPwmDeadband(-100.0f, -200.0f, 150.0f, 255.0f), -255.0f);

    // No push for a zero target or a zero PID output.
    ExpectNear(ApplyPwmDeadband(0.05f, 50.0f, 150.0f, 255.0f), 0.0f);
    ExpectNear(ApplyPwmDeadband(30.0f, 0.05f, 150.0f, 255.0f), 0.0f);
}
