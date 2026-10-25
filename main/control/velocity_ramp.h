#pragma once

#include <algorithm>

// Moves current toward target by at most max_step (> 0). A step change in
// the speed target makes the speed PID add kp x (the whole change) in one
// cycle, which overshoots; ramping the target spreads that over the ramp.
inline float RampToward(float current, float target, float max_step) {
  return target > current ? std::min(current + max_step, target)
                          : std::max(current - max_step, target);
}
