#pragma once

#include <algorithm>
#include <cmath>

// Open-loop wheel model measured on the ground: steady speed
// v = mms_per_pwm * (pwm - deadband_pwm) above the deadband, at a battery
// voltage of reference_v. PWM uses the 0-255 motor command scale.
struct WheelFeedforward {
  float deadband_pwm;
  float mms_per_pwm;
  float reference_v;
};

// Battery readings outside this window are not trusted for compensation
// (e.g. ~3.5 V with only USB power, when the motors cannot run anyway).
static constexpr float kFeedforwardMinBatteryV = 6.0f;
static constexpr float kFeedforwardMaxBatteryV = 9.0f;

// PWM that holds target_mms in steady state. A DC motor's speed follows the
// applied voltage (duty x battery), so duty scales with reference_v /
// battery_v; pass battery_v <= 0 to skip that. Returns 0 for a zero target.
inline float FeedforwardPwm(float target_mms, const WheelFeedforward& ff, float battery_v, float max_pwm) {
  if (target_mms == 0.0f) {
    return 0.0f;
  }
  float pwm = ff.deadband_pwm + std::fabs(target_mms) / ff.mms_per_pwm;
  if (battery_v >= kFeedforwardMinBatteryV && battery_v <= kFeedforwardMaxBatteryV) {
    pwm *= ff.reference_v / battery_v;
  }
  return std::copysign(std::min(pwm, max_pwm), target_mms);
}
