#pragma once

#include <algorithm>
#include <cmath>

// Motor commands use an 8-bit-equivalent scale (kMaxPwm = 255 is full duty)
// so the speed PID gains, the deadband and the measured motor curve keep the
// units they were tuned and measured in. Commands stay floats here and are
// only quantized by the motor driver's 10-bit LEDC timer, four times finer
// than rounding to 255 levels.

// Adds the deadband offset in the direction of the PID output, so the motor
// gets at least enough duty to turn. Returns 0 when the target or the PID
// output is (near) zero, so a stopped wheel is not pushed.
inline float ApplyPwmDeadband(float target, float pid_output, float deadband, float max_pwm) {
  if (std::fabs(target) <= 0.1f || std::fabs(pid_output) <= 0.1f) {
    return 0.0f;
  }
  const float out = pid_output + (pid_output > 0.0f ? deadband : -deadband);
  return std::clamp(out, -max_pwm, max_pwm);
}
