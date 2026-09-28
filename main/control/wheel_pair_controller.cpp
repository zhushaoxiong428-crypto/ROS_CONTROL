#include "wheel_pair_controller.h"

#include <algorithm>
#include <cmath>

namespace
{
constexpr float kZeroEpsilon = 0.001f;
constexpr float kTimeComparisonEpsilonS = 0.000001f;

int Direction(float value)
{
  if (value > kZeroEpsilon)
  {
    return 1;
  }
  if (value < -kZeroEpsilon)
  {
    return -1;
  }
  return 0;
}
} // namespace

WheelPairController::WheelPairController(const WheelPairControllerConfig &config)
    : config_(config)
{
}

void WheelPairController::Reset()
{
  active_ = false;
  startup_active_ = false;
  reversal_wait_active_ = false;
  startup_release_armed_ = false;
  startup_release_active_ = false;
  fault_latched_ = false;
  wheel_started_[0] = false;
  wheel_started_[1] = false;
  command_direction_ = 0;
  startup_elapsed_s_ = 0.0f;
  startup_release_elapsed_s_ = 0.0f;
  sync_integral_ = 0.0f;
}

bool WheelPairController::IsEligible(
    float left_target_rpm,
    float right_target_rpm,
    float left_final_target_rpm,
    float right_final_target_rpm,
    float dt,
    bool straight_control_enabled) const
{
  if (!straight_control_enabled || !std::isfinite(dt) || dt <= 0.0f)
  {
    return false;
  }

  const int final_left_direction = Direction(left_final_target_rpm);
  const int final_right_direction = Direction(right_final_target_rpm);
  if (final_left_direction == 0 || final_left_direction != final_right_direction)
  {
    return false;
  }

  if (std::abs(left_final_target_rpm) < config_.minimum_command_rpm ||
      std::abs(right_final_target_rpm) < config_.minimum_command_rpm ||
      std::abs(left_final_target_rpm - right_final_target_rpm) >
          config_.target_match_tolerance_rpm)
  {
    return false;
  }

  // During a commanded reversal the ramp first brings the old direction to
  // zero. Do not re-arm startup assistance until both ramped targets have
  // crossed into the new direction.
  return Direction(left_target_rpm) == final_left_direction &&
         Direction(right_target_rpm) == final_right_direction;
}

float WheelPairController::ApplyDeadband(float error_rpm) const
{
  const float magnitude = std::abs(error_rpm);
  if (magnitude <= config_.sync_error_deadband_rpm)
  {
    return 0.0f;
  }
  return std::copysign(magnitude - config_.sync_error_deadband_rpm, error_rpm);
}

float WheelPairController::ApplyStartupFloor(
    float pwm, float target_rpm, int wheel_index) const
{
  const float floor = config_.startup_pwm[wheel_index];
  const float direction = target_rpm > 0.0f ? 1.0f : -1.0f;
  const float directed_pwm = direction * pwm;
  return direction * std::max(directed_pwm, floor);
}

float WheelPairController::ApplyStartupCeiling(
    float pwm, float target_rpm, int wheel_index) const
{
  const float ceiling = config_.startup_pwm[wheel_index];
  const float direction = target_rpm > 0.0f ? 1.0f : -1.0f;
  const float directed_pwm = direction * pwm;
  return direction * std::clamp(directed_pwm, 0.0f, ceiling);
}

WheelPairControlOutput WheelPairController::Update(
    float left_target_rpm,
    float right_target_rpm,
    float left_final_target_rpm,
    float right_final_target_rpm,
    float left_actual_rpm,
    float right_actual_rpm,
    float left_base_pwm,
    float right_base_pwm,
    float dt,
    bool straight_control_enabled)
{
  WheelPairControlOutput output = {};
  if (!std::isfinite(left_target_rpm) ||
      !std::isfinite(right_target_rpm) ||
      !std::isfinite(left_final_target_rpm) ||
      !std::isfinite(right_final_target_rpm) ||
      !std::isfinite(left_actual_rpm) ||
      !std::isfinite(right_actual_rpm) ||
      !std::isfinite(left_base_pwm) ||
      !std::isfinite(right_base_pwm) ||
      !std::isfinite(dt) || dt <= 0.0f)
  {
    Reset();
    output.startup_fault = true;
    return output;
  }
  output.left_pwm = left_base_pwm;
  output.right_pwm = right_base_pwm;

  if (!IsEligible(
          left_target_rpm,
          right_target_rpm,
          left_final_target_rpm,
          right_final_target_rpm,
          dt,
          straight_control_enabled))
  {
    Reset();
    return output;
  }

  const int command_direction = Direction(left_final_target_rpm);
  if (!active_ || command_direction_ != command_direction)
  {
    Reset();
    active_ = true;
    startup_active_ = true;
    command_direction_ = command_direction;
    reversal_wait_active_ =
        command_direction_ * left_actual_rpm < -config_.started_threshold_rpm ||
        command_direction_ * right_actual_rpm < -config_.started_threshold_rpm;
    startup_release_armed_ =
        command_direction_ > 0 &&
        !reversal_wait_active_ &&
        std::abs(left_actual_rpm) < config_.started_threshold_rpm &&
        std::abs(right_actual_rpm) < config_.started_threshold_rpm;
  }

  output.sync_active = true;
  output.sync_error_rpm =
      (left_target_rpm - left_actual_rpm) -
      (right_target_rpm - right_actual_rpm);

  if (fault_latched_)
  {
    output.left_pwm = 0.0f;
    output.right_pwm = 0.0f;
    output.startup_fault = true;
    return output;
  }

  // On a direction reversal, let both wheels coast until the old-direction
  // speed is almost zero. Applying the breakaway floor while the chassis is
  // still rolling the other way would create an abrupt active-braking pulse.
  if (reversal_wait_active_)
  {
    const bool left_near_zero =
        command_direction_ * left_actual_rpm >= -config_.started_threshold_rpm;
    const bool right_near_zero =
        command_direction_ * right_actual_rpm >= -config_.started_threshold_rpm;
    if (!(left_near_zero && right_near_zero))
    {
      output.left_pwm = 0.0f;
      output.right_pwm = 0.0f;
      output.startup_active = true;
      return output;
    }
    reversal_wait_active_ = false;
  }

  if (std::abs(left_target_rpm) >= config_.sync_min_target_rpm &&
      std::abs(right_target_rpm) >= config_.sync_min_target_rpm)
  {
    const float control_error = ApplyDeadband(output.sync_error_rpm);
    const float candidate_integral = startup_active_
                                         ? sync_integral_
                                         : sync_integral_ + control_error * dt;
    const float unsaturated_correction =
        config_.sync_kp * control_error +
        config_.sync_ki * candidate_integral;
    // c is added to the left base output and subtracted from the right. Limit
    // it to the intersection of both actuators' remaining PWM headroom.
    const float correction_lower_bound = std::max(
        {-config_.sync_max_correction_pwm,
         -config_.max_pwm - left_base_pwm,
         right_base_pwm - config_.max_pwm});
    const float correction_upper_bound = std::min(
        {config_.sync_max_correction_pwm,
         config_.max_pwm - left_base_pwm,
         right_base_pwm + config_.max_pwm});
    const float correction = std::clamp(
        unsaturated_correction,
        correction_lower_bound,
        correction_upper_bound);

    // Conditional integration: accept the new integral while unsaturated, or
    // when the current error would drive an already saturated trim back toward
    // the linear range.
    const bool correction_not_saturated =
        unsaturated_correction >= correction_lower_bound &&
        unsaturated_correction <= correction_upper_bound;
    const bool unwinding_high_saturation =
        unsaturated_correction > correction_upper_bound &&
        control_error < 0.0f;
    const bool unwinding_low_saturation =
        unsaturated_correction < correction_lower_bound &&
        control_error > 0.0f;
    if (!startup_active_ &&
        (correction_not_saturated ||
        unwinding_high_saturation ||
        unwinding_low_saturation))
    {
      sync_integral_ = candidate_integral;
    }

    output.sync_correction_pwm = correction;
    output.left_pwm += correction;
    output.right_pwm -= correction;
  }

  if (startup_active_)
  {
    if (!startup_release_active_)
    {
      startup_elapsed_s_ += dt;
      // A reversal may still have filtered speed in the old direction after
      // the target has crossed zero. Only motion in the commanded direction
      // counts. Do not latch this test before both wheels have started: if a
      // wheel falls back below the threshold first, it needs assistance again.
      wheel_started_[0] =
          command_direction_ * left_actual_rpm >= config_.started_threshold_rpm;
      wheel_started_[1] =
          command_direction_ * right_actual_rpm >= config_.started_threshold_rpm;

      const bool both_wheels_started = wheel_started_[0] && wheel_started_[1];
      if (!both_wheels_started && startup_elapsed_s_ >= config_.startup_timeout_s)
      {
        startup_active_ = false;
        startup_release_armed_ = false;
        fault_latched_ = true;
        output.left_pwm = 0.0f;
        output.right_pwm = 0.0f;
        output.sync_correction_pwm = 0.0f;
        output.startup_fault = true;
        return output;
      }

      if (both_wheels_started)
      {
        if (startup_release_armed_ && config_.startup_release_duration_s > 0.0f)
        {
          startup_release_active_ = true;
          startup_release_elapsed_s_ = 0.0f;
        }
        else
        {
          startup_active_ = false;
          startup_release_armed_ = false;
        }
      }

      if (startup_active_ && !startup_release_active_)
      {
        if (!wheel_started_[0])
        {
          output.left_pwm = ApplyStartupFloor(output.left_pwm, left_target_rpm, 0);
        }
        else if (!wheel_started_[1])
        {
          output.left_pwm = ApplyStartupCeiling(output.left_pwm, left_target_rpm, 0);
        }

        if (!wheel_started_[1])
        {
          output.right_pwm = ApplyStartupFloor(output.right_pwm, right_target_rpm, 1);
        }
        else if (!wheel_started_[0])
        {
          output.right_pwm = ApplyStartupCeiling(output.right_pwm, right_target_rpm, 1);
        }
      }
    }

    if (startup_release_active_)
    {
      // Repeated binary floating-point additions such as 15 * 0.02 can land
      // microscopically below 0.30. Treat that numerical residue as the exact
      // endpoint so a nominal 300 ms release does not last one extra control
      // cycle.
      const bool release_complete =
          startup_release_elapsed_s_ + kTimeComparisonEpsilonS >=
          config_.startup_release_duration_s;
      const float progress = release_complete
                                 ? 1.0f
                                 : std::clamp(
                                       startup_release_elapsed_s_ /
                                           config_.startup_release_duration_s,
                                       0.0f,
                                       1.0f);
      const float left_release_pwm =
          std::copysign(config_.startup_pwm[0], left_target_rpm);
      const float right_release_pwm =
          std::copysign(config_.startup_pwm[1], right_target_rpm);
      output.left_pwm =
          left_release_pwm + progress * (output.left_pwm - left_release_pwm);
      output.right_pwm =
          right_release_pwm + progress * (output.right_pwm - right_release_pwm);

      if (release_complete)
      {
        startup_release_active_ = false;
        startup_release_armed_ = false;
        startup_active_ = false;
      }
      else
      {
        startup_release_elapsed_s_ += dt;
      }
    }
  }

  output.startup_active = startup_active_;
  output.left_pwm = std::clamp(output.left_pwm, -config_.max_pwm, config_.max_pwm);
  output.right_pwm = std::clamp(output.right_pwm, -config_.max_pwm, config_.max_pwm);
  return output;
}
