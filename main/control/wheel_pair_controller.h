#ifndef WHEEL_PAIR_CONTROLLER_H_
#define WHEEL_PAIR_CONTROLLER_H_

struct WheelPairControllerConfig
{
  // The startup assist is only used for a deliberate straight-line command.
  // Keep startup assist out of the uncalibrated ultra-low-speed region. The
  // lowest existing steady-state calibration point is 14.69 RPM.
  float minimum_command_rpm = 14.0f;
  float target_match_tolerance_rpm = 0.5f;
  float started_threshold_rpm = 1.0f;
  float startup_timeout_s = 0.30f;
  float startup_pwm[2] = {145.0f, 145.0f};
  // After a verified forward start from rest, release the equal breakaway
  // output gradually instead of exposing the full left/right feedforward
  // difference in one 20 ms control step. This does not alter steady-state
  // calibration, rolling commands, turns, or reverse motion.
  float startup_release_duration_s = 0.30f;

  // Cross-coupled PI trim. Positive trim is added to the left wheel and
  // subtracted from the right wheel.
  float sync_kp = 0.25f;
  float sync_ki = 0.50f;
  float sync_error_deadband_rpm = 0.5f;
  float sync_min_target_rpm = 5.0f;
  float sync_max_correction_pwm = 8.0f;
  float max_pwm = 255.0f;
};

struct WheelPairControlOutput
{
  float left_pwm = 0.0f;
  float right_pwm = 0.0f;
  float sync_error_rpm = 0.0f;
  float sync_correction_pwm = 0.0f;
  bool sync_active = false;
  bool startup_active = false;
  bool startup_fault = false;
};

// Coordinates the two otherwise independent wheel-speed loops during a
// straight-line start. It has no ESP-IDF dependencies so its state transitions
// and anti-windup behaviour can be covered by host-side tests.
class WheelPairController
{
public:
  explicit WheelPairController(const WheelPairControllerConfig &config);

  WheelPairControlOutput Update(
      float left_target_rpm,
      float right_target_rpm,
      float left_final_target_rpm,
      float right_final_target_rpm,
      float left_actual_rpm,
      float right_actual_rpm,
      float left_base_pwm,
      float right_base_pwm,
      float dt,
      bool straight_control_enabled);

  void Reset();

private:
  bool IsEligible(
      float left_target_rpm,
      float right_target_rpm,
      float left_final_target_rpm,
      float right_final_target_rpm,
      float dt,
      bool straight_control_enabled) const;
  float ApplyDeadband(float error_rpm) const;
  float ApplyStartupFloor(float pwm, float target_rpm, int wheel_index) const;
  float ApplyStartupCeiling(float pwm, float target_rpm, int wheel_index) const;

  WheelPairControllerConfig config_;
  bool active_ = false;
  bool startup_active_ = false;
  bool reversal_wait_active_ = false;
  bool startup_release_armed_ = false;
  bool startup_release_active_ = false;
  bool fault_latched_ = false;
  bool wheel_started_[2] = {false, false};
  int command_direction_ = 0;
  float startup_elapsed_s_ = 0.0f;
  float startup_release_elapsed_s_ = 0.0f;
  float sync_integral_ = 0.0f;
};

#endif // WHEEL_PAIR_CONTROLLER_H_
