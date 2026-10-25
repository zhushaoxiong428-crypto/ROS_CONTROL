#pragma once

#include <stdint.h>

namespace leap_config {
namespace motion {

static constexpr int kWheelCount = 2;
static constexpr float kWheelDiameterMm = 65.0f;
static constexpr float kTrackWidthMm = 131.7f;
static constexpr float kHalfTrackMm = kTrackWidthMm / 2.0f;
static constexpr float kRpmToMmS = (3.14159265358979323846 * kWheelDiameterMm) / 60.0f;
static constexpr float kMmSToRpm = 1.0f / kRpmToMmS;
static constexpr float kPulsesPerWheelRev = 4680.0f;
static constexpr int kEncoderSigns[kWheelCount] = {-1, 1};
// Tuned on the ground (data/pid_grid_20261023_ground): kp=1.2/ki=4 keeps
// steady error < 0.5 % with 4 % mean overshoot at 0.05-0.1 m/s.
static constexpr float kDefaultSpeedKp = 1.2f;
static constexpr float kDefaultSpeedKi = 4.0f;
static constexpr float kDefaultSpeedKd = 0.0f;
static constexpr float kDefaultPositionKp = 2.1f;
static constexpr float kDefaultPositionKi = 0.0f;
static constexpr float kDefaultPositionKd = 0.5f;
static constexpr float kDefaultYawKp = 2.5f;
static constexpr float kDefaultYawKi = 0.0f;
static constexpr float kDefaultYawKd = 0.5f;
// Motor command scale: 255 = full duty. Commands are floats, quantized by the
// 10-bit motor PWM (see control/pwm_output.h).
static constexpr float kMaxPwm = 255.0f;
static constexpr float kMotorPwmDeadband = 150.0f;
static constexpr float kVelocityFilterAlpha = 0.35f;
static constexpr float kMaxPositionVelocityMmS = 250.0f;
static constexpr float kMaxPositionYawRadS = 3.0f;
static constexpr int kRelativeTurnSettleMaxCycles = 5;
static constexpr float kRelativeTurnSettleVxMms = 10.0f;
static constexpr float kDefaultMaxAccelerationRpmS = 200.0f;
static constexpr uint32_t kControlLoopPeriodMs = 20;
// Streamed velocity commands older than this stop the robot (same default as
// ros2_control diff_drive_controller cmd_vel_timeout).
static constexpr uint32_t kCommandTimeoutMs = 500;
// Control loop timing report: periods beyond nominal + margin count as
// overruns; a summary is logged every kTimingReportCycles cycles (10 s).
static constexpr int64_t kControlLoopOverrunMarginUs = 2000;
static constexpr uint32_t kTimingReportCycles = 500;

}  // namespace motion
}  // namespace leap_config
