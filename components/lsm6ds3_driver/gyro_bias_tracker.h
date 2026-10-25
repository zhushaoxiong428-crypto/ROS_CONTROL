#ifndef GYRO_BIAS_TRACKER_H_
#define GYRO_BIAS_TRACKER_H_

#include <algorithm>
#include <cmath>

// Keeps one gyro axis' bias current while the robot stands still.
//
// The bias measured at boot drifts with temperature (~0.4 deg/s within an
// hour on this board, i.e. ~23 deg/min of heading drift). Whenever the
// wheel encoders report the robot has been still for settle_s, the raw rate
// is pulled into the bias with a first-order filter, and the corrected rate
// is reported as 0 so the heading does not creep. A rate above max_still_dps
// while the wheels are still (the robot turned by hand) is neither learned
// nor suppressed.
class GyroBiasTracker {
 public:
  GyroBiasTracker(float initial_bias_dps, float time_constant_s, float settle_s, float max_still_dps)
      : bias_dps_(initial_bias_dps),
        time_constant_s_(time_constant_s),
        settle_s_(settle_s),
        max_still_dps_(max_still_dps) {}

  void Reset(float bias_dps) {
    bias_dps_ = bias_dps;
    still_s_ = 0.0f;
  }

  // raw_dps: sensor reading; wheels_still: encoders report no motion.
  // Returns the bias-corrected rate in deg/s.
  float Update(float raw_dps, bool wheels_still, float dt_s) {
    const float rate = raw_dps - bias_dps_;
    if (!wheels_still) {
      still_s_ = 0.0f;
      return rate;
    }
    if (std::fabs(rate) > max_still_dps_) {
      return rate;
    }
    still_s_ += dt_s;
    if (still_s_ >= settle_s_) {
      bias_dps_ += rate * std::min(dt_s / time_constant_s_, 1.0f);
    }
    return 0.0f;
  }

  float bias_dps() const { return bias_dps_; }

 private:
  float bias_dps_;
  float time_constant_s_;
  float settle_s_;
  float max_still_dps_;
  float still_s_ = 0.0f;
};

#endif  // GYRO_BIAS_TRACKER_H_
