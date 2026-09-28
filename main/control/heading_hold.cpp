#include "heading_hold.h"

#include <algorithm>
#include <cmath>

float HeadingHold::WrapAngle(float angle_rad)
{
  constexpr float kPi = 3.14159265358979f;
  return std::remainder(angle_rad, 2.0f * kPi);
}

float HeadingHold::Update(bool engaged, float yaw_rad, float dt)
{
  if (!engaged || !std::isfinite(yaw_rad) || !std::isfinite(dt) || dt <= 0.0f)
  {
    Reset();
    return 0.0f;
  }
  if (!active_)
  {
    active_ = true;
    reference_rad_ = yaw_rad;
    integral_ = 0.0f;
  }

  const float error = WrapAngle(reference_rad_ - yaw_rad);
  const float candidate_integral =
      std::clamp(integral_ + error * dt, -config_.max_integral, config_.max_integral);
  const float unsaturated = config_.kp * error + config_.ki * candidate_integral;
  const float correction = std::clamp(unsaturated, -config_.max_wz, config_.max_wz);

  // 条件积分抗饱和：未饱和时，或误差方向能把输出拉回线性区时才更新积分。
  const bool saturated_high = unsaturated > config_.max_wz;
  const bool saturated_low = unsaturated < -config_.max_wz;
  if ((!saturated_high && !saturated_low) ||
      (saturated_high && error < 0.0f) ||
      (saturated_low && error > 0.0f))
  {
    integral_ = candidate_integral;
  }

  last_error_rad_ = error;
  last_correction_ = correction;
  return correction;
}
