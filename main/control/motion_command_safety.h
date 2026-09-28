#pragma once

#include <cmath>

#include "msg/motion_msg.h"

inline bool IsExplicitAllStopCommand(const MotionMsg &command)
{
    constexpr float kStopEpsilon = 0.001f;
    return command.control_mode == 0 &&
           std::isfinite(command.target_vx) &&
           std::isfinite(command.target_vy) &&
           std::isfinite(command.target_wz) &&
           std::abs(command.target_vx) < kStopEpsilon &&
           std::abs(command.target_vy) < kStopEpsilon &&
           std::abs(command.target_wz) < kStopEpsilon;
}

