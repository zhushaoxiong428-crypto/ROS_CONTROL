#pragma once

// Runtime speed-loop options for experiments (POST /api/motion-tuning).
// Not persisted; motion_config.h holds the defaults used after a reboot.
struct MotionTuningMsg {
    bool feedforward = true;
    float accel_limit_mps2 = 0.0f;  // speed target ramp, 0 = step
};
