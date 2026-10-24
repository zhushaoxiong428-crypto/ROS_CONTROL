#pragma once

#include <stdint.h>

enum MotionCmdSource : uint8_t {
    MOTION_SRC_NONE = 0,
    MOTION_SRC_SCRIPT = 1,
    MOTION_SRC_UDP = 2,
    MOTION_SRC_ESPNOW = 3,
    MOTION_SRC_UART = 4,
    MOTION_SRC_SYSTEM = 5,
    MOTION_SRC_MICROROS = 6,
    MOTION_SRC_GAMEPAD = 7,
};

// Open-loop PWM for motor characterization (CONFIG_LEAP_MOTOR_CHARACTERIZATION).
static constexpr uint8_t kMotionModeOpenLoopPwm = 7;

struct MotionMsg {
    uint8_t source = MOTION_SRC_NONE;
    uint8_t control_mode = 0; // 0速度, 1位置, 2相对运动, 3单轮速度, 6双轮速度, 7开环PWM

    // 当前状态
    float vx = 0.0f, vy = 0.0f, wz = 0.0f;
    float x = 0.0f, y = 0.0f, yaw = 0.0f;
    float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;

    // 目标指令 (速度)
    float target_vx = 0.0f, target_vy = 0.0f, target_wz = 0.0f;

    // 目标指令 (位置)
    float target_x = 0.0f, target_y = 0.0f, target_yaw = 0.0f;

    int motor_id = 0;          // 0:左轮, 1:右轮
    float target_motor_v = 0;  // 目标电机速度 (mm/s)

    // 开环 PWM (模式 7)，-255..255
    float target_pwm_left = 0.0f;
    float target_pwm_right = 0.0f;

    float vel_left = 0.0f;
    float vel_right = 0.0f;
};
