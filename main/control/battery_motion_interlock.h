#pragma once

#include <stdint.h>

enum class BatteryInterlockReason : uint8_t
{
    kNone = 0,
    kNoSample,
    kAdcInvalid,
    kStale,
    kSampleGap,
    kUndervoltage,
    kOvervoltage,
    kRecovering,
    kNeedsZeroRearm,
};

enum class BatteryCommandDecision : uint8_t
{
    kExecuteMotion = 0,
    kRejectMotion,
    kStopOnly,
    kStopAndArm,
};

struct BatteryMotionInterlockConfig
{
    float low_trip_voltage_v = 6.0f;
    float high_trip_voltage_v = 8.6f;
    float low_recovery_voltage_v = 6.4f;
    float high_recovery_voltage_v = 8.5f;
    uint8_t recovery_samples = 3;
    uint32_t recovery_min_sample_interval_ms = 400;
    uint32_t recovery_min_duration_ms = 1000;
    uint32_t stale_timeout_ms = 1200;
};

struct BatteryMotionInterlockStatus
{
    BatteryInterlockReason reason = BatteryInterlockReason::kNoSample;
    BatteryInterlockReason last_trip_reason = BatteryInterlockReason::kNoSample;
    bool has_sample = false;
    bool adc_valid = false;
    bool sample_fresh = false;
    bool voltage_ready = false;
    bool armed = false;
    bool motion_allowed = false;
    float voltage_v = 0.0f;
    uint8_t recovery_sample_count = 0;
    uint32_t sample_seq = 0;
    uint32_t sample_age_ms = 0;
    uint32_t trip_count = 0;
};

// Pure state machine. BatteryMsg::valid remains an ADC-acquisition result;
// motion permission is deliberately tracked here as a separate concern.
class BatteryMotionInterlock
{
public:
    explicit BatteryMotionInterlock(
        const BatteryMotionInterlockConfig &config = BatteryMotionInterlockConfig{});

    // Repeated peeks of the one-slot battery queue are ignored by sequence ID.
    // Returns true only when a new independent sample was consumed.
    bool ObserveSample(
        uint32_t sample_seq,
        uint32_t sample_time_ms,
        bool adc_valid,
        float voltage_v);

    void Poll(uint32_t now_ms);
    BatteryCommandDecision EvaluateCommand(bool explicit_all_stop);

    bool MotionAllowed() const;
    BatteryMotionInterlockStatus Status(uint32_t now_ms) const;
    BatteryInterlockReason Reason() const { return reason_; }
    BatteryInterlockReason LastTripReason() const { return last_trip_reason_; }
    static const char *ReasonName(BatteryInterlockReason reason);

private:
    void Trip(BatteryInterlockReason reason);
    void ResetRecovery(BatteryInterlockReason reason);

    BatteryMotionInterlockConfig config_;
    BatteryInterlockReason reason_ = BatteryInterlockReason::kNoSample;
    BatteryInterlockReason last_trip_reason_ = BatteryInterlockReason::kNoSample;
    bool has_sample_ = false;
    bool adc_valid_ = false;
    bool stale_ = false;
    bool voltage_ready_ = false;
    bool armed_ = false;
    float voltage_v_ = 0.0f;
    uint8_t recovery_sample_count_ = 0;
    uint32_t recovery_start_time_ms_ = 0;
    uint32_t last_sample_seq_ = 0;
    uint32_t last_sample_time_ms_ = 0;
    uint32_t trip_count_ = 0;
};
