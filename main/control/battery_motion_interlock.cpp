#include "battery_motion_interlock.h"

#include <cmath>

BatteryMotionInterlock::BatteryMotionInterlock(
    const BatteryMotionInterlockConfig &config)
    : config_(config)
{
    if (config_.recovery_samples == 0)
    {
        config_.recovery_samples = 1;
    }
}

bool BatteryMotionInterlock::ObserveSample(
    uint32_t sample_seq,
    uint32_t sample_time_ms,
    bool adc_valid,
    float voltage_v)
{
    if (has_sample_ && sample_seq == last_sample_seq_)
    {
        return false;
    }

    bool recovery_continuous = false;
    if (has_sample_)
    {
        const uint32_t sample_interval_ms = sample_time_ms - last_sample_time_ms_;
        const bool sequence_contiguous = sample_seq == last_sample_seq_ + 1U;
        if (!sequence_contiguous)
        {
            Trip(BatteryInterlockReason::kSampleGap);
        }
        else if (sample_interval_ms > config_.stale_timeout_ms)
        {
            Trip(BatteryInterlockReason::kStale);
        }
        else
        {
            recovery_continuous =
                sample_interval_ms >= config_.recovery_min_sample_interval_ms;
        }
    }

    has_sample_ = true;
    stale_ = false;
    last_sample_seq_ = sample_seq;
    last_sample_time_ms_ = sample_time_ms;
    // Expose effective validity to diagnostics as well as using it for the
    // permission decision. A driver-level success with NaN/Inf is not a
    // usable battery sample.
    adc_valid_ = adc_valid && std::isfinite(voltage_v);
    voltage_v_ = voltage_v;

    if (!adc_valid_)
    {
        Trip(BatteryInterlockReason::kAdcInvalid);
        return true;
    }
    if (voltage_v <= config_.low_trip_voltage_v)
    {
        Trip(BatteryInterlockReason::kUndervoltage);
        return true;
    }
    if (voltage_v > config_.high_trip_voltage_v)
    {
        Trip(BatteryInterlockReason::kOvervoltage);
        return true;
    }

    // Only an already-armed robot may use the wider trip band as hysteresis.
    // While waiting for the explicit zero re-arm, leaving the recovery band
    // must restart recovery rather than allowing arming at a marginal voltage.
    if (voltage_ready_ && armed_)
    {
        reason_ = BatteryInterlockReason::kNone;
        return true;
    }

    if (voltage_v < config_.low_recovery_voltage_v ||
        voltage_v > config_.high_recovery_voltage_v)
    {
        ResetRecovery(BatteryInterlockReason::kRecovering);
        return true;
    }

    if (voltage_ready_)
    {
        reason_ = BatteryInterlockReason::kNeedsZeroRearm;
        return true;
    }

    if (!recovery_continuous || recovery_sample_count_ == 0)
    {
        recovery_sample_count_ = 0;
        recovery_start_time_ms_ = sample_time_ms;
    }
    if (recovery_sample_count_ < UINT8_MAX)
    {
        ++recovery_sample_count_;
    }
    const uint32_t recovery_duration_ms = sample_time_ms - recovery_start_time_ms_;
    if (recovery_sample_count_ >= config_.recovery_samples &&
        recovery_duration_ms >= config_.recovery_min_duration_ms)
    {
        voltage_ready_ = true;
        armed_ = false;
        reason_ = BatteryInterlockReason::kNeedsZeroRearm;
    }
    else
    {
        reason_ = BatteryInterlockReason::kRecovering;
    }
    return true;
}

void BatteryMotionInterlock::Poll(uint32_t now_ms)
{
    if (!has_sample_)
    {
        reason_ = BatteryInterlockReason::kNoSample;
        return;
    }

    // Unsigned subtraction intentionally handles the 32-bit millisecond wrap.
    if (now_ms - last_sample_time_ms_ > config_.stale_timeout_ms)
    {
        if (!stale_)
        {
            stale_ = true;
            Trip(BatteryInterlockReason::kStale);
        }
        else
        {
            reason_ = BatteryInterlockReason::kStale;
        }
    }
}

BatteryCommandDecision BatteryMotionInterlock::EvaluateCommand(
    bool explicit_all_stop)
{
    if (explicit_all_stop)
    {
        if (voltage_ready_ && !stale_)
        {
            armed_ = true;
            reason_ = BatteryInterlockReason::kNone;
            return BatteryCommandDecision::kStopAndArm;
        }
        return BatteryCommandDecision::kStopOnly;
    }

    return MotionAllowed()
        ? BatteryCommandDecision::kExecuteMotion
        : BatteryCommandDecision::kRejectMotion;
}

bool BatteryMotionInterlock::MotionAllowed() const
{
    return voltage_ready_ && armed_ && !stale_;
}

BatteryMotionInterlockStatus BatteryMotionInterlock::Status(uint32_t now_ms) const
{
    BatteryMotionInterlockStatus status = {};
    status.reason = reason_;
    status.last_trip_reason = last_trip_reason_;
    status.has_sample = has_sample_;
    status.adc_valid = adc_valid_;
    status.sample_fresh = has_sample_ && !stale_ &&
                          now_ms - last_sample_time_ms_ <= config_.stale_timeout_ms;
    status.voltage_ready = voltage_ready_;
    status.armed = armed_;
    status.motion_allowed = MotionAllowed();
    status.voltage_v = voltage_v_;
    status.recovery_sample_count = recovery_sample_count_;
    status.sample_seq = last_sample_seq_;
    status.sample_age_ms = has_sample_ ? now_ms - last_sample_time_ms_ : 0;
    status.trip_count = trip_count_;
    return status;
}

const char *BatteryMotionInterlock::ReasonName(BatteryInterlockReason reason)
{
    switch (reason)
    {
    case BatteryInterlockReason::kNone:
        return "none";
    case BatteryInterlockReason::kNoSample:
        return "no_sample";
    case BatteryInterlockReason::kAdcInvalid:
        return "adc_invalid";
    case BatteryInterlockReason::kStale:
        return "stale";
    case BatteryInterlockReason::kSampleGap:
        return "sample_gap";
    case BatteryInterlockReason::kUndervoltage:
        return "undervoltage";
    case BatteryInterlockReason::kOvervoltage:
        return "overvoltage";
    case BatteryInterlockReason::kRecovering:
        return "recovering";
    case BatteryInterlockReason::kNeedsZeroRearm:
        return "needs_zero_rearm";
    default:
        return "unknown";
    }
}

void BatteryMotionInterlock::Trip(BatteryInterlockReason reason)
{
    const bool was_ready_or_armed = voltage_ready_ || armed_;
    if (was_ready_or_armed)
    {
        ++trip_count_;
    }
    last_trip_reason_ = reason;
    voltage_ready_ = false;
    armed_ = false;
    recovery_sample_count_ = 0;
    recovery_start_time_ms_ = 0;
    reason_ = reason;
}

void BatteryMotionInterlock::ResetRecovery(BatteryInterlockReason reason)
{
    voltage_ready_ = false;
    armed_ = false;
    recovery_sample_count_ = 0;
    recovery_start_time_ms_ = 0;
    reason_ = reason;
}
