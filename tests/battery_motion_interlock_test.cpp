#include "battery_motion_interlock.h"
#include "motion_command_safety.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <limits>

namespace
{
void Require(bool condition, int line)
{
    if (!condition)
    {
        std::fprintf(stderr, "requirement failed at line %d\n", line);
        std::abort();
    }
}

#define REQUIRE(condition) Require((condition), __LINE__)

void RecoverBattery(BatteryMotionInterlock &guard, uint32_t first_seq, uint32_t first_ms)
{
    REQUIRE(guard.ObserveSample(first_seq, first_ms, true, 7.4f));
    REQUIRE(guard.ObserveSample(first_seq + 1, first_ms + 500, true, 7.4f));
    REQUIRE(guard.ObserveSample(first_seq + 2, first_ms + 1000, true, 7.4f));
}

void ArmBattery(BatteryMotionInterlock &guard, uint32_t first_seq, uint32_t first_ms)
{
    RecoverBattery(guard, first_seq, first_ms);
    REQUIRE(!guard.MotionAllowed());
    REQUIRE(guard.EvaluateCommand(true) == BatteryCommandDecision::kStopAndArm);
    REQUIRE(guard.MotionAllowed());
}

void TestStartupRecoveryAndExplicitRearm()
{
    BatteryMotionInterlock guard;
    guard.Poll(100);
    REQUIRE(guard.Reason() == BatteryInterlockReason::kNoSample);
    REQUIRE(!guard.MotionAllowed());
    REQUIRE(guard.EvaluateCommand(false) == BatteryCommandDecision::kRejectMotion);
    REQUIRE(guard.EvaluateCommand(true) == BatteryCommandDecision::kStopOnly);

    REQUIRE(guard.ObserveSample(1, 100, true, 7.4f));
    REQUIRE(!guard.ObserveSample(1, 100, true, 7.4f));
    REQUIRE(guard.Status(100).recovery_sample_count == 1);
    REQUIRE(guard.ObserveSample(2, 600, true, 7.4f));
    REQUIRE(guard.ObserveSample(3, 1100, true, 7.4f));
    REQUIRE(guard.Status(1100).voltage_ready);
    REQUIRE(!guard.MotionAllowed());
    REQUIRE(guard.Reason() == BatteryInterlockReason::kNeedsZeroRearm);
    REQUIRE(guard.EvaluateCommand(false) == BatteryCommandDecision::kRejectMotion);
    REQUIRE(guard.EvaluateCommand(true) == BatteryCommandDecision::kStopAndArm);
    REQUIRE(guard.EvaluateCommand(false) == BatteryCommandDecision::kExecuteMotion);
}

void TestLowVoltageTripHysteresisAndNoAutomaticRestart()
{
    BatteryMotionInterlock guard;
    ArmBattery(guard, 1, 0);

    REQUIRE(guard.ObserveSample(4, 1500, true, 6.001f));
    REQUIRE(guard.MotionAllowed());
    REQUIRE(guard.ObserveSample(5, 2000, true, 6.0f));
    REQUIRE(!guard.MotionAllowed());
    REQUIRE(guard.Reason() == BatteryInterlockReason::kUndervoltage);
    REQUIRE(guard.Status(2000).trip_count == 1);

    REQUIRE(guard.ObserveSample(6, 2500, true, 6.39f));
    REQUIRE(guard.Status(2500).recovery_sample_count == 0);
    RecoverBattery(guard, 7, 3000);
    REQUIRE(guard.Status(4000).voltage_ready);
    REQUIRE(!guard.MotionAllowed());
    REQUIRE(guard.EvaluateCommand(false) == BatteryCommandDecision::kRejectMotion);
    REQUIRE(!guard.MotionAllowed());
    REQUIRE(guard.EvaluateCommand(true) == BatteryCommandDecision::kStopAndArm);
    REQUIRE(guard.MotionAllowed());
}

void TestUnarmedReadyStateCannotUseRunningHysteresisBand()
{
    BatteryMotionInterlock guard;
    RecoverBattery(guard, 1, 0);
    REQUIRE(guard.Status(1000).voltage_ready);
    REQUIRE(!guard.Status(1000).armed);

    REQUIRE(guard.ObserveSample(4, 1500, true, 6.1f));
    REQUIRE(!guard.Status(1500).voltage_ready);
    REQUIRE(guard.Status(1500).recovery_sample_count == 0);
    REQUIRE(guard.EvaluateCommand(true) == BatteryCommandDecision::kStopOnly);

    RecoverBattery(guard, 5, 2000);
    REQUIRE(guard.ObserveSample(8, 3500, true, 8.51f));
    REQUIRE(!guard.Status(3500).voltage_ready);
    REQUIRE(guard.EvaluateCommand(true) == BatteryCommandDecision::kStopOnly);
}

void TestHighVoltageAndInvalidSamples()
{
    BatteryMotionInterlock guard;
    ArmBattery(guard, 1, 0);
    REQUIRE(guard.ObserveSample(4, 1500, true, 8.6f));
    REQUIRE(guard.MotionAllowed());
    REQUIRE(guard.ObserveSample(5, 2000, true, 8.601f));
    REQUIRE(guard.Reason() == BatteryInterlockReason::kOvervoltage);
    REQUIRE(!guard.MotionAllowed());

    REQUIRE(guard.ObserveSample(6, 2500, true, 8.5f));
    REQUIRE(guard.Status(2500).recovery_sample_count == 1);
    REQUIRE(guard.ObserveSample(7, 3000, false, 7.4f));
    REQUIRE(guard.Reason() == BatteryInterlockReason::kAdcInvalid);
    REQUIRE(guard.ObserveSample(8, 3500, true, std::numeric_limits<float>::quiet_NaN()));
    REQUIRE(guard.Reason() == BatteryInterlockReason::kAdcInvalid);
    REQUIRE(!guard.Status(3500).adc_valid);
    REQUIRE(guard.ObserveSample(9, 4000, true, 3.63f));
    REQUIRE(guard.Reason() == BatteryInterlockReason::kUndervoltage);
}

void TestStaleBoundaryAndTickWrap()
{
    BatteryMotionInterlock guard;
    ArmBattery(guard, 1, 100);
    guard.Poll(2300);
    REQUIRE(guard.MotionAllowed());
    guard.Poll(2301);
    REQUIRE(!guard.MotionAllowed());
    REQUIRE(guard.Reason() == BatteryInterlockReason::kStale);
    REQUIRE(guard.Status(2301).trip_count == 1);
    guard.Poll(2400);
    REQUIRE(guard.Status(2400).trip_count == 1);

    BatteryMotionInterlock wrap_guard;
    REQUIRE(wrap_guard.ObserveSample(1, 0xfffffff0U, true, 7.4f));
    wrap_guard.Poll(0x000004a0U);
    REQUIRE(wrap_guard.Status(0x000004a0U).sample_fresh);
    wrap_guard.Poll(0x000004a1U);
    REQUIRE(wrap_guard.Reason() == BatteryInterlockReason::kStale);
}

void TestRecoveryRejectsSequenceGapsAndRapidCatchupSamples()
{
    BatteryMotionInterlock guard;
    REQUIRE(guard.ObserveSample(1, 0, true, 7.4f));
    REQUIRE(guard.ObserveSample(3, 500, true, 7.4f));
    REQUIRE(guard.Status(500).recovery_sample_count == 1);
    REQUIRE(!guard.Status(500).voltage_ready);

    REQUIRE(guard.ObserveSample(4, 600, true, 7.4f));
    REQUIRE(guard.Status(600).recovery_sample_count == 1);
    REQUIRE(guard.ObserveSample(5, 700, true, 7.4f));
    REQUIRE(guard.Status(700).recovery_sample_count == 1);

    REQUIRE(guard.ObserveSample(6, 1200, true, 7.4f));
    REQUIRE(guard.ObserveSample(7, 1699, true, 7.4f));
    REQUIRE(!guard.Status(1699).voltage_ready);
    REQUIRE(guard.ObserveSample(8, 2200, true, 7.4f));
    REQUIRE(guard.Status(2200).voltage_ready);

    REQUIRE(guard.EvaluateCommand(true) == BatteryCommandDecision::kStopAndArm);
    REQUIRE(guard.ObserveSample(10, 2700, true, 7.4f));
    REQUIRE(!guard.MotionAllowed());
    REQUIRE(guard.LastTripReason() == BatteryInterlockReason::kSampleGap);
    REQUIRE(guard.Status(2700).recovery_sample_count == 1);
}

void TestRecoveryRequiresFullDuration()
{
    BatteryMotionInterlock guard;
    REQUIRE(guard.ObserveSample(1, 0, true, 7.4f));
    REQUIRE(guard.ObserveSample(2, 400, true, 7.4f));
    REQUIRE(guard.ObserveSample(3, 800, true, 7.4f));
    REQUIRE(guard.Status(800).recovery_sample_count == 3);
    REQUIRE(!guard.Status(800).voltage_ready);
    REQUIRE(guard.ObserveSample(4, 1200, true, 7.4f));
    REQUIRE(guard.Status(1200).voltage_ready);
}

void TestRecoveryHandlesSequenceAndTimeWrap()
{
    BatteryMotionInterlock sequence_wrap;
    REQUIRE(sequence_wrap.ObserveSample(UINT32_MAX - 1U, 0, true, 7.4f));
    REQUIRE(sequence_wrap.ObserveSample(UINT32_MAX, 500, true, 7.4f));
    REQUIRE(sequence_wrap.ObserveSample(0, 1000, true, 7.4f));
    REQUIRE(sequence_wrap.Status(1000).voltage_ready);

    BatteryMotionInterlock time_wrap;
    REQUIRE(time_wrap.ObserveSample(1, 0xffffff00U, true, 7.4f));
    REQUIRE(time_wrap.ObserveSample(2, 0x000000f4U, true, 7.4f));
    REQUIRE(time_wrap.ObserveSample(3, 0x000002e8U, true, 7.4f));
    REQUIRE(time_wrap.Status(0x000002e8U).voltage_ready);
}

void TestObservedSampleGapBoundary()
{
    BatteryMotionInterlock guard;
    ArmBattery(guard, 1, 0);
    REQUIRE(guard.ObserveSample(4, 2200, true, 7.4f));
    REQUIRE(guard.MotionAllowed());
    REQUIRE(guard.ObserveSample(5, 3401, true, 7.4f));
    REQUIRE(!guard.MotionAllowed());
    REQUIRE(guard.LastTripReason() == BatteryInterlockReason::kStale);
    REQUIRE(guard.Status(3401).recovery_sample_count == 1);
}

void TestRecoveryVoltageBandIsClosed()
{
    BatteryMotionInterlock guard;

    REQUIRE(guard.ObserveSample(1, 0, true, 6.4f));
    REQUIRE(guard.ObserveSample(2, 500, true, 8.5f));
    REQUIRE(guard.ObserveSample(3, 1000, true, 6.4f));
    REQUIRE(guard.Status(1000).voltage_ready);
}

void TestCommandClassification()
{
    MotionMsg command = {};
    command.control_mode = 0;
    REQUIRE(IsExplicitAllStopCommand(command));
    command.target_vx = 0.01f;
    REQUIRE(!IsExplicitAllStopCommand(command));
    command.target_vx = 0.0f;
    command.target_wz = std::numeric_limits<float>::infinity();
    REQUIRE(!IsExplicitAllStopCommand(command));

    command = {};
    for (uint8_t mode : {1, 2, 3, 6})
    {
        command.control_mode = mode;
        REQUIRE(!IsExplicitAllStopCommand(command));
    }
}
} // namespace

int main()
{
    TestStartupRecoveryAndExplicitRearm();
    TestLowVoltageTripHysteresisAndNoAutomaticRestart();
    TestUnarmedReadyStateCannotUseRunningHysteresisBand();
    TestHighVoltageAndInvalidSamples();
    TestStaleBoundaryAndTickWrap();
    TestRecoveryRejectsSequenceGapsAndRapidCatchupSamples();
    TestRecoveryRequiresFullDuration();
    TestRecoveryHandlesSequenceAndTimeWrap();
    TestObservedSampleGapBoundary();
    TestRecoveryVoltageBandIsClosed();
    TestCommandClassification();
}
