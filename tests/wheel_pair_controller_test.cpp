#include "wheel_pair_controller.h"

#include <cstdio>
#include <cstdlib>
#include <cmath>

namespace
{
bool Near(float lhs, float rhs, float tolerance = 0.001f)
{
  return std::abs(lhs - rhs) <= tolerance;
}

void RequireAt(bool condition, int line)
{
  if (!condition)
  {
    std::fprintf(stderr, "wheel_pair_controller_test assertion failed at line %d\n", line);
    std::abort();
  }
}

#define Require(condition) RequireAt((condition), __LINE__)

WheelPairController MakeController()
{
  return WheelPairController(WheelPairControllerConfig{});
}

void TestInactiveDuringTurn()
{
  auto controller = MakeController();
  const auto output = controller.Update(
      10.0f, 20.0f, 10.0f, 20.0f,
      0.0f, 0.0f, 130.0f, 140.0f, 0.02f, false);
  Require(!output.sync_active);
  Require(!output.startup_active);
  Require(Near(output.left_pwm, 130.0f));
  Require(Near(output.right_pwm, 140.0f));
}

void TestStraightStartAppliesBreakawayFloor()
{
  auto controller = MakeController();
  const auto output = controller.Update(
      2.0f, 2.0f, 29.38f, 29.38f,
      0.0f, 0.0f, 122.0f, 136.0f, 0.02f, true);
  Require(output.sync_active);
  Require(output.startup_active);
  Require(Near(output.left_pwm, 145.0f));
  Require(Near(output.right_pwm, 145.0f));
}

void TestStartedWheelIsReleasedWhileOtherGetsAssistance()
{
  auto controller = MakeController();
  controller.Update(
      2.0f, 2.0f, 29.38f, 29.38f,
      0.0f, 0.0f, 122.0f, 136.0f, 0.02f, true);
  const auto output = controller.Update(
      6.0f, 6.0f, 29.38f, 29.38f,
      0.0f, 2.0f, 130.0f, 153.0f, 0.02f, true);

  Require(output.startup_active);
  Require(output.sync_correction_pwm > 0.0f);
  Require(output.left_pwm >= 145.0f);
  Require(output.right_pwm <= 145.0f);
}

void TestForwardStartupReleasesGraduallyAfterBothWheelsMove()
{
  auto controller = MakeController();
  controller.Update(
      2.0f, 2.0f, 29.38f, 29.38f,
      0.0f, 0.0f, 122.0f, 136.0f, 0.02f, true);
  auto output = controller.Update(
      8.0f, 8.0f, 29.38f, 29.38f,
      2.0f, 2.0f, 136.0f, 150.0f, 0.02f, true);

  // The first release sample remains at the already-proven breakaway output;
  // it must not expose the full per-wheel feedforward difference at once.
  Require(output.startup_active);
  Require(output.sync_active);
  Require(Near(output.left_pwm, 145.0f));
  Require(Near(output.right_pwm, 145.0f));

  output = controller.Update(
      8.0f, 8.0f, 29.38f, 29.38f,
      2.0f, 2.0f, 136.0f, 150.0f, 0.02f, true);
  Require(output.startup_active);
  Require(Near(output.left_pwm, 145.0f + (136.0f - 145.0f) / 15.0f));
  Require(Near(output.right_pwm, 145.0f + (150.0f - 145.0f) / 15.0f));

  for (int i = 2; i <= 15; ++i)
  {
    output = controller.Update(
        8.0f, 8.0f, 29.38f, 29.38f,
        2.0f, 2.0f, 136.0f, 150.0f, 0.02f, true);
  }
  Require(!output.startup_active);
  Require(!output.startup_fault);
  Require(Near(output.left_pwm, 136.0f));
  Require(Near(output.right_pwm, 150.0f));
}

void TestReleaseLongerThanBreakawayTimeoutDoesNotFault()
{
  WheelPairControllerConfig config = {};
  config.startup_release_duration_s = 0.40f;
  WheelPairController controller(config);
  controller.Update(
      2.0f, 2.0f, 29.38f, 29.38f,
      0.0f, 0.0f, 122.0f, 136.0f, 0.02f, true);
  auto output = controller.Update(
      8.0f, 8.0f, 29.38f, 29.38f,
      2.0f, 2.0f, 136.0f, 150.0f, 0.02f, true);
  for (int i = 0; i < 25; ++i)
  {
    output = controller.Update(
        8.0f, 8.0f, 29.38f, 29.38f,
        2.0f, 2.0f, 136.0f, 150.0f, 0.02f, true);
  }
  Require(!output.startup_fault);
  Require(!output.startup_active);
}

void TestRollingStraightCommandDoesNotArmRelease()
{
  auto controller = MakeController();
  const auto output = controller.Update(
      20.0f, 20.0f, 29.38f, 29.38f,
      5.0f, 5.0f, 136.0f, 150.0f, 0.02f, true);
  Require(output.sync_active);
  Require(!output.startup_active);
  Require(Near(output.left_pwm, 136.0f));
  Require(Near(output.right_pwm, 150.0f));
}

void TestTurnCancelsForwardRelease()
{
  auto controller = MakeController();
  controller.Update(
      2.0f, 2.0f, 29.38f, 29.38f,
      0.0f, 0.0f, 122.0f, 136.0f, 0.02f, true);
  auto output = controller.Update(
      8.0f, 8.0f, 29.38f, 29.38f,
      2.0f, 2.0f, 136.0f, 150.0f, 0.02f, true);
  Require(output.startup_active);

  output = controller.Update(
      10.0f, 20.0f, 10.0f, 20.0f,
      2.0f, 2.0f, 130.0f, 150.0f, 0.02f, false);
  Require(!output.startup_active);
  Require(!output.sync_active);
  Require(Near(output.left_pwm, 130.0f));
  Require(Near(output.right_pwm, 150.0f));
}

void TestWheelThatDropsBackGetsAssistanceAgain()
{
  auto controller = MakeController();
  controller.Update(
      2.0f, 2.0f, 29.38f, 29.38f,
      0.0f, 2.0f, 122.0f, 153.0f, 0.02f, true);
  const auto output = controller.Update(
      4.0f, 4.0f, 29.38f, 29.38f,
      0.0f, 0.5f, 130.0f, 140.0f, 0.02f, true);

  Require(output.startup_active);
  Require(output.left_pwm >= 145.0f);
  Require(output.right_pwm >= 145.0f);
}

void TestReverseStartupUsesNegativeFloor()
{
  auto controller = MakeController();
  const auto output = controller.Update(
      -2.0f, -2.0f, -29.38f, -29.38f,
      0.0f, 0.0f, -122.0f, -136.0f, 0.02f, true);
  Require(output.startup_active);
  Require(Near(output.left_pwm, -145.0f));
  Require(Near(output.right_pwm, -145.0f));

  const auto released = controller.Update(
      -8.0f, -8.0f, -29.38f, -29.38f,
      -2.0f, -2.0f, -136.0f, -150.0f, 0.02f, true);
  Require(!released.startup_active);
  Require(Near(released.left_pwm, -136.0f));
  Require(Near(released.right_pwm, -150.0f));
}

void TestOldDirectionSpeedDoesNotCompleteReverseStartup()
{
  auto controller = MakeController();
  const auto waiting = controller.Update(
      -2.0f, -2.0f, -29.38f, -29.38f,
      5.0f, 4.0f, -122.0f, -136.0f, 0.02f, true);

  Require(waiting.startup_active);
  Require(Near(waiting.left_pwm, 0.0f));
  Require(Near(waiting.right_pwm, 0.0f));

  const auto released = controller.Update(
      -2.0f, -2.0f, -29.38f, -29.38f,
      0.5f, 0.2f, -122.0f, -136.0f, 0.02f, true);
  Require(released.startup_active);
  Require(Near(released.left_pwm, -145.0f));
  Require(Near(released.right_pwm, -145.0f));
}

void TestTurnResetsAccumulatedTrim()
{
  auto controller = MakeController();
  for (int i = 0; i < 20; ++i)
  {
    controller.Update(
        20.0f, 20.0f, 29.38f, 29.38f,
        10.0f, 15.0f, 150.0f, 150.0f, 0.02f, true);
  }

  const auto turn = controller.Update(
      10.0f, 20.0f, 10.0f, 20.0f,
      10.0f, 15.0f, 140.0f, 150.0f, 0.02f, false);
  Require(!turn.sync_active);
  Require(Near(turn.sync_correction_pwm, 0.0f));

  const auto straight = controller.Update(
      20.0f, 20.0f, 29.38f, 29.38f,
      20.0f, 20.0f, 145.0f, 151.0f, 0.02f, true);
  Require(straight.sync_active);
  Require(Near(straight.sync_correction_pwm, 0.0f));
}

void TestStartupTimeoutLatchesFaultUntilReset()
{
  auto controller = MakeController();
  WheelPairControlOutput output = {};
  for (int i = 0; i < 20; ++i)
  {
    output = controller.Update(
        20.0f, 20.0f, 29.38f, 29.38f,
        0.0f, 0.0f, 145.0f, 145.0f, 0.02f, true);
  }
  Require(output.startup_fault);
  Require(Near(output.left_pwm, 0.0f));
  Require(Near(output.right_pwm, 0.0f));

  output = controller.Update(
      20.0f, 20.0f, 29.38f, 29.38f,
      20.0f, 20.0f, 145.0f, 151.0f, 0.02f, true);
  Require(output.startup_fault);

  controller.Reset();
  output = controller.Update(
      20.0f, 20.0f, 29.38f, 29.38f,
      20.0f, 20.0f, 145.0f, 151.0f, 0.02f, true);
  Require(!output.startup_fault);
}

void TestLowSpeedCommandDoesNotUseStartupAssist()
{
  auto controller = MakeController();
  const auto output = controller.Update(
      2.0f, 2.0f, 13.99f, 13.99f,
      0.0f, 0.0f, 122.0f, 136.0f, 0.02f, true);
  Require(!output.sync_active);
  Require(!output.startup_active);
  Require(Near(output.left_pwm, 122.0f));
  Require(Near(output.right_pwm, 136.0f));
}

void TestSyncCorrectionRespectsPwmHeadroom()
{
  auto controller = MakeController();
  const auto output = controller.Update(
      20.0f, 20.0f, 29.38f, 29.38f,
      10.0f, 20.0f, 255.0f, 150.0f, 0.02f, true);
  Require(Near(output.sync_correction_pwm, 0.0f));
  Require(Near(output.left_pwm, 255.0f));
}

void TestReverseSyncCorrectionHasCorrectSign()
{
  auto controller = MakeController();
  const auto output = controller.Update(
      -20.0f, -20.0f, -29.38f, -29.38f,
      -10.0f, -15.0f, -145.0f, -151.0f, 0.02f, true);
  Require(output.sync_correction_pwm < 0.0f);
  Require(output.left_pwm < -145.0f);
  Require(output.right_pwm > -151.0f);
}

void TestNonFiniteInputFailsClosed()
{
  auto controller = MakeController();
  const auto output = controller.Update(
      NAN, 20.0f, 29.38f, 29.38f,
      0.0f, 0.0f, 145.0f, 145.0f, 0.02f, true);
  Require(output.startup_fault);
  Require(Near(output.left_pwm, 0.0f));
  Require(Near(output.right_pwm, 0.0f));
}
} // namespace

int main()
{
  TestInactiveDuringTurn();
  TestStraightStartAppliesBreakawayFloor();
  TestStartedWheelIsReleasedWhileOtherGetsAssistance();
  TestForwardStartupReleasesGraduallyAfterBothWheelsMove();
  TestReleaseLongerThanBreakawayTimeoutDoesNotFault();
  TestRollingStraightCommandDoesNotArmRelease();
  TestTurnCancelsForwardRelease();
  TestWheelThatDropsBackGetsAssistanceAgain();
  TestReverseStartupUsesNegativeFloor();
  TestOldDirectionSpeedDoesNotCompleteReverseStartup();
  TestTurnResetsAccumulatedTrim();
  TestStartupTimeoutLatchesFaultUntilReset();
  TestLowSpeedCommandDoesNotUseStartupAssist();
  TestSyncCorrectionRespectsPwmHeadroom();
  TestReverseSyncCorrectionHasCorrectSign();
  TestNonFiniteInputFailsClosed();
  return 0;
}
