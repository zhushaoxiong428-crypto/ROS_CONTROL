#include "heading_hold.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace
{
int g_failures = 0;
constexpr float kDt = 0.02f;
constexpr float kPi = 3.14159265358979f;

void Expect(bool condition, const char *message)
{
  if (!condition)
  {
    std::cerr << "FAIL: " << message << '\n';
    ++g_failures;
  }
}

bool Near(float a, float b, float tol = 1e-4f)
{
  return std::abs(a - b) <= tol;
}

void TestDisengagedOutputsZero()
{
  HeadingHold hold;
  Expect(Near(hold.Update(false, 0.3f, kDt), 0.0f), "no correction when not driving straight");
  Expect(!hold.Active(), "inactive when not engaged");
}

void TestLatchesReferenceOnEngage()
{
  HeadingHold hold;
  Expect(Near(hold.Update(true, 0.7f, kDt), 0.0f), "zero error on the first engaged cycle");
  Expect(hold.Active(), "active after engage");
  Expect(Near(hold.ReferenceRad(), 0.7f), "reference is the heading at engage time");
}

void TestLeftDriftCommandsRightTurn()
{
  // 车向左偏（航向逆时针增大）时，修正量应为负，即向右转。
  HeadingHold hold;
  hold.Update(true, 0.0f, kDt);
  const float wz = hold.Update(true, 0.1f, kDt);
  Expect(wz < 0.0f, "left drift gives a clockwise (negative) correction");
  HeadingHold hold2;
  hold2.Update(true, 0.0f, kDt);
  Expect(hold2.Update(true, -0.1f, kDt) > 0.0f, "right drift gives a counter-clockwise correction");
}

void TestIntegralRemovesConstantDisturbance()
{
  // 简单闭环：航向变化率 = 修正量 + 恒定左偏扰动。积分项应把稳态误差压到接近 0。
  HeadingHold hold;
  const float disturbance = 0.05f; // rad/s，约 2.9 deg/s 的持续左偏
  float yaw = 0.0f;
  for (int i = 0; i < 1500; ++i) // 30 s
  {
    const float wz = hold.Update(true, yaw, kDt);
    yaw += (wz + disturbance) * kDt;
  }
  Expect(std::abs(yaw) < 0.005f, "steady-state heading error below 0.3 deg under constant drift");
}

void TestOutputIsLimited()
{
  HeadingHoldConfig config;
  HeadingHold hold(config);
  hold.Update(true, 0.0f, kDt);
  float wz = 0.0f;
  for (int i = 0; i < 200; ++i)
  {
    wz = hold.Update(true, 1.0f, kDt);
  }
  Expect(Near(wz, -config.max_wz), "correction clamped to max_wz");
  // 误差反向后应能很快离开饱和（抗积分饱和）。
  const float recovered = hold.Update(true, -0.2f, kDt);
  Expect(recovered > 0.0f, "anti-windup lets the output recover promptly");
}

void TestWrapAroundPi()
{
  HeadingHold hold;
  hold.Update(true, kPi - 0.05f, kDt);
  const float wz = hold.Update(true, -kPi + 0.05f, kDt); // 实际只向左转了 0.1 rad
  Expect(wz < 0.0f && wz > -0.3f, "error wraps across +/-pi instead of seeing a ~2pi jump");
}

void TestDisengageResets()
{
  HeadingHold hold;
  hold.Update(true, 0.0f, kDt);
  hold.Update(true, 0.2f, kDt);
  hold.Update(false, 0.2f, kDt);
  Expect(!hold.Active(), "turning or stopping resets the hold");
  hold.Update(true, 0.5f, kDt);
  Expect(Near(hold.ReferenceRad(), 0.5f), "next straight segment latches a fresh reference");
}

void TestRejectsInvalidInput()
{
  HeadingHold hold;
  hold.Update(true, 0.0f, kDt);
  Expect(Near(hold.Update(true, NAN, kDt), 0.0f), "NaN yaw gives no correction");
  Expect(!hold.Active(), "NaN yaw resets the hold");
}
} // namespace

int main()
{
  TestDisengagedOutputsZero();
  TestLatchesReferenceOnEngage();
  TestLeftDriftCommandsRightTurn();
  TestIntegralRemovesConstantDisturbance();
  TestOutputIsLimited();
  TestWrapAroundPi();
  TestDisengageResets();
  TestRejectsInvalidInput();
  if (g_failures != 0)
  {
    std::cerr << g_failures << " failure(s)\n";
    return EXIT_FAILURE;
  }
  std::cout << "heading_hold_test passed\n";
  return EXIT_SUCCESS;
}
