#include "emergency_stop_gate.h"

#include <cstdlib>
#include <iostream>

namespace
{
int g_failures = 0;

void Expect(bool condition, const char *message)
{
  if (!condition)
  {
    std::cerr << "FAIL: " << message << '\n';
    ++g_failures;
  }
}

void TestPassesCommandsWhenNeverLatched()
{
  EmergencyStopGate gate;
  gate.Observe(false);
  Expect(gate.Admit(false), "motion command passes before any e-stop");
  Expect(gate.Admit(true), "stop command passes before any e-stop");
}

void TestDropsEverythingWhileLatched()
{
  EmergencyStopGate gate;
  gate.Observe(true);
  Expect(!gate.Admit(false), "motion command dropped while latched");
  Expect(!gate.Admit(true), "stop command also dropped while latched");
  Expect(gate.Active(), "gate reports active");
}

void TestReleaseRequiresZeroCommandFirst()
{
  EmergencyStopGate gate;
  gate.Observe(true);
  gate.Observe(false);
  Expect(gate.AwaitingZeroCommand(), "release waits for a zero command");
  Expect(!gate.Admit(false), "non-zero command rejected right after release");
  Expect(!gate.Admit(false), "still rejected until a zero command arrives");
  Expect(gate.Admit(true), "zero command re-arms and passes");
  Expect(!gate.AwaitingZeroCommand(), "gate re-armed");
  Expect(gate.Admit(false), "motion passes after re-arm");
}

void TestRelatchWhileAwaitingZero()
{
  EmergencyStopGate gate;
  gate.Observe(true);
  gate.Observe(false);
  gate.Observe(true);
  Expect(!gate.Admit(true), "re-latched gate drops zero command");
  gate.Observe(false);
  Expect(!gate.Admit(false), "still needs a zero command after second release");
  Expect(gate.Admit(true), "zero command re-arms");
}
} // namespace

int main()
{
  TestPassesCommandsWhenNeverLatched();
  TestDropsEverythingWhileLatched();
  TestReleaseRequiresZeroCommandFirst();
  TestRelatchWhileAwaitingZero();
  if (g_failures != 0)
  {
    std::cerr << g_failures << " failure(s)\n";
    return EXIT_FAILURE;
  }
  std::cout << "emergency_stop_gate_test passed\n";
  return EXIT_SUCCESS;
}
