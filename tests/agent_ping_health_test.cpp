#include "agent_ping_health.h"

#include <cstdlib>

namespace
{
void Require(bool condition)
{
    if (!condition)
    {
        std::abort();
    }
}

void TestSustainedFailureWindow()
{
    AgentPingHealth health;
    Require(health.KeepSession(false, 1000, 12000, 3));
    Require(health.KeepSession(false, 3000, 12000, 3));
    Require(health.KeepSession(false, 5000, 12000, 3));
    Require(health.KeepSession(false, 7000, 12000, 3));
    Require(health.KeepSession(false, 9000, 12000, 3));
    Require(health.KeepSession(false, 11000, 12000, 3));
    Require(health.consecutive_failures == 6);
    Require(health.FailureAgeTicks(11000) == 10000);

    Require(!health.KeepSession(false, 13000, 12000, 3));
    Require(health.consecutive_failures == 7);
    Require(health.FailureAgeTicks(13000) == 12000);
}

void TestSuccessClearsFailureStreak()
{
    AgentPingHealth health;
    Require(health.KeepSession(false, 100, 12000, 3));
    Require(health.KeepSession(false, 2100, 12000, 3));
    Require(health.consecutive_failures == 2);
    Require(health.failure_window_active);

    Require(health.KeepSession(true, 2200, 12000, 3));
    Require(health.consecutive_failures == 0);
    Require(!health.failure_window_active);
    Require(health.FailureAgeTicks(2200) == 0);

    Require(health.KeepSession(false, 5000, 12000, 3));
    Require(health.consecutive_failures == 1);

    health.Reset();
    Require(health.consecutive_failures == 0);
    Require(!health.failure_window_active);
}

void TestMinimumFailureCountAndTickWrap()
{
    AgentPingHealth health;
    Require(health.KeepSession(false, 100, 0, 3));
    Require(health.consecutive_failures == 1);
    Require(health.KeepSession(false, 101, 0, 3));
    Require(!health.KeepSession(false, 102, 0, 3));

    health.Reset();
    Require(health.KeepSession(false, 0xfffffff0U, 32, 2));
    Require(!health.KeepSession(false, 0x00000010U, 32, 2));
    Require(health.FailureAgeTicks(0x00000010U) == 32);
}
} // namespace

int main()
{
    TestSustainedFailureWindow();
    TestSuccessClearsFailureStreak();
    TestMinimumFailureCountAndTickWrap();
}
