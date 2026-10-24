#include "agent_discovery.h"

#include <cassert>
#include <cstdint>
#include <vector>

static constexpr uint32_t Ip(uint32_t a, uint32_t b, uint32_t c, uint32_t d) {
    return (a << 24) | (b << 16) | (c << 8) | d;
}

int main() {
    assert(IsAutoAgentIp("auto"));
    assert(!IsAutoAgentIp("AUTO"));
    assert(!IsAutoAgentIp("10.80.9.120"));
    assert(!IsAutoAgentIp(""));
    assert(!IsAutoAgentIp(nullptr));

    // Phone hotspot: robot 10.80.9.48/24.
    const uint32_t robot = Ip(10, 80, 9, 48);
    const uint32_t mask24 = Ip(255, 255, 255, 0);
    assert(SubnetBroadcast(robot, mask24) == Ip(10, 80, 9, 255));

    std::vector<uint32_t> hosts;
    const uint32_t count = ForEachSubnetHost(robot, mask24, [&](uint32_t host) { hosts.push_back(host); });
    // .1 to .254, minus the robot itself.
    assert(count == 253);
    assert(hosts.size() == 253);
    assert(hosts.front() == Ip(10, 80, 9, 1));
    assert(hosts.back() == Ip(10, 80, 9, 254));
    for (uint32_t host : hosts) {
        assert(host != robot);
        assert(host != Ip(10, 80, 9, 0));
        assert(host != Ip(10, 80, 9, 255));
    }

    // Smaller subnet: 192.168.4.2/28 sweeps .1 to .14 except itself.
    hosts.clear();
    assert(ForEachSubnetHost(Ip(192, 168, 4, 2), Ip(255, 255, 255, 240),
                             [&](uint32_t host) { hosts.push_back(host); }) == 13);
    assert(hosts.front() == Ip(192, 168, 4, 1));
    assert(hosts.back() == Ip(192, 168, 4, 14));

    // Campus /16: too large to sweep, broadcast only.
    assert(ForEachSubnetHost(Ip(10, 80, 9, 48), Ip(255, 255, 0, 0), [](uint32_t) { assert(false); }) == 0);
    assert(SubnetBroadcast(Ip(10, 80, 9, 48), Ip(255, 255, 0, 0)) == Ip(10, 80, 255, 255));
}
