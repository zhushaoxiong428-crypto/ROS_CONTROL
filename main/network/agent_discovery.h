#pragma once

#include <stdint.h>
#include <string.h>

// micro-ROS agent discovery helpers. Addresses are IPv4 in host byte order.

static constexpr const char kAutoAgentIp[] = "auto";

// Subnets larger than /24 are only reached by broadcast; sweeping them host by
// host would take too long.
static constexpr uint32_t kMaxSweepNetmask = 0xFFFFFF00u;

inline bool IsAutoAgentIp(const char *ip) {
  return ip != nullptr && strcmp(ip, kAutoAgentIp) == 0;
}

inline uint32_t SubnetBroadcast(uint32_t ip, uint32_t netmask) {
  return ip | ~netmask;
}

// Calls fn(host) for every address on the local subnet except the network
// address, the broadcast address and the robot itself. Returns the count.
template <typename Fn>
uint32_t ForEachSubnetHost(uint32_t ip, uint32_t netmask, Fn fn) {
  if (netmask < kMaxSweepNetmask) {
    return 0;
  }
  const uint32_t network = ip & netmask;
  const uint32_t broadcast = SubnetBroadcast(ip, netmask);
  uint32_t count = 0;
  for (uint32_t host = network + 1; host < broadcast; ++host) {
    if (host != ip) {
      fn(host);
      ++count;
    }
  }
  return count;
}
