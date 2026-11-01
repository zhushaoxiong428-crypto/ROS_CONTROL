#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

#include "my_robot/mavlink.h"

namespace leap_bridge {

// System/component ids of the firmware (mavlink_common.cpp) and of this host.
constexpr uint8_t kRobotSystemId = 1;
constexpr uint8_t kRobotComponentId = 1;
constexpr uint8_t kHostSystemId = 255;
constexpr uint8_t kHostComponentId = MAV_COMP_ID_ONBOARD_COMPUTER;

struct LinkStats {
    uint64_t bytes = 0;
    uint64_t frames = 0;
    uint64_t crc_errors = 0;
    // Frames the robot sent but we never saw, from gaps in its sequence numbers.
    uint64_t lost_frames = 0;
};

// Turns the UART byte stream into MAVLink messages and back. The firmware
// shares UART0 with its log console, so the stream is MAVLink frames mixed
// with ASCII log lines; the parser skips the text because ASCII never
// contains a MAVLink start byte (0xFD / 0xFE).
class MavlinkLink {
public:
    using MessageHandler = std::function<void(const mavlink_message_t &)>;

    void SetHandler(MessageHandler handler) { handler_ = std::move(handler); }

    // Feed raw bytes; calls the handler once per complete, CRC-valid frame.
    void Feed(const uint8_t *data, size_t len);

    const LinkStats &stats() const { return stats_; }

    // Velocity setpoint in the /cmd_vel convention: vx in m/s (forward),
    // wz in rad/s (counter-clockwise). Returns the frame length in buf.
    static size_t PackVelocityCommand(float vx, float wz, uint8_t *buf);

private:
    void TrackSequence(const mavlink_message_t &msg);

    mavlink_message_t rx_buffer_ = {};
    mavlink_status_t rx_status_ = {};
    MessageHandler handler_;
    LinkStats stats_;
    bool have_last_seq_ = false;
    uint8_t last_seq_ = 0;
};

}  // namespace leap_bridge
