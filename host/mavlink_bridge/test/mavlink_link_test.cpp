#include "leap_bridge/mavlink_link.h"

#include <cassert>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

using leap_bridge::kRobotComponentId;
using leap_bridge::kRobotSystemId;
using leap_bridge::MavlinkLink;

namespace {

// Packs a frame the way the firmware does; each call advances the sequence number.
std::vector<uint8_t> RobotOdometry(float x) {
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float covariance[21] = {NAN};
    mavlink_message_t msg;
    mavlink_msg_odometry_pack(kRobotSystemId, kRobotComponentId, &msg, 0,
                              MAV_FRAME_LOCAL_FLU, MAV_FRAME_BODY_FRD,
                              x, 0.0f, 0.0f, q, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                              covariance, covariance, 0, MAV_ESTIMATOR_TYPE_NAIVE, 0);
    std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);
    buf.resize(mavlink_msg_to_send_buffer(buf.data(), &msg));
    return buf;
}

std::vector<uint8_t> RobotHeartbeat() {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(kRobotSystemId, kRobotComponentId, &msg, MAV_TYPE_GROUND_ROVER,
                               MAV_AUTOPILOT_GENERIC, 0, 0, MAV_STATE_ACTIVE);
    std::vector<uint8_t> buf(MAVLINK_MAX_PACKET_LEN);
    buf.resize(mavlink_msg_to_send_buffer(buf.data(), &msg));
    return buf;
}

void Append(std::vector<uint8_t> *stream, const std::vector<uint8_t> &bytes) {
    stream->insert(stream->end(), bytes.begin(), bytes.end());
}

void Append(std::vector<uint8_t> *stream, const std::string &text) {
    stream->insert(stream->end(), text.begin(), text.end());
}

// Feeds in small chunks so frames are split across reads, as on a real UART.
void FeedInChunks(MavlinkLink *link, const std::vector<uint8_t> &stream, size_t chunk) {
    for (size_t i = 0; i < stream.size(); i += chunk) {
        link->Feed(stream.data() + i, std::min(chunk, stream.size() - i));
    }
}

void TestFramesBetweenLogLines() {
    MavlinkLink link;
    std::vector<uint32_t> ids;
    float odom_x = 0.0f;
    link.SetHandler([&](const mavlink_message_t &msg) {
        ids.push_back(msg.msgid);
        if (msg.msgid == MAVLINK_MSG_ID_ODOMETRY) {
            odom_x = mavlink_msg_odometry_get_x(&msg);
        }
    });

    // UART0 carries the firmware's log console as well as MAVLink.
    std::vector<uint8_t> stream;
    Append(&stream, "I (1234) MOTION: loop period 20.000 ms\r\n");
    Append(&stream, RobotOdometry(1.5f));
    RobotOdometry(9.0f);  // sent by the robot but lost on the wire
    Append(&stream, "W (1300) MAVLINK: tx buffer low\r\n");
    Append(&stream, RobotHeartbeat());

    FeedInChunks(&link, stream, 7);

    assert(ids.size() == 2);
    assert(ids[0] == MAVLINK_MSG_ID_ODOMETRY);
    assert(ids[1] == MAVLINK_MSG_ID_HEARTBEAT);
    assert(odom_x == 1.5f);
    assert(link.stats().frames == 2);
    assert(link.stats().lost_frames == 1);
    assert(link.stats().crc_errors == 0);
    assert(link.stats().bytes == stream.size());
}

void TestCorruptedFrameIsDropped() {
    MavlinkLink link;
    int delivered = 0;
    link.SetHandler([&](const mavlink_message_t &) { ++delivered; });

    // Pack in sending order so the sequence numbers run 0, 1, 2 along the stream.
    std::vector<uint8_t> stream;
    Append(&stream, RobotHeartbeat());
    std::vector<uint8_t> corrupted = RobotOdometry(2.0f);
    corrupted[MAVLINK_NUM_HEADER_BYTES + 2] ^= 0x40;  // a payload byte
    Append(&stream, corrupted);
    Append(&stream, RobotHeartbeat());
    link.Feed(stream.data(), stream.size());

    // The parser resynchronises on the next frame; the bad one counts as lost.
    assert(delivered == 2);
    assert(link.stats().crc_errors == 1);
    assert(link.stats().lost_frames == 1);
}

void TestSequenceWrapIsNotLoss() {
    MavlinkLink link;
    std::vector<uint8_t> stream;
    for (int i = 0; i < 300; ++i) {
        Append(&stream, RobotHeartbeat());
    }
    link.Feed(stream.data(), stream.size());
    assert(link.stats().frames == 300);
    assert(link.stats().lost_frames == 0);
}

// Checks the command the way the firmware reads it (HandleSetPositionTargetLocalNed).
void TestVelocityCommandIsVelocityMode() {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const size_t len = MavlinkLink::PackVelocityCommand(0.25f, -0.5f, buf);

    MavlinkLink link;
    mavlink_set_position_target_local_ned_t cmd = {};
    int delivered = 0;
    link.SetHandler([&](const mavlink_message_t &msg) {
        assert(msg.msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED);
        mavlink_msg_set_position_target_local_ned_decode(&msg, &cmd);
        ++delivered;
    });
    link.Feed(buf, len);
    assert(delivered == 1);

    auto used = [&](uint16_t ignore_flag) { return (cmd.type_mask & ignore_flag) == 0; };
    // Any used position or yaw field would switch the firmware to position mode.
    assert(!used(POSITION_TARGET_TYPEMASK_X_IGNORE));
    assert(!used(POSITION_TARGET_TYPEMASK_Y_IGNORE));
    assert(!used(POSITION_TARGET_TYPEMASK_YAW_IGNORE));
    assert(used(POSITION_TARGET_TYPEMASK_VX_IGNORE));
    assert(used(POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE));

    assert(cmd.vx == 0.25f);
    assert(cmd.vy == 0.0f);
    assert(cmd.yaw_rate == -0.5f);
    assert(cmd.target_system == kRobotSystemId);
    // Host frames do not count towards the robot's sequence tracking.
    assert(link.stats().lost_frames == 0);
}

}  // namespace

int main() {
    TestFramesBetweenLogLines();
    TestCorruptedFrameIsDropped();
    TestSequenceWrapIsNotLoss();
    TestVelocityCommandIsVelocityMode();
    return 0;
}
