#include "leap_bridge/mavlink_link.h"

namespace leap_bridge {

void MavlinkLink::Feed(const uint8_t *data, size_t len) {
    stats_.bytes += len;
    for (size_t i = 0; i < len; ++i) {
        mavlink_message_t msg;
        mavlink_status_t status;
        // The per-object state (not mavlink_parse_char's global channel table)
        // keeps two links, or a test and a link, from sharing a parser.
        const uint8_t result = mavlink_frame_char_buffer(&rx_buffer_, &rx_status_, data[i], &msg, &status);
        if (result == MAVLINK_FRAMING_BAD_CRC) {
            ++stats_.crc_errors;
        } else if (result == MAVLINK_FRAMING_OK) {
            ++stats_.frames;
            TrackSequence(msg);
            if (handler_) {
                handler_(msg);
            }
        }
    }
}

void MavlinkLink::TrackSequence(const mavlink_message_t &msg) {
    if (msg.sysid != kRobotSystemId || msg.compid != kRobotComponentId) {
        return;
    }
    if (have_last_seq_) {
        // uint8_t arithmetic wraps 255 -> 0 the same way the sender's counter does.
        const uint8_t gap = static_cast<uint8_t>(msg.seq - last_seq_ - 1);
        stats_.lost_frames += gap;
    }
    have_last_seq_ = true;
    last_seq_ = msg.seq;
}

size_t MavlinkLink::PackVelocityCommand(float vx, float wz, uint8_t *buf) {
    // The firmware takes velocity mode when no position or yaw field is used
    // (HandleSetPositionTargetLocalNed), so ignore everything except vx, vy, yaw_rate.
    constexpr uint16_t kVelocityOnly =
        POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE |
        POSITION_TARGET_TYPEMASK_Z_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE |
        POSITION_TARGET_TYPEMASK_AZ_IGNORE | POSITION_TARGET_TYPEMASK_YAW_IGNORE;

    mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_pack(
        kHostSystemId, kHostComponentId, &msg,
        0,  // time_boot_ms: unused by the firmware
        kRobotSystemId, kRobotComponentId,
        MAV_FRAME_BODY_NED, kVelocityOnly,
        0.0f, 0.0f, 0.0f,  // x, y, z
        vx, 0.0f, 0.0f,    // vx, vy, vz
        0.0f, 0.0f, 0.0f,  // afx, afy, afz
        0.0f, wz);         // yaw, yaw_rate
    return mavlink_msg_to_send_buffer(buf, &msg);
}

}  // namespace leap_bridge
