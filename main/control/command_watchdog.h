#pragma once

#include <stdint.h>

// Velocity commands are streamed; if the sender stops publishing, the last
// command must not keep the robot moving. Timestamps are free-running
// milliseconds and may wrap around.
class CommandWatchdog final {
 public:
  explicit CommandWatchdog(uint32_t timeout_ms) : timeout_ms_(timeout_ms) {}

  void Feed(uint32_t now_ms) {
    last_feed_ms_ = now_ms;
    armed_ = true;
  }

  void Disarm() { armed_ = false; }

  bool IsArmed() const { return armed_; }

  // True only when a command was fed and is not older than the timeout.
  bool IsFresh(uint32_t now_ms) const {
    return armed_ && static_cast<uint32_t>(now_ms - last_feed_ms_) <= timeout_ms_;
  }

 private:
  uint32_t timeout_ms_;
  uint32_t last_feed_ms_ = 0;
  bool armed_ = false;
};

// Modes 0 (body velocity), 3 (single wheel) and 6 (both wheels) hold a speed
// until the next command, so they need the watchdog. Position and relative
// moves finish on their own.
inline bool IsStreamingMotionMode(int control_mode) {
  return control_mode == 0 || control_mode == 3 || control_mode == 6;
}
