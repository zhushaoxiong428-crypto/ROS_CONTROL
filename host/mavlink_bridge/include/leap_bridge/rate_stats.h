#pragma once

#include <algorithm>
#include <cstdint>

namespace leap_bridge {

// Arrival rate and inter-arrival spread of one message stream over a window,
// the Linux-side counterpart of tools/topic_rates.py for the micro-ROS path.
class RateStats {
public:
    void Add(int64_t now_ns) {
        if (last_ns_ >= 0) {
            const int64_t interval = now_ns - last_ns_;
            min_ns_ = intervals_ == 0 ? interval : std::min(min_ns_, interval);
            max_ns_ = std::max(max_ns_, interval);
            sum_ns_ += interval;
            ++intervals_;
        }
        last_ns_ = now_ns;
    }

    // Starts a new window; the last arrival carries over so the first
    // interval of the next window is not lost.
    void ResetWindow() {
        intervals_ = 0;
        sum_ns_ = 0;
        min_ns_ = 0;
        max_ns_ = 0;
    }

    uint64_t intervals() const { return intervals_; }
    double rate_hz() const { return sum_ns_ > 0 ? intervals_ * 1e9 / sum_ns_ : 0.0; }
    double mean_ms() const { return intervals_ > 0 ? sum_ns_ / 1e6 / intervals_ : 0.0; }
    double min_ms() const { return min_ns_ / 1e6; }
    double max_ms() const { return max_ns_ / 1e6; }

private:
    int64_t last_ns_ = -1;
    uint64_t intervals_ = 0;
    int64_t sum_ns_ = 0;
    int64_t min_ns_ = 0;
    int64_t max_ns_ = 0;
};

}  // namespace leap_bridge
