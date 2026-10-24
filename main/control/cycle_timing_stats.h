#pragma once

#include <stdint.h>

#include <algorithm>
#include <cmath>

// Running statistics of a periodic loop. Periods and busy times are in
// microseconds; sums are integers so the hot path avoids double math.
class CycleTimingStats final {
 public:
  struct Summary {
    uint32_t count;
    float period_mean_us;
    float period_std_us;
    int64_t period_min_us;
    int64_t period_max_us;
    int64_t max_abs_jitter_us;  // largest |period - nominal|
    float busy_mean_us;
    int64_t busy_max_us;
    uint32_t overruns;          // periods longer than nominal + margin
  };

  CycleTimingStats(int64_t nominal_period_us, int64_t overrun_margin_us)
      : nominal_us_(nominal_period_us), overrun_margin_us_(overrun_margin_us) {}

  void Add(int64_t period_us, int64_t busy_us) {
    if (count_ == 0) {
      period_min_us_ = period_max_us_ = period_us;
    }
    ++count_;
    period_sum_ += period_us;
    period_sq_sum_ += period_us * period_us;
    period_min_us_ = std::min(period_min_us_, period_us);
    period_max_us_ = std::max(period_max_us_, period_us);
    const int64_t jitter = period_us > nominal_us_ ? period_us - nominal_us_ : nominal_us_ - period_us;
    max_abs_jitter_us_ = std::max(max_abs_jitter_us_, jitter);
    busy_sum_ += busy_us;
    busy_max_us_ = std::max(busy_max_us_, busy_us);
    if (period_us > nominal_us_ + overrun_margin_us_) {
      ++overruns_;
    }
  }

  uint32_t count() const { return count_; }

  Summary Get() const {
    Summary s = {};
    s.count = count_;
    if (count_ == 0) {
      return s;
    }
    const double n = static_cast<double>(count_);
    const double mean = static_cast<double>(period_sum_) / n;
    const double variance = static_cast<double>(period_sq_sum_) / n - mean * mean;
    s.period_mean_us = static_cast<float>(mean);
    s.period_std_us = static_cast<float>(std::sqrt(std::max(variance, 0.0)));
    s.period_min_us = period_min_us_;
    s.period_max_us = period_max_us_;
    s.max_abs_jitter_us = max_abs_jitter_us_;
    s.busy_mean_us = static_cast<float>(static_cast<double>(busy_sum_) / n);
    s.busy_max_us = busy_max_us_;
    s.overruns = overruns_;
    return s;
  }

  void Reset() { *this = CycleTimingStats(nominal_us_, overrun_margin_us_); }

 private:
  int64_t nominal_us_;
  int64_t overrun_margin_us_;
  uint32_t count_ = 0;
  int64_t period_sum_ = 0;
  int64_t period_sq_sum_ = 0;
  int64_t period_min_us_ = 0;
  int64_t period_max_us_ = 0;
  int64_t max_abs_jitter_us_ = 0;
  int64_t busy_sum_ = 0;
  int64_t busy_max_us_ = 0;
  uint32_t overruns_ = 0;
};

// Loop dt from a measured period, limited so one stalled cycle (or the first
// cycle) cannot produce a wild velocity estimate.
inline float ClampLoopDtS(int64_t period_us, int64_t nominal_us) {
  const int64_t clamped = std::clamp(period_us, nominal_us / 2, nominal_us * 2);
  return static_cast<float>(clamped) * 1e-6f;
}
