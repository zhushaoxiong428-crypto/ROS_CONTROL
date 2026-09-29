#pragma once

#include <cmath>
#include <cstdint>

// 周期任务的实时性统计（纯逻辑，可在主机上单元测试）。
// period：相邻两次唤醒的间隔；busy：一次循环体的执行耗时。
struct CycleTimingSnapshot
{
  uint32_t count = 0;
  int64_t period_min_us = 0;
  int64_t period_max_us = 0;
  float period_mean_us = 0.0f;
  float period_std_us = 0.0f;
  int64_t max_abs_jitter_us = 0; // max |period - nominal|
  float busy_mean_us = 0.0f;
  int64_t busy_max_us = 0;
  uint32_t overruns = 0; // period > nominal + overrun_margin 的次数
};

class CycleTimingStats
{
public:
  CycleTimingStats(int64_t nominal_period_us, int64_t overrun_margin_us)
      : nominal_us_(nominal_period_us), overrun_margin_us_(overrun_margin_us) {}

  void AddCycle(int64_t period_us, int64_t busy_us)
  {
    if (count_ == 0 || period_us < period_min_us_)
      period_min_us_ = period_us;
    if (count_ == 0 || period_us > period_max_us_)
      period_max_us_ = period_us;
    if (busy_us > busy_max_us_)
      busy_max_us_ = busy_us;
    const int64_t jitter = period_us > nominal_us_ ? period_us - nominal_us_ : nominal_us_ - period_us;
    if (jitter > max_abs_jitter_us_)
      max_abs_jitter_us_ = jitter;
    if (period_us > nominal_us_ + overrun_margin_us_)
      ++overruns_;
    // 以名义周期为偏移累加，避免大数相减的精度损失。
    const double deviation = static_cast<double>(period_us - nominal_us_);
    sum_dev_ += deviation;
    sum_dev_sq_ += deviation * deviation;
    sum_busy_ += static_cast<double>(busy_us);
    ++count_;
  }

  uint32_t Count() const { return count_; }

  CycleTimingSnapshot Snapshot() const
  {
    CycleTimingSnapshot s;
    s.count = count_;
    if (count_ == 0)
      return s;
    const double mean_dev = sum_dev_ / count_;
    const double variance = sum_dev_sq_ / count_ - mean_dev * mean_dev;
    s.period_min_us = period_min_us_;
    s.period_max_us = period_max_us_;
    s.period_mean_us = static_cast<float>(nominal_us_ + mean_dev);
    s.period_std_us = static_cast<float>(variance > 0.0 ? std::sqrt(variance) : 0.0);
    s.max_abs_jitter_us = max_abs_jitter_us_;
    s.busy_mean_us = static_cast<float>(sum_busy_ / count_);
    s.busy_max_us = busy_max_us_;
    s.overruns = overruns_;
    return s;
  }

  void Reset()
  {
    count_ = 0;
    period_min_us_ = period_max_us_ = busy_max_us_ = max_abs_jitter_us_ = 0;
    sum_dev_ = sum_dev_sq_ = sum_busy_ = 0.0;
    overruns_ = 0;
  }

private:
  int64_t nominal_us_;
  int64_t overrun_margin_us_;
  uint32_t count_ = 0;
  int64_t period_min_us_ = 0;
  int64_t period_max_us_ = 0;
  int64_t busy_max_us_ = 0;
  int64_t max_abs_jitter_us_ = 0;
  double sum_dev_ = 0.0;
  double sum_dev_sq_ = 0.0;
  double sum_busy_ = 0.0;
  uint32_t overruns_ = 0;
};
