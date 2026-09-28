// Copyright 2026. All rights reserved.

#ifndef ENCODER_DRIVER_H_
#define ENCODER_DRIVER_H_

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/pulse_cnt.h" // 引入 ESP-IDF v5+ 的 PCNT 驱动

class QuadratureEncoder {
 public:
  QuadratureEncoder(gpio_num_t pin_a, gpio_num_t pin_b);
  ~QuadratureEncoder();

  void Init();
  
  // 获取当前累计的绝对脉冲数。硬件计数器在 ±kCountLimit 处溢出，
  // 由 PCNT 驱动在溢出中断里累加（accum_count），这里读到的是连续值。
  int32_t GetCount() const;
  
  // 重置脉冲计数
  void ResetCount();

  bool IsInitialized() const;

 private:
  gpio_num_t pin_a_;
  gpio_num_t pin_b_;

  pcnt_unit_handle_t pcnt_unit_;
};

#endif  // ENCODER_DRIVER_H_
