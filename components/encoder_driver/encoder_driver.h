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
  
  // 获取当前累计的绝对脉冲数 (32位无溢出)
  int32_t GetCount();
  
  // 重置脉冲计数
  void ResetCount();

  bool IsInitialized() const;

 private:
  gpio_num_t pin_a_;
  gpio_num_t pin_b_;

  pcnt_unit_handle_t pcnt_unit_;

  // 上一次的读数，用于识别溢出补偿尚未完成时的错误读数
  int32_t last_count_;
};

#endif  // ENCODER_DRIVER_H_
