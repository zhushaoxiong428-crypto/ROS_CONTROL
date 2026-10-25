#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/ledc.h"

class At8236Motor {
 public:
  At8236Motor(gpio_num_t in1_pin, gpio_num_t in2_pin);

  void Init();
  // Signed duty in [-1, 1]: positive drives IN1, negative IN2, 0 coasts.
  // Quantized by the 10-bit LEDC timer (1024 steps per direction).
  void SetDuty(float duty);
  void Brake();

 private:
  gpio_num_t in1_pin_;
  gpio_num_t in2_pin_;
  ledc_channel_t channel_in1_;
  ledc_channel_t channel_in2_;
  bool initialized_;
  bool usable_;
};

#endif  // MOTOR_DRIVER_H_
