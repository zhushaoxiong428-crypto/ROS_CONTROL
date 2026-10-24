// Copyright 2026. All rights reserved.

#include "encoder_driver.h"

#include <cstdlib>

#include "esp_log.h"
#include "rom/ets_sys.h"

static const char* TAG = "ENCODER_PCNT";

// 硬件计数范围 [-kPcntLimit, kPcntLimit]，到达上下限时硬件清零，
// 由驱动在中断里把 ±kPcntLimit 累加进 32 位计数 (accum_count)。
static constexpr int kPcntLimit = 30000;
// 两次读数间隔 20 ms，轮子不可能转过半个量程 (约 3 圈)；超过即视为
// 读到了“硬件已清零、中断尚未累加”的瞬间，稍等后重读。
static constexpr int32_t kImplausibleDelta = kPcntLimit / 2;
static constexpr uint32_t kOverflowSettleUs = 20;

QuadratureEncoder::QuadratureEncoder(gpio_num_t pin_a, gpio_num_t pin_b)
    : pin_a_(pin_a),
      pin_b_(pin_b),
      pcnt_unit_(nullptr),
      last_count_(0) {}

QuadratureEncoder::~QuadratureEncoder() {
  if (pcnt_unit_) {
    pcnt_unit_stop(pcnt_unit_);
    pcnt_unit_disable(pcnt_unit_);
    pcnt_del_unit(pcnt_unit_);
  }
}

void QuadratureEncoder::Init() {
  if (pin_a_ == GPIO_NUM_NC || pin_b_ == GPIO_NUM_NC) {
    ESP_LOGI(TAG, "Skipping PCNT init on pins %d/%d because encoder is not populated", pin_a_, pin_b_);
    pcnt_unit_ = nullptr;
    last_count_ = 0;
    return;
  }

  ESP_LOGI(TAG, "Initializing PCNT on pins %d and %d", pin_a_, pin_b_);

  // 1. 配置 PCNT 单元 (Unit)
  pcnt_unit_config_t unit_config = {};
  unit_config.low_limit = -kPcntLimit;
  unit_config.high_limit = kPcntLimit;
  unit_config.flags.accum_count = true;
  
  ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit_));

  // 2. 设置硬件毛刺滤波器 (极大提升抗干扰能力)
  pcnt_glitch_filter_config_t filter_config = {};
  filter_config.max_glitch_ns = 1000; // 过滤 1微秒 (1000ns) 以下的短跳变
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit_, &filter_config));

  // 3. 实例化两个通道 (Channel A 和 Channel B)
  pcnt_chan_config_t chan_a_config = {};
  chan_a_config.edge_gpio_num = pin_a_;
  chan_a_config.level_gpio_num = pin_b_;
  pcnt_channel_handle_t pcnt_chan_a = nullptr;
  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_, &chan_a_config, &pcnt_chan_a));

  pcnt_chan_config_t chan_b_config = {};
  chan_b_config.edge_gpio_num = pin_b_;
  chan_b_config.level_gpio_num = pin_a_;
  pcnt_channel_handle_t pcnt_chan_b = nullptr;
  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_, &chan_b_config, &pcnt_chan_b));

  // 4. 设置硬件四倍频 (4X Quadrature) 逻辑
  // A相：当A跳变时，根据B相的电平决定加还是减
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, 
      PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, 
      PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  // B相：当B跳变时，根据A相的电平决定加还是减
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, 
      PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, 
      PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  // 5. accum_count 只在上下限观察点触发时累加，必须注册这两个观察点
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_, kPcntLimit));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit_, -kPcntLimit));

  // 6. 启用、清零并启动 PCNT 硬件
  ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_));
  ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_));
}

int32_t QuadratureEncoder::GetCount() {
  if (pcnt_unit_ == nullptr) {
    return 0;
  }

  int count = 0;
  pcnt_unit_get_count(pcnt_unit_, &count);
  if (std::abs(count - last_count_) > kImplausibleDelta) {
    ets_delay_us(kOverflowSettleUs);
    pcnt_unit_get_count(pcnt_unit_, &count);
  }
  last_count_ = count;
  return count;
}

void QuadratureEncoder::ResetCount() {
  if (pcnt_unit_ == nullptr) {
    last_count_ = 0;
    return;
  }

  pcnt_unit_clear_count(pcnt_unit_);
  last_count_ = 0;
}

bool QuadratureEncoder::IsInitialized() const {
  return pcnt_unit_ != nullptr;
}
