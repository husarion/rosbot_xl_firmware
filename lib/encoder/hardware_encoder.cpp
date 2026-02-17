// Copyright 2022 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hardware_encoder.hpp"

#include <cassert>

static TIM_HandleTypeDef s_htim_pool[4];
static uint8_t s_htim_count = 0;

// ---------- helpers ------------------------------------------------

static void enableTimerClock(TIM_TypeDef* tim) {
  if (tim == TIM1)
    __HAL_RCC_TIM1_CLK_ENABLE();
  else if (tim == TIM2)
    __HAL_RCC_TIM2_CLK_ENABLE();
  else if (tim == TIM3)
    __HAL_RCC_TIM3_CLK_ENABLE();
  else if (tim == TIM4)
    __HAL_RCC_TIM4_CLK_ENABLE();
  else if (tim == TIM5)
    __HAL_RCC_TIM5_CLK_ENABLE();
  else if (tim == TIM8)
    __HAL_RCC_TIM8_CLK_ENABLE();
  else
    assert(!"Invalid timer instance");
}

static void enableGpioClock(GPIO_TypeDef* port) {
  if (port == GPIOA)
    __HAL_RCC_GPIOA_CLK_ENABLE();
  else if (port == GPIOB)
    __HAL_RCC_GPIOB_CLK_ENABLE();
  else if (port == GPIOC)
    __HAL_RCC_GPIOC_CLK_ENABLE();
  else if (port == GPIOD)
    __HAL_RCC_GPIOD_CLK_ENABLE();
}

static uint8_t timerAF(TIM_TypeDef* tim) {
  if (tim == TIM1 || tim == TIM2) return GPIO_AF1_TIM1;

  if (tim == TIM3 || tim == TIM4 || tim == TIM5) return GPIO_AF2_TIM3;

  if (tim == TIM8) return GPIO_AF3_TIM8;

  // TIM6, TIM7 no AF
  assert(!"Timer does not support GPIO Alternate Function");
}

HardwareEncoder::HardwareEncoder(const HardwareEncoderConfig& cfg)
    : cfg_(cfg) {}

void HardwareEncoder::init() {
  timer_handle_ = &s_htim_pool[s_htim_count++];

  enableTimerClock(cfg_.timer);

  // GPIO setup
  GPIO_TypeDef* port_a = digitalPinToPort(cfg_.pin_a);
  GPIO_TypeDef* port_b = digitalPinToPort(cfg_.pin_b);
  enableGpioClock(port_a);
  enableGpioClock(port_b);

  GPIO_InitTypeDef gpio = {};
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = timerAF(cfg_.timer);

  gpio.Pin = digitalPinToBitMask(cfg_.pin_a);
  HAL_GPIO_Init(port_a, &gpio);

  gpio.Pin = digitalPinToBitMask(cfg_.pin_b);
  HAL_GPIO_Init(port_b, &gpio);

  // Timer config
  timer_handle_->Instance = cfg_.timer;
  timer_handle_->Init.Prescaler = 0;
  timer_handle_->Init.CounterMode = TIM_COUNTERMODE_UP;
  timer_handle_->Init.Period = CNT_MAX;
  timer_handle_->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  timer_handle_->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  // Encoder mode x4
  encoder_cfg_.EncoderMode = TIM_ENCODERMODE_TI12;
  encoder_cfg_.IC1Polarity =
      (cfg_.dir_cw) ? TIM_ICPOLARITY_RISING : TIM_ICPOLARITY_FALLING;
  encoder_cfg_.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  encoder_cfg_.IC1Prescaler = TIM_ICPSC_DIV1;
  encoder_cfg_.IC1Filter = 0x0F;
  encoder_cfg_.IC2Polarity = TIM_ICPOLARITY_RISING;
  encoder_cfg_.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  encoder_cfg_.IC2Prescaler = TIM_ICPSC_DIV1;
  encoder_cfg_.IC2Filter = 0x0F;

  HAL_TIM_Encoder_Init(timer_handle_, &encoder_cfg_);
  HAL_TIM_Encoder_Start(timer_handle_, TIM_CHANNEL_ALL);

  reset();
}

void HardwareEncoder::reset() {
  __HAL_TIM_SET_COUNTER(timer_handle_, 0);
  last_cnt_ = 0;
  last_time_us_ = micros();
  data_ = {};
  last_velocity_ = 0.0f;
}

void HardwareEncoder::update() {
  const uint32_t now = micros();
  const uint32_t dt_us = now - last_time_us_;

  if (dt_us >= MIN_DT_US) {
    const uint32_t cnt = getTicks();
    int32_t delta = compute_delta(cnt, last_cnt_);

    float delta_pos = static_cast<float>(delta) * cfg_.rad_per_tick;
    data_.position += delta_pos;

    float dt = static_cast<float>(dt_us) * US_TO_SEC;
    float vel = delta_pos / dt;
    data_.velocity = lowPass(last_velocity_, vel, 0.1f);

    last_velocity_ = data_.velocity;
    last_cnt_ = cnt;
    last_time_us_ = now;
  }
}

inline int32_t HardwareEncoder::compute_delta(uint32_t cnt, uint32_t last_cnt) {
  int32_t delta = static_cast<int32_t>(cnt - last_cnt);

  // Wrap-around detection (dla 16-bit timer)
  if (delta > CNT_HALF)
    delta -= (CNT_MAX + 1);
  else if (delta < -CNT_HALF)
    delta += (CNT_MAX + 1);

  return delta;
}
