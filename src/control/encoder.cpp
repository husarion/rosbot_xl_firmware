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

#include "control/encoder.hpp"

#include "config.hpp"

static TIM_HandleTypeDef htim_encoders[4];
static uint8_t encoder_count = 0;

void Encoder::init(uint8_t pin_a, uint8_t pin_b, TIM_TypeDef* timer,
                   Direction dir, float rad_per_tick) {
  timer_handle_ = &htim_encoders[encoder_count++];
  rad_per_tick_ = rad_per_tick;

  // Enable timer clock
  if (timer == TIM1)
    __HAL_RCC_TIM1_CLK_ENABLE();
  else if (timer == TIM2)
    __HAL_RCC_TIM2_CLK_ENABLE();
  else if (timer == TIM3)
    __HAL_RCC_TIM3_CLK_ENABLE();
  else if (timer == TIM4)
    __HAL_RCC_TIM4_CLK_ENABLE();
  else if (timer == TIM5)
    __HAL_RCC_TIM5_CLK_ENABLE();
  else if (timer == TIM8)
    __HAL_RCC_TIM8_CLK_ENABLE();

  // Configure GPIO pins
  GPIO_InitTypeDef gpio = {0};
  GPIO_TypeDef* port_a = digitalPinToPort(pin_a);
  GPIO_TypeDef* port_b = digitalPinToPort(pin_b);

  // Enable GPIO clocks
  if (port_a == GPIOA || port_b == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
  if (port_a == GPIOB || port_b == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
  if (port_a == GPIOC || port_b == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();

  // Alternate function
  uint8_t af = GPIO_AF2_TIM3;
  if (timer == TIM1 || timer == TIM2)
    af = GPIO_AF1_TIM1;
  else if (timer == TIM8)
    af = GPIO_AF3_TIM8;

  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = af;

  gpio.Pin = digitalPinToBitMask(pin_a);
  HAL_GPIO_Init(port_a, &gpio);

  gpio.Pin = digitalPinToBitMask(pin_b);
  HAL_GPIO_Init(port_b, &gpio);

  // Timer config
  timer_handle_->Instance = timer;
  timer_handle_->Init.Prescaler = 0;
  timer_handle_->Init.CounterMode = TIM_COUNTERMODE_UP;
  timer_handle_->Init.Period = CNT_MAX;
  timer_handle_->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  timer_handle_->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  // Encoder mode x4
  encoder_config_.EncoderMode = TIM_ENCODERMODE_TI12;
  encoder_config_.IC1Polarity =
      (dir == Direction::CW) ? TIM_ICPOLARITY_RISING : TIM_ICPOLARITY_FALLING;
  encoder_config_.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  encoder_config_.IC1Prescaler = TIM_ICPSC_DIV1;
  encoder_config_.IC1Filter = 0x0F;
  encoder_config_.IC2Polarity = TIM_ICPOLARITY_RISING;
  encoder_config_.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  encoder_config_.IC2Prescaler = TIM_ICPSC_DIV1;
  encoder_config_.IC2Filter = 0x0F;

  HAL_TIM_Encoder_Init(timer_handle_, &encoder_config_);
  HAL_TIM_Encoder_Start(timer_handle_, TIM_CHANNEL_ALL);

  reset();
}

void Encoder::reset() {
  __HAL_TIM_SET_COUNTER(timer_handle_, 0);
  last_cnt_ = 0;
  last_time_us_ = micros();
  position_ = 0.0f;
  velocity_ = 0.0f;
  last_velocity_ = 0.0f;
}

void Encoder::update() {
  const uint32_t now = micros();
  const uint32_t dt = now - last_time_us_;

  if (dt >= MIN_DT_US) {
    const uint32_t cnt = getTicks();
    int32_t delta = static_cast<int32_t>(cnt - last_cnt_);

    if (delta > CNT_HALF)
      delta -= (CNT_MAX + 1);
    else if (delta < -CNT_HALF)
      delta += (CNT_MAX + 1);

    float delta_position = static_cast<float>(delta) * rad_per_tick_;
    position_ += delta_position;
    velocity_ = (delta_position * 1000000.0f) / static_cast<float>(dt);
    velocity_ = lowPass(last_velocity_, velocity_, 0.1f);
    last_velocity_ = velocity_;

    last_cnt_ = cnt;
    last_time_us_ = now;
  }
}
