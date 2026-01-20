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

#pragma once

#include <Arduino.h>

#include "config_types.hpp"

// ============================================================================
// HARDWARE ENCODER CLASS - Using STM32 Timer Encoder Mode
// ============================================================================
class Encoder {
 public:
  Encoder() = default;

  /**
   * @brief Initialize encoder with hardware timer
   * @param pin_a Channel A pin (must be TIMx_CH1)
   * @param pin_b Channel B pin (must be TIMx_CH2)
   * @param timer Timer instance (TIM1, TIM2, TIM3, TIM4, TIM5, TIM8)
   * @param dir Direction inversion
   */
  void init(uint8_t pin_a, uint8_t pin_b, TIM_TypeDef* timer, Direction dir,
            float rad_per_tick);

  /**
   * @brief Reset encoder counter to zero
   */
  void reset();

  /**
   * @brief Update position and velocity calculations
   */
  void update();

  /**
   * @brief Get raw timer ticks
   * @return Timer ticks
   */
  const uint32_t getTicks() const { return timer_handle_->Instance->CNT; }

  /**
   * @brief Get position in radians
   * @return Position in radians
   */
  float getPosition() const { return position_; }

  /**
   * @brief Get velocity in rad/s
   * @return Velocity in rad/s
   */
  float getVelocity() const { return velocity_; }

  float lowPass(float prev, float input, float alpha) {
    float filtered = alpha * input + (1.0f - alpha) * prev;
    return filtered > MIN_VELOCITY ? filtered : 0.0f;
  }

 private:
  TIM_HandleTypeDef* timer_handle_ = nullptr;
  TIM_Encoder_InitTypeDef encoder_config_;
  float rad_per_tick_ = 0.0f;

  uint32_t last_cnt_ = 0;
  uint32_t last_time_us_ = 0;

  float position_ = 0.0f;
  float velocity_ = 0.0f;
  float last_velocity_ = 0.0f;

  // Below value are true for 16-bit timers (TIM2/TIM5 (32-bit) not supported)
  static constexpr uint32_t CNT_MAX = 0xFFFF;
  static constexpr int32_t CNT_HALF = 0x7FFF;
  static constexpr uint32_t MIN_DT_US = 100;
  static constexpr float MIN_VELOCITY = 0.01f;  // rad/s
};
