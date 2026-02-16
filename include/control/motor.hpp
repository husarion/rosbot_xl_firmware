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
#include <HardwareTimer.h>

#include <atomic>

#include "STM32FreeRTOS.h"
#include "encoder_array.hpp"
#include "encoder_interface.hpp"
#include "pid.hpp"

enum MotorMode : uint8_t { FORWARD, REVERSE, BRAKE, NEUTRAL };

// ============================================================================
// MOTOR DRIVER CLASS for DRV8848PWPR Controller (Hi-Z Control Scheme)
// ============================================================================
class SingleMotor {
 public:
  SingleMotor(PIDController pid) : pid_(pid) {};

  void init(uint8_t pwm_pin, uint8_t in_a_pin, uint8_t in_b_pin, bool dir_cw,
            EncoderInterface* enc);

  // Control methods
  void brake();
  void setEnabled(const bool en) { enabled_ = en; }
  void setNeutral();
  void setVelocity(const float vel);

  // Getters
  float getPosition() const { return encoder_->getData().position; }
  float getVelocity() const { return encoder_->getData().velocity; }
  float getEffort() const {
    return current_effort_.load(std::memory_order_relaxed);
  }
  float getTargetVelocity() const {
    return target_velocity_.load(std::memory_order_relaxed);
  }

  void update(float dt, bool move = true);
  void reset();

 private:
  void setMode(MotorMode dir);
  void applyPWM(float duty);

  // Pin mode configuration
  uint8_t pwm_pin_ = 0;
  uint16_t pwm_arr_ = 0;
  uint8_t in_a_pin_ = 0;
  uint8_t in_b_pin_ = 0;
  bool dir_cw_ = true;

  // Hardware
  EncoderInterface* encoder_;
  PIDController pid_;
  HardwareTimer* pwm_timer_ = nullptr;
  uint32_t pwm_channel_ = 0;

  // State
  std::atomic<float> target_velocity_{0.0f};
  std::atomic<float> current_effort_{0.0f};
  MotorMode current_mode_ = MotorMode::NEUTRAL;
  bool enabled_ = false;
};
