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

// ============================================================================
// MOTOR DRIVER CLASS - DRV8848PWPR Controller (Hi-Z Control Scheme)
// Thread-safe motor control with encoder feedback for FreeRTOS + micro-ROS
// ============================================================================

#include <Arduino.h>
#include <HardwareTimer.h>

#include <atomic>

#include "STM32FreeRTOS.h"
#include "encoder.hpp"
#include "pid.hpp"

enum class MotorMovement : uint8_t { FORWARD, REVERSE, BRAKE, COAST };

class SingleMotor {
 public:
  SingleMotor() = default;

  void init(uint8_t pwm_pin, uint8_t in_a_pin, uint8_t in_b_pin, Direction dir,
            Encoder& enc);

  // Control methods
  void setVelocity(const float vel);
  void stop();   // Coast stop (PWM = 0)
  void brake();  // Active braking

  // State getters - thread-safe
  float getPosition() const { return encoder_->getPosition(); }
  float getVelocity() { return encoder_->getVelocity(); }
  float getEffort() const {
    return current_effort_.load(std::memory_order_relaxed);
  }
  float getTargetVelocity() const {
    return target_velocity_.load(std::memory_order_relaxed);
  }
  void setTargetVelocity(float vel) {
    target_velocity_.store(vel, std::memory_order_relaxed);
  }

  // Control loop - call from RTOS task
  void update(float dt, bool move = true);

  // Configuration
  void resetPID() { pid_.reset(); }

 private:
  void setMovement(MotorMovement dir);
  void applyPWM(float duty);

  // Pin configuration
  uint8_t pwm_pin_ = 0;
  uint16_t pwm_arr_ = 0;
  uint8_t in_a_pin_ = 0;
  uint8_t in_b_pin_ = 0;
  Direction dir_ = Direction::CW;

  // Hardware
  Encoder* encoder_;
  PIDController pid_;
  HardwareTimer* pwm_timer_ = nullptr;
  uint32_t pwm_channel_ = 0;

  // State (atomic for thread safety)
  std::atomic<float> target_velocity_{0.0f};
  std::atomic<float> current_effort_{0.0f};

  // Current direction
  MotorMovement current_movement_ = MotorMovement::COAST;
};
