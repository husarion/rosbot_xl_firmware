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

#include "control/motor.hpp"

#include <HardwareTimer.h>

#include "robot_config.hpp"

// ============================================================================
// SINGLE MOTOR IMPLEMENTATION - Hi-Z Control
// ============================================================================

void SingleMotor::init(uint8_t pwm_pin, uint8_t in_a_pin, uint8_t in_b_pin,
                        Direction dir, Encoder& enc) {
  pwm_pin_ = pwm_pin;
  in_a_pin_ = in_a_pin;
  in_b_pin_ = in_b_pin;
  dir_ = dir;

  // Initialize direction pins to COAST (both Hi-Z)
  setMovement(MotorMovement::COAST);

  // Initialize PWM timer
  PinName pwm_pin_name = digitalPinToPinName(pwm_pin);
  TIM_TypeDef* timer_instance =
      (TIM_TypeDef*)pinmap_peripheral(pwm_pin_name, PinMap_PWM);
  pwm_channel_ = STM_PIN_CHANNEL(pinmap_function(pwm_pin_name, PinMap_PWM));

  pwm_timer_ = new HardwareTimer(timer_instance);
  pwm_timer_->setPWM(pwm_channel_, pwm_pin_name, ControlParams::MOTOR_PWM_FREQ,
                     0);
  pwm_arr_ = pwm_timer_->getOverflow(TICK_FORMAT);

  // Initialize encoder
  encoder_ = &enc;

  // Initialize PID
  pid_.setLimits(-1.0f, 1.0f);
}

void SingleMotor::setMovement(MotorMovement movement) {
  // Skip if direction unchanged
  if (movement == current_movement_) return;

  current_movement_ = movement;

  uint8_t pin_a, pin_b;
  if (dir_ == Direction::CCW) {
    pin_a = in_a_pin_;
    pin_b = in_b_pin_;
  } else {
    pin_a = in_b_pin_;
    pin_b = in_a_pin_;
  }

  switch (movement) {
    case MotorMovement::FORWARD:
      // IN_A = Hi-Z (receives PWM), IN_B = GND
      pinMode(pin_a, INPUT);
      pinMode(pin_b, OUTPUT);
      digitalWrite(pin_b, LOW);
      break;

    case MotorMovement::REVERSE:
      // IN_A = GND, IN_B = Hi-Z (receives PWM)
      pinMode(pin_a, OUTPUT);
      digitalWrite(pin_a, LOW);
      pinMode(pin_b, INPUT);
      break;

    case MotorMovement::BRAKE:
      // Both pins HIGH - active braking
      pinMode(in_a_pin_, OUTPUT);
      pinMode(in_b_pin_, OUTPUT);
      digitalWrite(in_a_pin_, HIGH);
      digitalWrite(in_b_pin_, HIGH);
      pwm_timer_->setCaptureCompare(pwm_channel_, pwm_arr_);
      break;

    case MotorMovement::COAST:
    default:
      // Both pins LOW - motor coasts
      pinMode(in_a_pin_, OUTPUT);
      pinMode(in_b_pin_, OUTPUT);
      digitalWrite(in_a_pin_, LOW);
      digitalWrite(in_b_pin_, LOW);
      pwm_timer_->setCaptureCompare(pwm_channel_, 0);
      break;
  }
}

void SingleMotor::applyPWM(float duty) {
  duty = constrain(duty, -1.0f, 1.0f);
  current_effort_.store(duty, std::memory_order_relaxed);

  // Apply deadband
  const float abs_duty = fabs(duty);
  if (abs_duty < 0.01f) {
    setMovement(MotorMovement::BRAKE);
    return;
  }

  // Set direction based on sign
  if (duty > 0) {
    setMovement(MotorMovement::FORWARD);
  } else {
    setMovement(MotorMovement::REVERSE);
  }

  // Apply PWM value
  uint16_t pwm_value = static_cast<uint16_t>(abs_duty * pwm_arr_);
  pwm_timer_->setCaptureCompare(pwm_channel_, pwm_value);
}

void SingleMotor::setVelocity(const float vel) {
  // Apply inversion
  target_velocity_.store(vel, std::memory_order_relaxed);
}

void SingleMotor::stop() {
  target_velocity_.store(0.0f, std::memory_order_relaxed);

  setMovement(MotorMovement::COAST);

  pid_.reset();
}

void SingleMotor::brake() {
  target_velocity_.store(0.0f, std::memory_order_relaxed);
  current_effort_.store(0.0f, std::memory_order_relaxed);

  setMovement(MotorMovement::BRAKE);

  pid_.reset();
}

void SingleMotor::update(float dt, bool move) {

  if (!move) {
    brake();
    return;
  }

  const float target = target_velocity_.load(std::memory_order_relaxed);
  const float current = getVelocity();
  const float output = pid_.compute(target, current, dt);

  applyPWM(output);
}
