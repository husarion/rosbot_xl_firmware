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

#include "motor_drv8848.hpp"

void MotorDrv8848::init() {
  setMode(MotorMode::NEUTRAL);

  PinName pwm_pn = digitalPinToPinName(cfg_.pwm_pin);
  TIM_TypeDef* tim = (TIM_TypeDef*)pinmap_peripheral(pwm_pn, PinMap_PWM);
  pwm_channel_ = STM_PIN_CHANNEL(pinmap_function(pwm_pn, PinMap_PWM));

  pwm_timer_ = new HardwareTimer(tim);
  pwm_timer_->setPWM(pwm_channel_, pwm_pn, cfg_.pwm_freq, 0);
  pwm_arr_ = pwm_timer_->getOverflow(TICK_FORMAT);
}

MotorData MotorDrv8848::getData() const {
  MotorData d;
  if (encoder_) {
    const auto enc = encoder_->getData();
    d.position = enc.position;
    d.velocity = enc.velocity;
  }
  d.effort = current_effort_.load(std::memory_order_relaxed);
  d.target_velocity = target_velocity_.load(std::memory_order_relaxed);
  return d;
}

void MotorDrv8848::setMode(MotorMode movement) {
  if (movement == current_mode_) return;
  current_mode_ = movement;

  uint8_t pin_a = cfg_.dir_cw ? cfg_.in_b_pin : cfg_.in_a_pin;
  uint8_t pin_b = cfg_.dir_cw ? cfg_.in_a_pin : cfg_.in_b_pin;

  switch (movement) {
    case MotorMode::FORWARD:
      pinMode(pin_a, INPUT);  // Hi-Z → receives PWM
      pinMode(pin_b, OUTPUT);
      digitalWrite(pin_b, LOW);
      break;

    case MotorMode::REVERSE:
      pinMode(pin_a, OUTPUT);
      digitalWrite(pin_a, LOW);
      pinMode(pin_b, INPUT);  // Hi-Z → receives PWM
      break;

    case MotorMode::BRAKE:
      pinMode(cfg_.in_a_pin, OUTPUT);
      pinMode(cfg_.in_b_pin, OUTPUT);
      digitalWrite(cfg_.in_a_pin, HIGH);
      digitalWrite(cfg_.in_b_pin, HIGH);
      pwm_timer_->setCaptureCompare(pwm_channel_, pwm_arr_);
      break;

    case MotorMode::NEUTRAL:
    default:
      pinMode(cfg_.in_a_pin, OUTPUT);
      pinMode(cfg_.in_b_pin, OUTPUT);
      digitalWrite(cfg_.in_a_pin, LOW);
      digitalWrite(cfg_.in_b_pin, LOW);
      pwm_timer_->setCaptureCompare(pwm_channel_, 0);
      break;
  }
}

void MotorDrv8848::applyPWM(float duty) {
  duty = constrain(duty, -1.0f, 1.0f);
  current_effort_.store(duty, std::memory_order_relaxed);

  if (fabs(duty) < 0.01f) {
    setMode(MotorMode::BRAKE);
    return;
  }

  setMode(duty > 0 ? MotorMode::FORWARD : MotorMode::REVERSE);

  uint16_t pwm_value = static_cast<uint16_t>(fabs(duty) * pwm_arr_);
  pwm_timer_->setCaptureCompare(pwm_channel_, pwm_value);
}

void MotorDrv8848::setVelocity(float vel) {
  float v = constrain(vel, -cfg_.max_velocity, cfg_.max_velocity);
  if (fabs(v) < cfg_.min_velocity) v = 0.0f;
  target_velocity_.store(v, std::memory_order_relaxed);
}

void MotorDrv8848::setNeutral() {
  target_velocity_.store(0.0f, std::memory_order_relaxed);
  setMode(MotorMode::NEUTRAL);
  pid_.reset();
}

void MotorDrv8848::brake() {
  target_velocity_.store(0.0f, std::memory_order_relaxed);
  current_effort_.store(0.0f, std::memory_order_relaxed);
  setMode(MotorMode::BRAKE);
  pid_.reset();
}

void MotorDrv8848::update(float dt, bool move) {
  if (!move) {
    brake();
    return;
  }

  const float target = target_velocity_.load(std::memory_order_relaxed);
  const float current = encoder_ ? encoder_->getData().velocity : 0.0f;
  const float output = pid_.compute(target, current, dt);

  applyPWM(output);
}

void MotorDrv8848::reset() {
  brake();
  target_velocity_.store(0.0f, std::memory_order_relaxed);
  current_effort_.store(0.0f, std::memory_order_relaxed);
  pid_.reset();
  if (encoder_) encoder_->reset();
}
