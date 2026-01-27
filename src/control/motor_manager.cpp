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

#include <semphr.h>

#include "control/encoders_manager.hpp"
#include "control/motors_manager.hpp"
#include "robot_config.hpp"

// Global instance
MotorDriver& motors = MotorDriver::getInstance();

using namespace control;

MotorDriver& MotorDriver::getInstance() {
  static MotorDriver instance;
  return instance;
}

void MotorDriver::init() {
  // Create mutex for thread safety
  mutex_ = xSemaphoreCreateMutex();
  configASSERT(mutex_ != nullptr);

  // Setup driver control pins
  pinMode(RIGHT_WHEELS_SLEEP, OUTPUT);
  pinMode(LEFT_WHEELS_SLEEP, OUTPUT);
  pinMode(RIGHT_WHEELS_FAULT, INPUT_PULLUP);
  pinMode(LEFT_WHEELS_FAULT, INPUT_PULLUP);

  // Initially disable drivers
  disableDrivers();

  // Initialize each motor with polarity from config
  MotorID m;
  uint8_t m_idx;

  m = MotorID::FR;
  m_idx = static_cast<uint8_t>(m);
  motors_[m_idx].init(getPwmPin(m), getInAPin(m), getInBPin(m), getDirection(m),
                      encoders[m]);

  m = MotorID::RR;
  m_idx = static_cast<uint8_t>(m);
  motors_[m_idx].init(getPwmPin(m), getInAPin(m), getInBPin(m), getDirection(m),
                      encoders[m]);

  m = MotorID::RL;
  m_idx = static_cast<uint8_t>(m);
  motors_[m_idx].init(getPwmPin(m), getInAPin(m), getInBPin(m), getDirection(m),
                      encoders[m]);

  m = MotorID::FL;
  m_idx = static_cast<uint8_t>(m);
  motors_[m_idx].init(getPwmPin(m), getInAPin(m), getInBPin(m), getDirection(m),
                      encoders[m]);

  last_update_time_ = millis();
}

void MotorDriver::enableDrivers() {
  digitalWrite(RIGHT_WHEELS_SLEEP, HIGH);
  digitalWrite(LEFT_WHEELS_SLEEP, HIGH);
  drivers_enabled_.store(true, std::memory_order_relaxed);
  delayMicroseconds(100);  // DRV8848 wake-up time
}

void MotorDriver::disableDrivers() {
  digitalWrite(RIGHT_WHEELS_SLEEP, LOW);
  digitalWrite(LEFT_WHEELS_SLEEP, LOW);
  drivers_enabled_.store(false, std::memory_order_relaxed);
}

bool MotorDriver::checkFaults() {
  // FAULT pins are active LOW
  const bool fault1 = !digitalRead(RIGHT_WHEELS_FAULT);
  const bool fault2 = !digitalRead(LEFT_WHEELS_FAULT);
  return fault1 || fault2;
}

SingleMotor& MotorDriver::getMotor(MotorID id) {
  return motors_[static_cast<uint8_t>(id)];
}

void MotorDriver::setVelocities(const float velocities[NUM_MOTORS]) {
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      motors_[i].setVelocity(velocities[i]);
    }
    feedWatchdog();
    xSemaphoreGive(mutex_);
  }
}

void MotorDriver::setVelocities(float fr, float rr, float rl, float fl) {
  const float vel[NUM_MOTORS] = {fr, rr, rl, fl};
  setVelocities(vel);
}

void MotorDriver::stopAll() {
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    for (auto& motor : motors_) {
      motor.stop();
    }
    xSemaphoreGive(mutex_);
  }
}

void MotorDriver::brakeAll() {
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    for (auto& motor : motors_) {
      motor.brake();
    }
    xSemaphoreGive(mutex_);
  }
}

void MotorDriver::getPositions(float positions[NUM_MOTORS]) {
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    positions[i] = motors_[i].getPosition();
  }
}

void MotorDriver::getVelocities(float velocities[NUM_MOTORS]) {
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    velocities[i] = motors_[i].getVelocity();
  }
}

void MotorDriver::getEfforts(float efforts[NUM_MOTORS]) {
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    efforts[i] = motors_[i].getEffort();
  }
}

void MotorDriver::update() {
  if (!isDriversEnabled()) {
    return;
  }

  const uint32_t now = millis();
  const float dt = (now - last_update_time_) / 1000.0f;
  last_update_time_ = now;

  // Update all motor PID controllers
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
    for (auto& motor : motors_) {
      motor.update(dt, !isWatchdogExpired());
    }
    xSemaphoreGive(mutex_);
  }
}

void MotorDriver::feedWatchdog() {
  last_command_time_.store(millis(), std::memory_order_relaxed);
  watchdog_enabled_.store(true, std::memory_order_relaxed);
}

bool MotorDriver::isWatchdogExpired() const {
  if (!watchdog_enabled_.load(std::memory_order_relaxed)) {
    return false;
  }
  const uint32_t now = millis();
  const uint32_t last = last_command_time_.load(std::memory_order_relaxed);
  return (now - last) > WATCHDOG_TIMEOUT_MS;
}
