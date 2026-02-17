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

#include "motor_array.hpp"

#include <Arduino.h>
#include <FreeRTOS.h>

MotorArray::MotorArray(MotorInterface** motors, uint8_t count,
                       const DriverGroupConfig* drivers,
                       uint8_t num_driver_groups)
    : motors_(motors),
      count_(count),
      drivers_(drivers),
      num_drivers_(num_driver_groups) {}

// ─── init ─────────────────────────────────────────────────────────

void MotorArray::init() {
  mutex_ = xSemaphoreCreateMutex();
  configASSERT(mutex_ != nullptr);

  // Setup driver control pins
  for (uint8_t i = 0; i < num_drivers_; ++i) {
    pinMode(drivers_[i].sleep_pin, OUTPUT);
    pinMode(drivers_[i].fault_pin, INPUT_PULLUP);
  }
  disableDrivers();

  // Init all motors
  data_.count = count_;
  for (uint8_t i = 0; i < count_; ++i) motors_[i]->init();

  last_update_ = millis();
  enableDrivers();
}

// ─── driver control ───────────────────────────────────────────────

void MotorArray::enableDrivers() {
  for (uint8_t i = 0; i < num_drivers_; ++i)
    digitalWrite(drivers_[i].sleep_pin, HIGH);
  drivers_enabled_.store(true, std::memory_order_relaxed);
  delayMicroseconds(100);  // DRV8848 wake-up
}

void MotorArray::disableDrivers() {
  for (uint8_t i = 0; i < num_drivers_; ++i)
    digitalWrite(drivers_[i].sleep_pin, LOW);
  drivers_enabled_.store(false, std::memory_order_relaxed);
}

bool MotorArray::checkFaults() const {
  for (uint8_t i = 0; i < num_drivers_; ++i)
    if (!digitalRead(drivers_[i].fault_pin)) return true;
  return false;
}

// ─── bulk operations (mutex-protected) ────────────────────────────

void MotorArray::setVelocities(const float* velocities) {
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    for (uint8_t i = 0; i < count_; ++i) motors_[i]->setVelocity(velocities[i]);
    feedWatchdog();
    xSemaphoreGive(mutex_);
  }
}

void MotorArray::stopAll() {
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    for (uint8_t i = 0; i < count_; ++i) motors_[i]->setNeutral();
    xSemaphoreGive(mutex_);
  }
}

void MotorArray::brakeAll() {
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
    for (uint8_t i = 0; i < count_; ++i) motors_[i]->brake();
    xSemaphoreGive(mutex_);
  }
}

// ─── update (PID loop — called from control task) ────────────────

void MotorArray::update() {
  if (!isDriversEnabled()) return;

  const uint32_t now = millis();
  const float dt = (now - last_update_) / 1000.0f;
  last_update_ = now;

  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
    for (uint8_t i = 0; i < count_; ++i) {
      motors_[i]->update(dt, !isWatchdogExpired());
      const auto d = motors_[i]->getData();
      data_.position[i] = d.position;
      data_.velocity[i] = d.velocity;
      data_.effort[i] = d.effort;
    }
    xSemaphoreGive(mutex_);
  }
}

// ─── watchdog ────────────────────────────────────────────────────

void MotorArray::feedWatchdog() {
  last_cmd_time_.store(millis(), std::memory_order_relaxed);
  watchdog_enabled_.store(true, std::memory_order_relaxed);
}

bool MotorArray::isWatchdogExpired() const {
  if (!watchdog_enabled_.load(std::memory_order_relaxed)) return false;
  return (millis() - last_cmd_time_.load(std::memory_order_relaxed)) >
         WATCHDOG_TIMEOUT_MS;
}
