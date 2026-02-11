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

#include "control/encoder.hpp"
#include "control/motor.hpp"
#include "pid.hpp"
#include "config.hpp"

class MotorDriver {
 public:
  static MotorDriver& getInstance();
  static constexpr uint32_t WATCHDOG_TIMEOUT_MS = 500;
  static constexpr uint8_t NUM_MOTORS = static_cast<uint8_t>(MotorID::COUNT);

  // Initialization
  void init();
  void enableDrivers();
  void disableDrivers();
  bool checkFaults();
  bool isDriversEnabled() const { return drivers_enabled_.load(); }

  // Motor access
  SingleMotor& getMotor(MotorID id);
  SingleMotor& operator[](MotorID id) { return getMotor(id); }

  // Bulk operations (thread-safe)
  void setVelocities(const float velocities[NUM_MOTORS]);
  void setVelocities(float fr, float rr, float rl, float fl);
  void stopAll();
  void brakeAll();

  // State retrieval
  void getPositions(float positions[NUM_MOTORS]);
  void getVelocities(float velocities[NUM_MOTORS]);
  void getEfforts(float efforts[NUM_MOTORS]);

  // Control loop
  void update();

  // Watchdog
  void feedWatchdog();
  bool isWatchdogExpired() const;
  void disableWatchdog() { watchdog_enabled_.store(false); }
  void enableWatchdog() { watchdog_enabled_.store(true); }

 private:
  MotorDriver() = default;
  MotorDriver(const MotorDriver&) = delete;
  MotorDriver& operator=(const MotorDriver&) = delete;

  SingleMotor motors_[NUM_MOTORS] = {
      {PIDController(PID_KP, PID_KI, PID_KD)},
      {PIDController(PID_KP, PID_KI, PID_KD)},
      {PIDController(PID_KP, PID_KI, PID_KD)},
      {PIDController(PID_KP, PID_KI, PID_KD)}
  };

  // Watchdog state
  std::atomic<uint32_t> last_command_time_{0};
  std::atomic<bool> watchdog_enabled_{false};
  std::atomic<bool> drivers_enabled_{false};

  uint32_t last_update_time_ = 0;

  // FreeRTOS synchronization
  SemaphoreHandle_t mutex_ = nullptr;
};

// Global accessor
inline MotorDriver& motors = MotorDriver::getInstance();
