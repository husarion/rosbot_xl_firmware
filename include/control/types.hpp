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

#include <array>

// ================= IDENTIFICATION =================
enum class MotorID : uint8_t { FR = 0, RR = 1, RL = 2, FL = 3, COUNT = 4 };
enum class Direction : bool { CW = false, CCW = true };

// ================= STRUCTURES =================
struct MotorPins {
  uint8_t pwm;
  uint8_t in_a;
  uint8_t in_b;
  uint8_t enc_a;
  uint8_t enc_b;
};

struct MotorConfig {
  MotorID id;
  MotorPins pins;
  TIM_TypeDef* encoder_timer;
  Direction direction;
  const char* joint_name;
};

namespace control {

// ================= CONFIG DECLARATION =================
extern const std::array<MotorConfig, static_cast<size_t>(MotorID::COUNT)>
    CONFIG;

// ================= GETTERS =================
constexpr const MotorConfig& getConfig(MotorID id) {
  return CONFIG[static_cast<size_t>(id)];
}

constexpr uint8_t getPwmPin(MotorID id) { return getConfig(id).pins.pwm; }

constexpr uint8_t getInAPin(MotorID id) { return getConfig(id).pins.in_a; }

constexpr uint8_t getInBPin(MotorID id) { return getConfig(id).pins.in_b; }

constexpr uint8_t getEncAPin(MotorID id) { return getConfig(id).pins.enc_a; }

constexpr uint8_t getEncBPin(MotorID id) { return getConfig(id).pins.enc_b; }

constexpr TIM_TypeDef* getEncoderTimer(MotorID id) {
  return getConfig(id).encoder_timer;
}

constexpr Direction getDirection(MotorID id) { return getConfig(id).direction; }

constexpr const char* getJointName(MotorID id) {
  return getConfig(id).joint_name;
}

}  // namespace motors

