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

#include "encoder_interface.hpp"
#include "motor_interface.hpp"
#include "pid.hpp"

enum MotorMode : uint8_t { FORWARD, REVERSE, BRAKE, NEUTRAL };

struct MotorDrv8848Config {
  uint8_t pwm_pin;     // PWM output
  uint8_t in_a_pin;    // direction input A
  uint8_t in_b_pin;    // direction input B
  bool dir_cw;         // polarity inversion
  float max_velocity;  // [rad/s] clamp
  float min_velocity;  // [rad/s] deadband
  uint32_t pwm_freq;   // [Hz]
  const char* frame_id;
};

/// DRV8848-based motor with Hi-Z control scheme.
/// PWM applied to nSLEEP/EN, direction via IN_A / IN_B pins.
class MotorDrv8848 : public MotorInterface {
 public:
  MotorDrv8848() = default;
  explicit MotorDrv8848(const MotorDrv8848Config& cfg,
                        EncoderInterface* encoder, PIDController pid)
      : cfg_(cfg), encoder_(encoder), pid_(pid) {}

  void init() override;
  void update(float dt, bool move = true) override;
  void reset() override;
  void setVelocity(float vel) override;
  void brake() override;
  void setNeutral() override;
  void setEnabled(bool en) override { enabled_ = en; }

  MotorData getData() const override;
  const char* name() const override { return cfg_.frame_id; }

 private:
  void setMode(MotorMode mode);
  void applyPWM(float duty);

  MotorDrv8848Config cfg_ = {};
  EncoderInterface* encoder_ = nullptr;
  PIDController pid_;
  HardwareTimer* pwm_timer_ = nullptr;
  uint32_t pwm_channel_ = 0;
  uint16_t pwm_arr_ = 0;

  std::atomic<float> target_velocity_{0.0f};
  std::atomic<float> current_effort_{0.0f};
  MotorMode current_mode_ = MotorMode::NEUTRAL;
  bool enabled_ = false;
};
