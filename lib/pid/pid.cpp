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

#include "pid.hpp"

#include <Arduino.h>

#include <algorithm>

PIDController::PIDController(const PIDConfig& cfg) : cfg_(cfg) {
  if (cfg_.max_integral == 0.0f && cfg_.ki != 0.0f) {
    cfg_.max_integral = 1.0f / cfg_.ki;
  }
}

void PIDController::reset() {
  integral_ = 0.0f;
  prev_error_ = 0.0f;
  ramped_setpoint_ = 0.0f;
}

float PIDController::compute(float setpoint, float measurement, float dt) {
  if (dt <= 0.0f) return 0.0f;

  // ========== ACCELERATION LIMITING ==========
  float target = setpoint;
  if (cfg_.max_accel > 0.0f) {
    target = calculateRampedSetpoint(setpoint, dt);
  }

  // ========== STANDARD PID ==========
  const float error = target - measurement;

  // Proportional
  const float p = cfg_.kp * error;

  // Integral with anti-windup
  integral_ += error * dt;
  integral_ = constrain(integral_, -cfg_.max_integral, cfg_.max_integral);
  const float i = cfg_.ki * integral_;

  // Derivative
  const float derivative = (error - prev_error_) / dt;
  const float d = cfg_.kd * derivative;
  prev_error_ = error;

  float output = p + i + d;

  // Inertia compensation to overcome static friction at low speeds
  if (fabs(setpoint) > 0.01f && fabs(output) < cfg_.min_power_to_move) {
    output += ((output > 0) ? 1 : -1) * inertiaCompensation(measurement);
  }

  return constrain(output, cfg_.min_output, cfg_.max_output);
}

float PIDController::calculateRampedSetpoint(float setpoint, float dt) {
  const float max_change = cfg_.max_accel * dt;
  const float delta = setpoint - ramped_setpoint_;

  if (delta > max_change) {
    ramped_setpoint_ += max_change;
  } else if (delta < -max_change) {
    ramped_setpoint_ -= max_change;
  } else {
    ramped_setpoint_ = setpoint;
  }
  return ramped_setpoint_;
}

float PIDController::inertiaCompensation(float measurement) {
  float drive_scale =
      std::max(0.0f, 1.0f - fabs(measurement) / cfg_.compensation_up_to_speed);
  return cfg_.min_power_to_move * drive_scale;
}
