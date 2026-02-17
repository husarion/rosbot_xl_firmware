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

struct PIDConfig {
  float kp;
  float ki;
  float kd;
  float min_output = -1.0f;        // Default output limits
  float max_output = 1.0f;         // Default output limits
  float min_power_to_move = 0.0f;  // Minimum output to overcome
  float compensation_up_to_speed =
      2.0f;  // Speed up to which inertia compensation is applied
  float max_integral =
      0.0f;  // Will be set to 1/ki if ki > 0, otherwise no integral action
  float max_accel = 0.0f;  // No acceleration limit by default
};

class PIDController {
 public:
  PIDController(const PIDConfig& cfg);

  float compute(float setpoint, float measurement, float dt);
  void reset();

 private:
  float calculateRampedSetpoint(float setpoint, float dt);
  float inertiaCompensation(float measurement);

  PIDConfig cfg_;
  float integral_ = 0.0f;
  float prev_error_ = 0.0f;
  float ramped_setpoint_ = 0.0f;
};
