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
// PID CONTROLLER - Optimized for real-time control
// ============================================================================

class PIDController {
 public:
  static constexpr float DEFAULT_KP = 0.07f;
  static constexpr float DEFAULT_KI = 0.4f;
  static constexpr float DEFAULT_KD = 0.002f;
  static constexpr float DEFAULT_MAX_ACCEL = 0.0f;  // rad/s² 0 = disabled
  static constexpr float DEFAULT_MIN_OUTPUT = -1.0f;
  static constexpr float DEFAULT_MAX_OUTPUT = 1.0f;

  PIDController(float kp = DEFAULT_KP, float ki = DEFAULT_KI,
                float kd = DEFAULT_KD);

  void setMaxAccel(float max_accel);
  void setGains(float kp, float ki, float kd);
  void setLimits(float min_output, float max_output);
  void setMaxIntegral(float max_integral);
  void reset();

  float compute(float setpoint, float measurement, float dt, float min_output = 0.0f);

 private:
  float kp_, ki_, kd_;
  float min_output_ = DEFAULT_MIN_OUTPUT;
  float max_output_ = DEFAULT_MAX_OUTPUT;
  float max_integral_ = 1.0f / DEFAULT_KI;
  float max_accel_ = DEFAULT_MAX_ACCEL;

  float integral_ = 0.0f;
  float prev_error_ = 0.0f;
  float ramped_setpoint_ = 0.0f;
};
