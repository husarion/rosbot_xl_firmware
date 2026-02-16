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

class PIDController {
 public:
  PIDController(float kp, float ki, float kd, float min_output = -1.0f,
                float max_output = 1.0f);

  void setMaxAccel(float max_accel);
  void setGains(float kp, float ki, float kd);
  void setLimits(float min_output, float max_output);
  void setMaxIntegral(float max_integral);
  void reset();

  float compute(float setpoint, float measurement, float dt,
                float min_output = 0.0f);

 private:
  float kp_, ki_, kd_;
  float min_output_, max_output_;
  float max_integral_;

  float integral_ = 0.0f;
  float prev_error_ = 0.0f;

  float max_accel_ = 0.0f;  // No acceleration limit by default
  float ramped_setpoint_ = 0.0f;
};
