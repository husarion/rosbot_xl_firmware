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

#include <stdint.h>

struct MotorData {
  float position = 0.0f;         // [rad]
  float velocity = 0.0f;         // [rad/s]
  float effort = 0.0f;           // [Nm]
  float target_velocity = 0.0f;  // [rad/s]
};

class MotorInterface {
 public:
  virtual ~MotorInterface() = default;

  virtual void init() = 0;
  virtual void update(float dt, bool move = true) = 0;
  virtual void reset() = 0;
  virtual void setVelocity(float vel) = 0;

  virtual void brake() = 0;
  virtual void setNeutral() = 0;

  virtual void setEnabled(bool en) = 0;
  virtual MotorData getData() const { return data_; }

  virtual const char* name() const = 0;

 protected:
  MotorData data_ = {};
};
