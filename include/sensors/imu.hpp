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

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

struct ImuData {
  float orientation[4];       // quaternion: x y z w
  float angular_velocity[3];  // rad/s
  float acceleration[3];      // m/s^2
  int64_t timestamp_ns;       // timestamp przy odczycie
};

class ImuDriver {
 public:
  bool init(uint8_t id, uint8_t addr, TwoWire* wire);
  void update();
  ImuData getData() const { return data_; }
 private:
  ImuData data_;
  Adafruit_BNO055* imuBno_ = nullptr;
  sensors_event_t event_;
};

inline ImuDriver imuDriver;
