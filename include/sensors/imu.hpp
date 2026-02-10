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

#include "robot_config.hpp"

struct ImuData {
  float orientation[4];       // quaternion: x y z w
  float angular_velocity[3];  // rad/s
  float acceleration[3];      // m/s^2
  int64_t timestamp_ns;       // timestamp przy odczycie
};

class ImuDriver {
 public:
  ImuDriver(TwoWire* bus) : bus_(bus) {}
  bool init(uint8_t id, uint8_t addr);
  void update();
  ImuData getData() const { return data_; }

 private:
  TwoWire* bus_;
  ImuData data_;
  Adafruit_BNO055* imuBno_ = nullptr;
  sensors_event_t event_;
};

inline TwoWire imu_i2c(IMU_I2C_SDA, IMU_I2C_SCL);
inline ImuDriver imuDriver(&imu_i2c);
