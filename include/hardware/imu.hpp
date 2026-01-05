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
#include <Wire.h>

#if defined(ROSBOT)
#define ROBOT_IMU_AXIS_CONFIG Adafruit_BNO055::REMAP_CONFIG_P0
#elif defined(ROSBOT_XL)
#define ROBOT_IMU_AXIS_CONFIG Adafruit_BNO055::REMAP_CONFIG_P1
#endif

typedef struct {
  float orientation[4];       // quaternion: x y z w
  float angular_velocity[3];  // rad/s
  float acceleration[3];      // m/s^2
} imu_data_t;

class ImuDriver {
 public:
  ImuDriver(uint8_t ImuId_, uint8_t ImuAddr_, TwoWire* ImuWire_);
  ~ImuDriver();
  bool init();
  imu_data_t loopHandler();

 private:
  Adafruit_BNO055* imuBno;
  sensors_event_t event;
};

extern ImuDriver imuDriver;
