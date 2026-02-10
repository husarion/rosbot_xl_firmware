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

#include "bsp.hpp"

#include "log.hpp"
#include "sensors/imu.hpp"
#include "sensors/ranges.hpp"

TwoWire imu_i2c(IMU_I2C_SDA, IMU_I2C_SCL);
TwoWire range_i2c(RANGE_I2C_SDA, RANGE_I2C_SCL);

void BoardPheripheralsInit(void) {
  // Initialize Buttons
  pinMode(PUSH_BUTTON1, INPUT_PULLUP);
  pinMode(PUSH_BUTTON2, INPUT_PULLUP);

  // Initialize LEDs
  pinMode(RED_LED, OUTPUT);
  pinMode(GRN_LED, OUTPUT);
  pinMode(GRN_LED2, OUTPUT);
  digitalWrite(RED_LED, HIGH);

  // Enable power for IMU sensor
  pinMode(IMU_POWER_ON, OUTPUT);
  digitalWrite(IMU_POWER_ON, HIGH);
}
