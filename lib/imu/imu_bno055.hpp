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
#include <Wire.h>

#include "imu_interface.hpp"

struct ImuBno055Config {
  TwoWire* bus;
  uint8_t i2c_addr;
  int32_t sensor_id;
  uint16_t int_pin;
  Adafruit_BNO055::adafruit_bno055_axis_remap_config_t axis_config;
};

class ImuBno055 : public ImuInterface {
 public:
  explicit ImuBno055(const ImuBno055Config& cfg);

  bool init() override;
  void update() override;
  const char* name() const override { return "BNO055"; }

 private:
  ImuBno055Config cfg_;
  Adafruit_BNO055 bno_;
};
