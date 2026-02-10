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

#include <Arduino.h>
#include <VL53L0X.h>
#include <Wire.h>

#include <array>
#include <vector>

#include "robot_config.hpp"

struct RangesData {
  float range[Ranges::COUNT];
  int64_t timestamp_ns = 0;
};

struct VL53L0XSensor {
  VL53L0X sensor;
  uint8_t xshutPin;
  uint8_t address;
  uint16_t lastRange;
  bool timeout;
};

class VL53L0XManager {
 public:
  VL53L0XManager(TwoWire* bus,
                 const std::array<RangeConfig, Ranges::COUNT>& configs);

  void addSensor(uint8_t xshutPin, uint8_t address = 0);
  bool init();
  void update();
  RangesData getData() const { return data_; }

  uint8_t count() const;
  VL53L0XSensor& getSensor(uint8_t index);

 private:
  RangesData data_;
  TwoWire* bus_;
  std::vector<VL53L0XSensor> sensors_;
};

extern VL53L0XManager rangeSensorsManager;
