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

#include "sensors/ranges.hpp"

#include <Arduino.h>

#include <vector>

VL53L0XManager::VL53L0XManager(
    TwoWire* bus, const std::array<RangeConfig, Ranges::COUNT>& configs)
    : bus_(bus) {
  for (const auto& cfg : configs) {
    addSensor(cfg.xshutPin);
  }
}

void VL53L0XManager::addSensor(uint8_t xshutPin, uint8_t address) {
  VL53L0XSensor s;
  s.xshutPin = xshutPin;
  s.address = (address == 0) ? 0x30 + sensors_.size() : address;
  s.lastRange = 0;
  s.timeout = false;
  sensors_.push_back(s);
}

bool VL53L0XManager::init() {
  for (auto& s : sensors_) {
    pinMode(s.xshutPin, OUTPUT);
    digitalWrite(s.xshutPin, LOW);
  }
  delay(50);

  bus_->begin();
  bus_->setClock(400000);  // 400 kHz fast_mode / 100 kHz robust_mode

  for (uint8_t i = 0; i < sensors_.size(); i++) {
    auto& s = sensors_[i];
    digitalWrite(s.xshutPin, HIGH);
    delay(50);

    s.sensor.setBus(bus_);
    s.sensor.setTimeout(500);
    if (!s.sensor.init()) return false;

    s.sensor.setAddress(s.address);
    s.sensor.setSignalRateLimit(0.1);
    s.sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 16);
    s.sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 12);
    s.sensor.setMeasurementTimingBudget(50000);
    s.sensor.startContinuous(100);
  }

  return true;
}

void VL53L0XManager::update() {
  for (uint8_t i = 0; i < sensors_.size(); ++i) {
    auto& s = sensors_[i];

    if ((s.sensor.readReg(0x13) & 0x07) != 0) {
      s.lastRange = s.sensor.readRangeContinuousMillimeters();
      s.timeout = s.sensor.timeoutOccurred();
    }

    data_.range[i] = s.timeout ? NAN : (s.lastRange / 1000.0);
  }
}

uint8_t VL53L0XManager::count() const { return sensors_.size(); }

VL53L0XSensor& VL53L0XManager::getSensor(uint8_t index) {
  return sensors_[index];
}
