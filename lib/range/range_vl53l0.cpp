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

#include "range_vl53l0.hpp"

#include <Arduino.h>

static constexpr uint8_t RESULT_READY_MASK = 0x07;

RangeVl53l0x::RangeVl53l0x(TwoWire* bus, uint8_t xshut_pin, uint8_t i2c_address,
                           const Vl53l0xConfig& config)
    : bus_(bus), xshut_pin_(xshut_pin), address_(i2c_address), cfg_(config) {}

void RangeVl53l0x::init() {
  powerOn();

  driver_.setBus(bus_);
  driver_.setTimeout(cfg_.timeout_ms);
  if (!driver_.init()) return;

  driver_.setAddress(address_);
  driver_.setSignalRateLimit(cfg_.signal_rate_limit);
  driver_.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange,
                              cfg_.vcsel_pre_range);
  driver_.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange,
                              cfg_.vcsel_final_range);
  driver_.setMeasurementTimingBudget(cfg_.timing_budget_us);
  driver_.startContinuous(cfg_.continuous_period_ms);

  initialized_ = true;
}

void RangeVl53l0x::update() {
  if (!initialized_) {
    data_.range = NAN;
    return;
  }

  uint16_t range_mm = driver_.readRangeContinuousMillimeters();
  data_.range =
      driver_.timeoutOccurred() ? NAN : static_cast<float>(range_mm) / 1000.0f;
}

void RangeVl53l0x::powerOff() {
  pinMode(xshut_pin_, OUTPUT);
  digitalWrite(xshut_pin_, LOW);
  initialized_ = false;
  delay(10);
}

void RangeVl53l0x::powerOn() {
  pinMode(xshut_pin_, OUTPUT);
  digitalWrite(xshut_pin_, HIGH);
  delay(20);
}
