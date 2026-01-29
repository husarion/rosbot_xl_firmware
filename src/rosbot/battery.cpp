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

#include "rosbot/battery.hpp"

#include <Arduino.h>

void Battery::init(const Config& cfg) {
  cfg_ = cfg;
  pinMode(cfg_.adcPin, INPUT);
  dividerRatio_ =
      (cfg_.upperResistor + cfg_.lowerResistor) / cfg_.lowerResistor;
}

void Battery::init(uint8_t adcPin) {
  Config cfg;
  cfg.adcPin = adcPin;
  init(cfg);
}

void Battery::update() {
  float raw = analogRead(cfg_.adcPin) / 1023.0f;
  voltage_ = cfg_.vRef * cfg_.correction * dividerRatio_ * raw;

  if (voltage_ < cfg_.lowThreshold) {
    isLow_ = true;
  } else if (voltage_ > cfg_.lowThreshold + cfg_.hysteresis) {
    isLow_ = false;
  }
}

float Battery::percentage() const {
  float v = constrain(voltage_, cfg_.vMin, cfg_.vMax);
  return (v - cfg_.vMin) / (cfg_.vMax - cfg_.vMin);
}
