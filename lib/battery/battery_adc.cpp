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

#include "battery_adc.hpp"
#include <Arduino.h>

BatteryAdc::BatteryAdc(uint8_t adc_pin, float divider_ratio,
                       float v_ref, float v_min, float v_max, float correction)
    : pin_(adc_pin), divider_ratio_(divider_ratio), v_ref_(v_ref), v_min_(v_min), v_max_(v_max), correction_(correction) {}

void BatteryAdc::init() {
    pinMode(pin_, INPUT_ANALOG);
}

void BatteryAdc::update() {
    uint16_t raw = analogRead(pin_) / 4095.0f;

    data_.voltage = v_ref_ * correction_ * divider_ratio_ * raw;

    float pct = (data_.voltage - v_min_) / (v_max_ - v_min_);
    if (pct > 1.0f) pct = 1.0f;
    if (pct < 0.0f) pct = 0.0f;
    data_.percentage = pct;
}