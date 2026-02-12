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

BatteryAdc::BatteryAdc(const ADCConfig config)
    : config_(config) {}

void BatteryAdc::init() {
    pinMode(config_.adc_pin, INPUT);
}

void BatteryAdc::update() {
    float raw = analogRead(config_.adc_pin) / 1023.0f;

    data_.voltage = config_.v_ref * config_.correction * config_.divider * raw;

    float pct = (data_.voltage - config_.v_min) / (config_.v_max - config_.v_min);
    if (pct > 1.0f) pct = 1.0f;
    if (pct < 0.0f) pct = 0.0f;
    data_.percentage = pct;
}