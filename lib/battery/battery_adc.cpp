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
#include <cassert>
#include <cmath>

BatteryAdc::BatteryAdc(const ADCConfig config)
    : config_(config) {
    voltage_factor_ = config_.v_ref * config_.correction * config_.divider;
    assert(config_.v_max > config_.v_min);
    voltage_range_inv_ = 1.0f / (config_.v_max - config_.v_min);
}

void BatteryAdc::init() {
    pinMode(config_.adc_pin, INPUT);
}

void BatteryAdc::update() {
    constexpr float ADC_MAX_INV = 1.0f / 1023.0f;
    float raw = analogRead(config_.adc_pin) * ADC_MAX_INV;

    data_.voltage = raw * voltage_factor_;

    float pct = (data_.voltage - config_.v_min) * voltage_range_inv_;
    data_.percentage = fminf(fmaxf(pct, 0.0f), 1.0f);
}