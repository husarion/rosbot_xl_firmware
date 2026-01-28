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

class Battery {
public:
    struct Config {
        uint8_t adcPin;
        float vRef = 3.3f;
        float correction = 0.986f;
        float upperResistor = 5.6e4;
        float lowerResistor = 1.0e4;
        float vMin = 9.6f;
        float vMax = 12.6f;
        float lowThreshold = 10.8f;
        float hysteresis = 0.2f;
    };

    void init(const Config& cfg);
    void init(uint8_t adcPin);
    void update();

    float current() const { return current_; }
    float temperature() const { return temperature_; }
    float voltage() const { return voltage_; }
    float percentage() const;
    bool isLow() const { return isLow_; }
    bool isCritical() const { return voltage_ < cfg_.vMin; }

private:
    Config cfg_;
    float dividerRatio_ = 1.0f;
    float current_ = NAN; // Not implemented
    float temperature_ = NAN; // Not implemented
    float voltage_ = 0.0f;
    bool isLow_ = false;
};

inline Battery battery;
