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
#include "battery.hpp"

struct LedConfig {
    uint8_t pin;
    bool blink = false;
};

class LedIndicator {
public:
    void init(const LedConfig& battery, const LedConfig& uros, const LedConfig& error)
    {
        battery_ = battery;
        uros_ = uros;
        error_ = error;

        initLed(battery_);
        initLed(uros_);
        initLed(error_);
    }

    void update(bool batteryLow, bool urosConnected, bool errorState) {
        updateLed(battery_, battery_last_toggle_, batteryLow);
        updateLed(uros_, uros_last_toggle_, urosConnected);
        updateLed(error_, error_last_toggle_, errorState);
    }

private:
    static constexpr uint32_t BLINK_INTERVAL_MS = 500;

    LedConfig battery_;
    LedConfig uros_;
    LedConfig error_;

    uint32_t battery_last_toggle_;
    uint32_t uros_last_toggle_;
    uint32_t error_last_toggle_;

    void initLed(const LedConfig& led) {
        pinMode(led.pin, OUTPUT);
        digitalWrite(led.pin, LOW);
    }

    void updateLed(const LedConfig& config, uint32_t& last_toggle, bool condition) {
        if (config.blink) {
            if (condition) {
                uint32_t now = millis();
                if (now - last_toggle >= BLINK_INTERVAL_MS) {
                    digitalToggle(config.pin);
                    last_toggle = now;
                }
            } else {
                digitalWrite(config.pin, LOW);
            }
        } else {
            digitalWrite(config.pin, condition);
        }
    }

    // void blink(uint8_t pin, uint16_t duration_ms) {
    //     digitalWrite(pin, HIGH);
    //     delay(duration_ms);
    //     digitalWrite(pin, LOW);
    //     delay(200);
    // }

    // void errorLoop(const char* func) {
    //     digitalWrite(battery_.pin, LOW);
    //     digitalWrite(error_.pin, LOW);
    //     digitalWrite(uros_.pin, LOW);

    //     delay(500);

    //     // 2 SOS signals: ... --- ...
    //     for (uint8_t i = 0; i < 2; ++i) {
    //         for (uint8_t i = 0; i < 3; ++i) {
    //             blink(error_.pin, 200);
    //         }
    //         for (uint8_t i = 0; i < 3; ++i) {
    //             blink(error_.pin, 600);
    //         }
    //         for (uint8_t i = 0; i < 3; ++i) {
    //             blink(error_.pin, 200);
    //         }
    //         delay(1000);
    //     }

    //     NVIC_SystemReset();
    // }
};

inline LedIndicator ledIndicator;