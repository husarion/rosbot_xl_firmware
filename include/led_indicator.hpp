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

class LedStatusIndicator {
 public:
  void init(uint8_t pin, uint8_t initial_state = LOW) {
    pin_ = pin;
    pinMode(pin_, OUTPUT);
    digitalWrite(pin_, initial_state);
  }

  void update(bool battery_low, bool uros_connected, bool error) {
    uint32_t now = millis();
    if (error) {
      handleSOS(now);
      return;
    }
    resetSOS();

    if (battery_low) {
      handleBlink(now);
      return;
    }

    if (!uros_connected) {
      digitalWrite(pin_, HIGH);
      return;
    }

    digitalWrite(pin_, LOW);
  }

 private:
  uint8_t pin_{0};
  uint32_t lastToggle_{0};
  uint8_t sosStep_{0};

  void handleBlink(uint32_t now) {
    const uint16_t blinkPeriod = 500;

    if (now - lastToggle_ >= blinkPeriod) {
      digitalToggle(pin_);
      lastToggle_ = now;
    }
  }

  void handleSOS(uint32_t now) {
    static const uint16_t sosPattern[] = {
        200, 200, 200,  // S
        600, 200, 600,  // O
        200, 200, 200,  // S
        1000            // Pause
    };

    if (now - lastToggle_ >= sosPattern[sosStep_]) {
      digitalToggle(pin_);
      lastToggle_ = now;
      sosStep_++;

      if (sosStep_ >= sizeof(sosPattern) / sizeof(sosPattern[0])) {
        sosStep_ = 0;
      }
    }
  }

  void resetSOS() { sosStep_ = 0; }
};

inline LedStatusIndicator ledIndicator;
