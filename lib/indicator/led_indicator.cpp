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

#include "led_indicator.hpp"

void LedIndicator::init() {
  pinMode(cfg_.pin, OUTPUT);
  digitalWrite(cfg_.pin, cfg_.initial_state);
}

void LedIndicator::update(bool battery_low, bool uros_disconnected,
                          bool error) {
  uint32_t now = millis();

  // Priority 1: error → SOS pattern
  if (error) {
    handleSOS(now);
    return;
  }
  resetSOS();

  // Priority 2: battery low → blink
  if (battery_low) {
    handleBlink(now);
    return;
  }

  // Priority 3: µROS disconnected → solid ON
  if (uros_disconnected) {
    digitalWrite(cfg_.pin, HIGH);
    return;
  }

  // All OK → OFF
  digitalWrite(cfg_.pin, LOW);
}

void LedIndicator::handleBlink(uint32_t now) {
  if (now - last_toggle_ >= cfg_.blink_period_ms) {
    digitalToggle(cfg_.pin);
    last_toggle_ = now;
  }
}

void LedIndicator::handleSOS(uint32_t now) {
  static const uint16_t sos_pattern[] = {
      200, 200, 200,  // S (dit dit dit)
      600, 200, 600,  // O (dah dah dah)
      200, 200, 200,  // S (dit dit dit)
      1000            // pause
  };

  if (now - last_toggle_ >= sos_pattern[sos_step_]) {
    digitalToggle(cfg_.pin);
    last_toggle_ = now;
    sos_step_++;
    if (sos_step_ >= sizeof(sos_pattern) / sizeof(sos_pattern[0]))
      sos_step_ = 0;
  }
}
