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

/// Configuration for a status LED indicator.
struct LedIndicatorConfig {
  uint8_t pin;               // GPIO pin number
  uint8_t initial_state;     // LOW or HIGH at startup
  uint16_t blink_period_ms;  // blink interval [ms]
  const char* label;         // e.g. "STATUS"
};

/// Status LED with three modes:
///   - solid ON     = µROS disconnected
///   - blinking     = battery low
///   - SOS pattern  = error
///   - OFF          = all OK
class LedIndicator {
 public:
  LedIndicator() = default;
  explicit LedIndicator(const LedIndicatorConfig& cfg) : cfg_(cfg) {}

  /// Initialize GPIO pin and set initial state.
  void init();

  /// Call in main loop or periodic task.
  /// Priority: error > battery_low > disconnected > OK.
  void update(bool battery_low, bool uros_disconnected, bool error);

  const char* name() const { return cfg_.label; }
  bool isAvailable() const { return cfg_.pin != 0; }

 private:
  void handleBlink(uint32_t now);
  void handleSOS(uint32_t now);
  void resetSOS() { sos_step_ = 0; }

  LedIndicatorConfig cfg_ = {};
  uint32_t last_toggle_ = 0;
  uint8_t sos_step_ = 0;
};

extern LedIndicator g_indicator;
