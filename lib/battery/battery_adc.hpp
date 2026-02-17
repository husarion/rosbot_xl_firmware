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

#include "battery_interface.hpp"

struct BatteryAdcConfig {
  uint8_t adc_pin;
  float v_ref;
  float v_min;
  float v_max;
  float divider;
  float correction = 1.0f;
};

class BatteryAdc : public BatteryInterface {
 public:
  explicit BatteryAdc(const BatteryAdcConfig config);

  void init() override;
  void update() override;
  const char* name() const override { return "ADC"; }

 private:
  const BatteryAdcConfig cfg_;
  float voltage_factor_ = 1.0f;
  float voltage_range_inv_ = 0.0f;
  uint16_t adc_dma_buffer_[1] = {};
};
