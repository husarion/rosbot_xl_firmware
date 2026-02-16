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

#include <cmath>
#include <cstdint>

struct BatteryData {
  float current = NAN;
  float percentage = NAN;
  float temperature = NAN;
  float voltage = NAN;
};

class BatteryInterface {
 public:
  virtual ~BatteryInterface() = default;

  virtual void init() = 0;
  virtual void update() = 0;
  virtual const BatteryData getData() const { return data_; }
  virtual const char* name() const = 0;

  virtual bool isLow(float thresh = 0.2, float hist = 0.02) {
    const float p = data_.percentage;
    is_low_ = (is_low_ ? (p <= thresh + hist) : (p < thresh));
    return is_low_;
  }

  virtual bool isCritical(float thresh = 0.05, float hist = 0.02) {
    const float p = data_.percentage;
    is_critical_ = (is_critical_ ? (p <= thresh + hist) : (p < thresh));
    return is_critical_;
  }

 protected:
  BatteryData data_ = {};
  bool is_low_{false};
  bool is_critical_{false};
};

extern BatteryInterface* g_battery;
