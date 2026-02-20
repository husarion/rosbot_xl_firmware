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

#include <cstdint>

#include "range_interface.hpp"

#define MAX_NUM_RANGE_SENSORS 4
#define MAX_FRAME_ID_LENGTH 16

struct RangesData {
  float range[MAX_NUM_RANGE_SENSORS] = {};
  char frame_id[MAX_NUM_RANGE_SENSORS][MAX_FRAME_ID_LENGTH] = {};
  uint8_t count = 0;
};

/// Holds N independent RangeInterface pointers.
class RangeArray {
 public:
  RangeArray() = default;
  RangeArray(RangeInterface** sensors, uint8_t count);

  void init();
  void update();

  const RangesData getData() const { return data_; }
  uint8_t count() const { return count_; }
  bool isAvailable() const { return count_ > 0; }

  RangeInterface* operator[](uint8_t idx) {
    return (idx < count_) ? sensors_[idx] : nullptr;
  }
  const RangeInterface* operator[](uint8_t idx) const {
    return (idx < count_) ? sensors_[idx] : nullptr;
  }

 private:
  RangeInterface** sensors_ = nullptr;
  uint8_t count_ = 0;
  RangesData data_ = {};
};

extern RangeArray g_ranges;
