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

#include "range_array.hpp"

#include <Arduino.h>

RangeArray::RangeArray(RangeInterface** sensors, uint8_t count)
    : sensors_(sensors), count_(count) {
  if (count_ > MAX_NUM_RANGE_SENSORS) {
    count_ = MAX_NUM_RANGE_SENSORS;
  }
  data_.count = count_;
}

void RangeArray::init() {
  for (uint8_t i = 0; i < count_; i++) {
    sensors_[i]->powerOff();
  }
  for (uint8_t i = 0; i < count_; i++) {
    sensors_[i]->init();
  }
}

void RangeArray::update() {
  for (uint8_t i = 0; i < count_; i++) {
    sensors_[i]->update();
    data_.range[i] = sensors_[i]->getData().range;
    strncpy(data_.frame_id[i], sensors_[i]->name(), MAX_FRAME_ID_LENGTH - 1);
  }
}
