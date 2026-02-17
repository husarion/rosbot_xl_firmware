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

#include "encoder_array.hpp"

EncoderArray::EncoderArray(EncoderInterface** encoders, uint8_t count)
    : encoders_(encoders), count_(count) {
  if (count_ > MAX_NUM_ENCODERS) {
    count_ = MAX_NUM_ENCODERS;
  }
  data_.count = count_;
}

void EncoderArray::init() {
  data_.count = count_;
  for (uint8_t i = 0; i < count_; ++i) encoders_[i]->init();
}

void EncoderArray::update() {
  for (uint8_t i = 0; i < count_; ++i) {
    encoders_[i]->update();
    const auto d = encoders_[i]->getData();
    data_.position[i] = d.position;
    data_.velocity[i] = d.velocity;
  }
}

void EncoderArray::resetEncoder(uint8_t idx) {
  if (idx < count_) encoders_[idx]->reset();
}

void EncoderArray::resetAll() {
  for (uint8_t i = 0; i < count_; ++i) encoders_[i]->reset();
}
