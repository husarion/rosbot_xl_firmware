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

#include "encoder_interface.hpp"

#define MAX_NUM_ENCODERS 4

struct EncodersData {
  float position[MAX_NUM_ENCODERS] = {};
  float velocity[MAX_NUM_ENCODERS] = {};
  uint8_t count = 0;
};

class EncoderArray {
 public:
  EncoderArray() = default;
  EncoderArray(EncoderInterface** encoders, uint8_t count);

  void init();
  void update();

  const EncodersData getData() const { return data_; }
  uint8_t count() const { return count_; }
  bool isAvailable() const { return count_ > 0; }
  void resetEncoder(uint8_t idx);
  void resetAll();

  EncoderInterface* operator[](uint8_t idx) {
    return (idx < count_) ? encoders_[idx] : nullptr;
  }
  const EncoderInterface* operator[](uint8_t idx) const {
    return (idx < count_) ? encoders_[idx] : nullptr;
  }

 private:
  EncoderInterface** encoders_ = nullptr;
  uint8_t count_ = 0;
  EncodersData data_ = {};
};

extern EncoderArray g_encoders;
