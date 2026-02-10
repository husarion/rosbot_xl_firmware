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

#include "control/encoder.hpp"
#include "control/types.hpp"

struct EncodersData
{
    float position[static_cast<uint8_t>(MotorID::COUNT)];
    float velocity[static_cast<uint8_t>(MotorID::COUNT)];
    float effort[static_cast<uint8_t>(MotorID::COUNT)];
    uint64_t timestamp_ns = 0;
};


class EncoderManager {
 public:
  static constexpr uint8_t NUM_ENCODERS = static_cast<uint8_t>(MotorID::COUNT);
  EncoderManager() = default;

  void init();

  Encoder& operator[](MotorID id) {
    return encoders_[static_cast<uint8_t>(id)];
  }

  const Encoder& operator[](MotorID id) const {
    return encoders_[static_cast<uint8_t>(id)];
  }

  void update();
  EncodersData getData() const { return data_; }

 private:
  Encoder encoders_[NUM_ENCODERS];
  EncodersData data_;
};

inline EncoderManager encoders;
