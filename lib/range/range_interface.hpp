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

struct RangeData {
  float range = NAN;
};

class RangeInterface {
 public:
  virtual ~RangeInterface() = default;

  virtual void init() = 0;
  virtual void update() = 0;
  virtual void powerOff() = 0;
  virtual void powerOn() = 0;
  virtual const RangeData getData() const { return data_; }
  virtual const char* name() const = 0;

 protected:
  RangeData data_;
};
