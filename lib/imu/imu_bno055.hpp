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

#include "imu_interface.hpp"

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


class ImuBno055 : public ImuInterface {
public:
    ImuBno055(TwoWire* bus, uint8_t id, uint8_t addr, Adafruit_BNO055::adafruit_bno055_axis_remap_config_t axis_config);

    bool init() override;
    void update() override;
    const char* name() const override { return "BNO055"; }

private:
    TwoWire*         bus_;
    uint8_t          id_;
    uint8_t          addr_;
    Adafruit_BNO055::adafruit_bno055_axis_remap_config_t axis_config_;
    Adafruit_BNO055* bno_ = nullptr;
    ImuData          data_ = {};
};

extern ImuBno055 imu_impl;