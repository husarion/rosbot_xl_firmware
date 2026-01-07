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

#include <Wire.h>
#include <VL53L0X.h>
#include <Arduino.h>
#include <vector>

enum Ranges {
  RF, FL, RR, RL, RANGES_COUNT
};
static const char *range_frame_names[] = {"fr_range", "fl_range", "rr_range", "rl_range"};

typedef struct {
  float range[RANGES_COUNT];
} ranges_data_t;

struct VL53L0XSensor {
    VL53L0X sensor;
    uint8_t xshutPin;
    uint8_t address;
    uint16_t lastRange;
    bool timeout;
};

class VL53L0XManager {
public:
    VL53L0XManager(TwoWire* bus);

    void addSensor(uint8_t xshutPin, uint8_t address = 0);
    bool begin();
    void readAll();

    size_t count() const;
    VL53L0XSensor& getSensor(size_t index);

    // NOWA FUNKCJA – zwraca wskaźnik do tablicy ostatnich pomiarów
    uint16_t* getAllRanges();

private:
    TwoWire* _bus;
    std::vector<VL53L0XSensor> _sensors;
};

extern VL53L0XManager rangeSensorsManager;
