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

#include "ranges.hpp"

#include <Arduino.h>

#include "bsp.hpp"

VL53L0XManager rangeSensorsManager(&range_i2c);

VL53L0XManager::VL53L0XManager(TwoWire* bus) : _bus(bus) {}

void VL53L0XManager::addSensor(uint8_t xshutPin, uint8_t address) {
    VL53L0XSensor s;
    s.xshutPin = xshutPin;
    s.address = (address == 0) ? 0x30 + _sensors.size() : address;
    s.lastRange = 0;
    s.timeout = false;
    _sensors.push_back(s);
}

bool VL53L0XManager::begin() {
    for (auto& s : _sensors) {
        pinMode(s.xshutPin, OUTPUT);
        digitalWrite(s.xshutPin, LOW);
    }
    delay(50);

    _bus->begin();
    _bus->setClock(100000);

    for (size_t i = 0; i < _sensors.size(); i++) {
        auto& s = _sensors[i];
        digitalWrite(s.xshutPin, HIGH);
        delay(50);

        s.sensor.setBus(_bus);
        s.sensor.setTimeout(500);
        if (!s.sensor.init()) return false;

        s.sensor.setAddress(s.address);
        s.sensor.setSignalRateLimit(0.1);
        s.sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 16);
        s.sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 12);
        s.sensor.setMeasurementTimingBudget(50000);
        s.sensor.startContinuous(100);
    }

    return true;
}

void VL53L0XManager::readAll() {
    for (auto& s : _sensors) {
        if ((s.sensor.readReg(0x13) & 0x07) != 0) {
            s.lastRange = s.sensor.readRangeContinuousMillimeters();
            s.timeout = s.sensor.timeoutOccurred();
        }
    }
}

size_t VL53L0XManager::count() const {
    return _sensors.size();
}

VL53L0XSensor& VL53L0XManager::getSensor(size_t index) {
    return _sensors[index];
}

// NOWA FUNKCJA – zwraca wskaźnik do tablicy ostatnich pomiarów
uint16_t* VL53L0XManager::getAllRanges() {
    static uint16_t ranges[32]; // maksymalnie 32 sensory – wskaźnik statyczny, bezpieczny do odczytu
    size_t n = _sensors.size() > 32 ? 32 : _sensors.size();
    for (size_t i = 0; i < n; i++) {
        ranges[i] = _sensors[i].lastRange;
    }
    return ranges;
}
