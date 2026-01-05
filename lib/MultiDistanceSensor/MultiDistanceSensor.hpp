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
#include <VL53L0X.h>
#include <Wire.h>

#define NUM_DISTANCE_SENSORS 4
#define DISTANCE_SENSORS_DEFAULT_I2C_FREQ 100000U

enum SensorSelector : uint8_t {
  SENSOR_FR = 0,
  SENSOR_FL = 1,
  SENSOR_RR = 2,
  SENSOR_RL = 3
};

struct SensorsMeasurement {
  float range[NUM_DISTANCE_SENSORS];  // meters
  uint32_t timestamp;                 // millis()
  uint8_t status;
};

class MultiDistanceSensor {
 public:
  enum {
    ERR_NONE = 0,
    ERR_BUSY = 1,
    ERR_I2C_FAILURE = 2,
    ERR_NOT_READY = 3,
    ERR_NOT_INIT = 4
  };

  static MultiDistanceSensor& getInstance();

  // Initialize sensors with XSHUT pins array
  int init(uint8_t xshutPins[NUM_DISTANCE_SENSORS]);

  // Start/stop continuous measurement
  void start();
  void stop();

  // Read latest measurement
  void readMeasurements(SensorsMeasurement& measurement);

  // Call periodically to update measurements
  void update();

 private:
  MultiDistanceSensor();
  ~MultiDistanceSensor();

  int restartSensors();
  void runMeasurement();

  VL53L0X _sensors[NUM_DISTANCE_SENSORS];
  uint8_t _xshutPins[NUM_DISTANCE_SENSORS];
  bool _isActive[NUM_DISTANCE_SENSORS];
  bool _initialized;
  bool _sensorsEnabled;
  int _lastSensorIndex;
  SensorsMeasurement _m;

  uint32_t _lastUpdateTime;
  uint16_t _updateInterval;  // ms
};
