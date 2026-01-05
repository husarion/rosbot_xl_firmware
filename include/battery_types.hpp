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

#include "hardware_cfg.hpp"

typedef enum {
  unknown_status = 0,
  charging = 1,
  discharging = 2,
  not_charging = 3,
  full = 4
} BatteryStatusTypeDef;

typedef enum {
  unknown_health = 0,
  good = 1,
  overhaet = 2,
  dead = 3,
  overvoltage = 4,
  unspec_failure = 5,
  cold = 6,
  watchdog_timer_expire = 7,
  safety_timer_expire = 8
} BatteryHealthTypeDef;

typedef enum {
  unknown_type = 0,
  NIMH = 1,
  LION = 2,
  LIPO = 3,
  LIFE = 4,
  NICD = 5,
  LIMN = 6
} BatteryTechnologyTypeDef;

typedef struct {
  // ROS battery msgs variables
  float voltage;
  float temperature;
  float current;
  float charge_current;
  float capacity;
  float design_capacity;
  float percentage;
  float cell_temperature[BATTERY_STATE_MSG_CELL_TEMPERATURE_ARRAY_SIZE];
  float cell_voltage[BATTERY_STATE_MSG_CELL_VOLTAGE_ARRAY_SIZE];
  BatteryStatusTypeDef status;
  BatteryHealthTypeDef health;
  BatteryTechnologyTypeDef technology;
  bool present;
} battery_state_t;
