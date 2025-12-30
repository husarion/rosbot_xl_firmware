/**
 * @file BATTERY_TYPES_H.h
 * @author Jakub Klein
 * @brief
 * @version 0.1
 * @date 2025-09-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef BATTERY_TYPES_H
#define BATTERY_TYPES_H

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

#endif /* BATTERY_TYPES_H */
