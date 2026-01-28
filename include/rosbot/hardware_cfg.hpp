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

/* OTHERS */
#define DEFAULT_FIRMWARE_MODE 2  // 0 - normal; 1 - error; 2 - debug

/* POWER OFF */
#define POWEROFF_DELAY 5000  // ms

/* REAR PANEL */
#define BUTTONS_COUNT 2

/* SBC */
#define SBC_SERIAL_TIMEOUT 1  // ms
#define SBC_STATUS \
  PG6  // According to "Rosbot v1.3 schematics", this should be connected to
       // GPIO_03 in RPI which is an I2C with pullup (intended for detection)
#define RPI_CONSOLE PG5
#define RPI_BTN PG7

/* IMU */
#define IMU_POWER_ON PG4
#define IMU_I2C_SDA PC9
#define IMU_I2C_SCL PA8
#define IMU_ID 0xA0  // used internally by the Adafruit Unified Sensor API ?
#define IMU_ADDR_A 0x28
#define IMU_ADDR_B 0x29

/* Range Sensors */
#define RANGE_FR_SHD_PIN PB1
#define RANGE_FL_SHD_PIN PD8
#define RANGE_RR_SHD_PIN PD9
#define RANGE_RL_SHD_PIN PD10
#define RANGE_I2C_SDA PB9
#define RANGE_I2C_SCL PB8

/* EXTERNAL PERIPHERALS */

// // EXT SPI
// #define EXT_SPI SPI1
// #define EXT_SPI_SCK PA5
// #define EXT_SPI_MISO PA6
// #define EXT_SPI_MOSI PB5
// // EXT I2C1
// #define EXT_I2C1 I2C1
// #define EXT_I2C1_SDA PB7
// #define EXT_I2C1_SCL PB6
// // EXT I2C2
// #define EXT_I2C2 I2C3
// #define EXT_I2C2_SDA PC9
// #define EXT_I2C2_SCL PA8
// // EXT Serial
// #define EXT_SERIAL_EN_FLAG 1
// #define EXT_SERIAL Serial6
// #define EXT_SERIAL_BAUDRATE 115200
// #define EXT_SERIAL_RX PG9
// #define EXT_SERIAL_TX PG14
// // EXT PWM1
// #define EXT_PWM1_TIM TIM9
// #define EXT_PWM1_CH CH1
// #define EXT_PWM1_PIN PE5
// // EXT PWM2
// #define EXT_PWM2_TIM TIM9
// #define EXT_PWM2_CH CH2
// #define EXT_PWM2_PIN PE6
// // EXT PWM3
// #define EXT_PWM3_TIM TIM12
// #define EXT_PWM3_CH CH1
// #define EXT_PWM3_PIN PB14
// // EXT ANALOG
// #define EXT_ANALOG_IN1 PF10
// #define EXT_ANALOG_IN2 PF3
// // EXT GPIO
// #define EXT_GPIO1 PG2
// #define EXT_GPIO2 PG3
// #define EXT_GPIO3 PG4

/* WATCHDOG */
#define WATCHDOG_TIMEOUT 20000000  // microseconds

/* BATTERY */
#define BATTERY_ADC_PIN PA5
#define BATTERY_CELLS_SERIES 3
#define BATTERY_CELLS_PARALLEL 3
#define BATTERY_STATE_MSG_CELL_TEMPERATURE_ARRAY_SIZE 1  // in unmeasured
#define BATTERY_STATE_MSG_CELL_VOLTAGE_ARRAY_SIZE 1      // in unmeasured
// #define BATTERY_STATE_MSG_CELL_TEMPERATURE_ARRAY_SIZE
// (BATTERY_CELLS_PARALLEL * BATTERY_CELLS_SERIES) #define
// BATTERY_STATE_MSG_CELL_VOLTAGE_ARRAY_SIZE (BATTERY_CELLS_PARALLEL *
// BATTERY_CELLS_SERIES)
