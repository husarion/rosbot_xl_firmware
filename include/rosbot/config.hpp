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
#include <HardwareSerial.h>

#include <array>

#include "control/types.hpp"

namespace control {

constexpr uint8_t RIGHT_WHEELS_SLEEP = PC13;
constexpr uint8_t RIGHT_WHEELS_FAULT = PE0;
constexpr uint8_t LEFT_WHEELS_SLEEP = PC14;
constexpr uint8_t LEFT_WHEELS_FAULT = PE1;
constexpr uint32_t MOTOR_PWM_FREQ = 20000;  // 20 kHz
constexpr float MAX_VELOCITY = 30.0f;
constexpr float MIN_VELOCITY = 1.0f;
constexpr float MIN_FRICTION_OUTPUT = 0.4f;

inline const std::array<MotorConfig, static_cast<size_t>(MotorID::COUNT)>
    CONFIG = {{{MotorID::FL,
                {PF9, PE5, PE6, PB6, PB7},
                TIM4,
                Direction::CCW,
                "fl_wheel_joint"},

               {MotorID::FR,
                {PF6, PG10, PG11, PA0, PA1},
                TIM2,
                Direction::CW,
                "fr_wheel_joint"},

               {MotorID::RL,
                {PF8, PC15, PF2, PB4, PA7},
                TIM3,
                Direction::CCW,
                "rl_wheel_joint"},

               {MotorID::RR,
                {PF7, PD3, PD4, PC6, PC7},
                TIM8,
                Direction::CW,
                "rr_wheel_joint"}}};

}  // namespace control

// ============================================================================
// ROBOT PHYSICAL PARAMETERS
// ============================================================================
namespace RobotParams {

// Wheel & Drivetrain
constexpr float GEAR_RATIO = 34.014f;
constexpr uint16_t ENCODER_CPR = 48;
constexpr float TICKS_PER_REVOLUTION = ENCODER_CPR * GEAR_RATIO;
constexpr float RAD_PER_TICK = (2.0f * PI) / TICKS_PER_REVOLUTION;

}  // namespace RobotParams

struct SerialConfig {
  HardwareSerial* serial;
  uint32_t baudrate;
  uint8_t rxPin;
  uint8_t txPin;
  uint32_t timeout;
  const char* name;
};

// Primary: SBC Serial (SBC connection)
inline constexpr SerialConfig SBC_SERIAL_CONFIG = {.serial = &Serial1,
                                                   .baudrate = 921600,
                                                   .rxPin = PA10,
                                                   .txPin = PA9,
                                                   .timeout = 1,
                                                   .name = "SBC_SERIAL"};

// Secondary: FTDI Serial (Rear panel USB connection)
inline constexpr SerialConfig FTDI_SERIAL_CONFIG = {.serial = &Serial3,
                                                    .baudrate = 921600,
                                                    .rxPin = PB11,
                                                    .txPin = PB10,
                                                    .timeout = 1,
                                                    .name = "FTDI_SERIAL"};

// ============== Buttons ==============
#define PUSH_BUTTON1 PG12
#define PUSH_BUTTON2 PG13

// ============== LEDs ==============
#define RED_LED PE2
#define GRN_LED PE3
#define GRN_LED2 PE4

// ============== Battery ==============
#define BATTERY_ADC_PIN PA5

// ============== Power Management ==============
#define POWEROFF_DELAY 5000  // ms

// ============== SBC Interface ==============
#define SBC_SERIAL_TIMEOUT 1  // ms
#define SBC_STATUS \
  PG6  // According to "Rosbot v1.3 schematics", this should be connected to
       // GPIO_03 in RPI which is an I2C with pullup (intended for detection)
#define RPI_CONSOLE PG5
#define RPI_BTN PG7

// ============== IMU ==============
#define IMU_POWER_ON PG4
#define IMU_I2C_SDA PC9
#define IMU_I2C_SCL PA8
#define IMU_ID 0xA0  // used internally by the Adafruit Unified Sensor API ?
#define IMU_ADDR_A 0x28
#define IMU_ADDR_B 0x29

// ============== Ranges ==============
#define RANGE_FR_SHD_PIN PB1
#define RANGE_FL_SHD_PIN PD8
#define RANGE_RR_SHD_PIN PD9
#define RANGE_RL_SHD_PIN PD10
#define RANGE_I2C_SDA PB9
#define RANGE_I2C_SCL PB8
