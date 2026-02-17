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

#include <array>

#include "control/types.hpp"
#include "battery_adc.hpp"
#include "hardware_encoder.hpp"
#include "imu_bno055.hpp"
#include "pid.hpp"

// ────────────── PID ──────────────
#define PID_KP 0.07f
#define PID_KI 0.4f
#define PID_KD 0.002f
#define PID_MAX_ACCEL 0.0f
#define PID_MIN_OUTPUT -1.0f
#define PID_MAX_OUTPUT 1.0f

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
                false,
                "fl_wheel_joint"},

               {MotorID::FR,
                {PF6, PG10, PG11, PA0, PA1},
                TIM2,
                true,
                "fr_wheel_joint"},

               {MotorID::RL,
                {PF8, PC15, PF2, PB4, PA7},
                TIM3,
                false,
                "rl_wheel_joint"},

               {MotorID::RR,
                {PF7, PD3, PD4, PC6, PC7},
                TIM8,
                true,
                "rr_wheel_joint"}}};

}  // namespace control

struct SerialConfig {
  HardwareSerial* serial;
  uint32_t baudrate;
  uint8_t rxPin;
  uint8_t txPin;
  uint32_t timeout_ms;
  const char* name;
};

// Primary: SBC Serial (SBC connection)
inline constexpr SerialConfig SBC_SERIAL_CONFIG = {.serial = &Serial1,
                                                   .baudrate = 921600,
                                                   .rxPin = PA10,
                                                   .txPin = PA9,
                                                   .timeout_ms = 1,
                                                   .name = "SBC_SERIAL"};

// Secondary: FTDI Serial (Rear panel USB connection)
inline constexpr SerialConfig FTDI_SERIAL_CONFIG = {.serial = &Serial3,
                                                    .baudrate = 921600,
                                                    .rxPin = PB11,
                                                    .txPin = PB10,
                                                    .timeout_ms = 1,
                                                    .name = "FTDI_SERIAL"};

// ────────────── Battery ──────────────
inline constexpr BatteryAdcConfig battery_adc_config = {
    .adc_pin = PA5,
    .v_ref = 3.3f,
    .v_min = 9.6f,
    .v_max = 12.6f,
    .divider = (5.6e4 + 1.0e4) / 1.0e4,
    .correction = 0.986f};

// ────────────── Buttons ──────────────
#define PUSH_BUTTON1 PG12
#define PUSH_BUTTON2 PG13

// ────────────── Encoders ──────────────
constexpr float GEAR_RATIO = 34.014f;
constexpr uint16_t ENCODER_CPR = 48;
constexpr float TICKS_PER_REVOLUTION = ENCODER_CPR * GEAR_RATIO;
constexpr float RAD_PER_TICK = (2.0f * PI) / TICKS_PER_REVOLUTION;

inline constexpr HardwareEncoderConfig enc_fl_config = {
    .pin_a = PB6,
    .pin_b = PB7,
    .timer = TIM4,
    .dir_cw = false,
    .rad_per_tick = RAD_PER_TICK,
    .label = "fl",
};

inline constexpr HardwareEncoderConfig enc_fr_config = {
    .pin_a = PA0,
    .pin_b = PA1,
    .timer = TIM2,
    .dir_cw = true,
    .rad_per_tick = RAD_PER_TICK,
    .label = "fr",
};

inline constexpr HardwareEncoderConfig enc_rl_config = {
    .pin_a = PB4,
    .pin_b = PA7,
    .timer = TIM3,
    .dir_cw = false,
    .rad_per_tick = RAD_PER_TICK,
    .label = "rl",
};

inline constexpr HardwareEncoderConfig enc_rr_config = {
    .pin_a = PC6,
    .pin_b = PC7,
    .timer = TIM8,
    .dir_cw = true,
    .rad_per_tick = RAD_PER_TICK,
    .label = "rr",
};

// ────────────── LEDs ──────────────
#define RED_LED PE2
#define GRN_LED PE3
#define GRN_LED2 PE4

// ────────────── Power Management ──────────────
#define POWEROFF_DELAY 5000  // ms

// ────────────── SBC Interface ──────────────
#define SBC_SERIAL_TIMEOUT 1  // ms
#define SBC_STATUS \
  PG6  // According to "Rosbot v1.3 schematics", this should be connected to
       // GPIO_03 in RPI which is an I2C with pullup (intended for detection)
#define RPI_CONSOLE PG5
#define RPI_BTN PG7

// ────────────── IMU ──────────────
#define IMU_POWER_ON PG4
#define IMU_I2C_SDA PC9
#define IMU_I2C_SCL PA8

inline TwoWire imu_i2c(IMU_I2C_SDA, IMU_I2C_SCL);
inline constexpr ImuBno055Config imu_bno055_config = {
    .bus = &imu_i2c,
    .i2c_addr = 0x29,
    .sensor_id = 0xA0,
    .int_pin = PA6,
    .axis_config = Adafruit_BNO055::REMAP_CONFIG_P0,
};

// ────────────── PID ──────────────
// PID configuration is the same for all motors
inline constexpr PIDConfig pid_config = {
    .kp = 0.07f,
    .ki = 0.4f,
    .kd = 0.002f,
    .min_output = -1.0f,
    .max_output = 1.0f,
    .min_power_to_move = 0.4f,
    .compensation_up_to_speed = 2.0f,
};

// ────────────── Ranges ──────────────
#define RANGE_I2C_SDA PB9
#define RANGE_I2C_SCL PB8
#define RANGE_XSHUT_FL PD8
#define RANGE_XSHUT_FR PB1
#define RANGE_XSHUT_RL PD10
#define RANGE_XSHUT_RR PD9

enum Ranges { RF, FL, RR, RL, COUNT };
struct RangeConfig {
  uint8_t xshutPin;
  const char* frame_id;
};

inline const std::array<RangeConfig, static_cast<size_t>(Ranges::COUNT)>
    RANGE_CONFIG = {{{PD8, "fl_range"},
                     {PB1, "fr_range"},
                     {PD10, "rl_range"},
                     {PD9, "rr_range"}}};
