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

#include "battery_adc.hpp"
#include "hardware_encoder.hpp"
#include "imu_bno055.hpp"
#include "led_indicator.hpp"
#include "motor_array.hpp"
#include "motor_drv8848.hpp"
#include "pid.hpp"

enum class MotorID : uint8_t { FL = 0, FR = 1, RL = 2, RR = 3, COUNT = 4 };

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

// ────────────── LEDs ──────────────
#define RED_LED PE2
#define GRN_LED PE3
#define GRN_LED2 PE4

inline constexpr LedIndicatorConfig led_status_config = {
    .pin             = RED_LED,
    .initial_state   = HIGH,
    .blink_period_ms = 500,
    .label           = "STATUS",
};

// ────────────── Motors ──────────────
inline constexpr DriverGroupConfig right_motors_driver = {PC13, PE0};
inline constexpr DriverGroupConfig left_motors_driver = {PC14, PE1};
inline constexpr DriverGroupConfig driver_groups[] = {
    right_motors_driver,
    left_motors_driver,
};

constexpr uint32_t MOTOR_PWM_FREQ = 20000;  // 20 kHz
constexpr float MAX_VELOCITY = 30.0f;
constexpr float MIN_VELOCITY = 1.0f;

inline constexpr MotorDrv8848Config motor_fl_config = {
    .pwm_pin = PF9,
    .in_a_pin = PE5,
    .in_b_pin = PE6,
    .dir_cw = false,
    .max_velocity = MAX_VELOCITY,
    .min_velocity = MIN_VELOCITY,
    .pwm_freq = MOTOR_PWM_FREQ,
    .label = "FL",
};

inline constexpr MotorDrv8848Config motor_fr_config = {
    .pwm_pin = PF6,
    .in_a_pin = PG10,
    .in_b_pin = PG11,
    .dir_cw = true,
    .max_velocity = MAX_VELOCITY,
    .min_velocity = MIN_VELOCITY,
    .pwm_freq = MOTOR_PWM_FREQ,
    .label = "FR",
};

inline constexpr MotorDrv8848Config motor_rl_config = {
    .pwm_pin = PF8,
    .in_a_pin = PC15,
    .in_b_pin = PF2,
    .dir_cw = false,
    .max_velocity = MAX_VELOCITY,
    .min_velocity = MIN_VELOCITY,
    .pwm_freq = MOTOR_PWM_FREQ,
    .label = "RL",
};

inline constexpr MotorDrv8848Config motor_rr_config = {
    .pwm_pin = PF7,
    .in_a_pin = PD3,
    .in_b_pin = PD4,
    .dir_cw = true,
    .max_velocity = MAX_VELOCITY,
    .min_velocity = MIN_VELOCITY,
    .pwm_freq = MOTOR_PWM_FREQ,
    .label = "RR",
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

inline const std::array<const char*, 4> RANGE_FRAME_IDS = {
    "fl_range", "fr_range", "rl_range", "rr_range"};

// ────────────── SBC Interface ──────────────
#define SBC_SERIAL_TIMEOUT 1  // ms
#define SBC_STATUS \
  PG6  // According to "Rosbot v1.3 schematics", this should be connected to
       // GPIO_03 in RPI which is an I2C with pullup (intended for detection)
#define RPI_CONSOLE PG5
#define RPI_BTN PG7
