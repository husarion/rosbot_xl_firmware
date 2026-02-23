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

// ────────────── Buttons ──────────────
static constexpr uint8_t PUSH_BUTTON1 = PF11;
static constexpr uint8_t PUSH_BUTTON2 = PF12;

// ────────────── Encoders ──────────────
constexpr float GEAR_RATIO = 50.0f;
constexpr uint16_t ENCODER_CPR = 64;
constexpr float TICKS_PER_REVOLUTION = ENCODER_CPR * GEAR_RATIO;
constexpr float RAD_PER_TICK = (2.0f * PI) / TICKS_PER_REVOLUTION;

inline constexpr HardwareEncoderConfig enc_fl_config = {
    .pin_a = PE9,
    .pin_b = PE11,
    .timer = TIM1,
    .dir_cw = false,
    .rad_per_tick = RAD_PER_TICK,
    .frame_id = "fl_wheel_joint",
};

inline constexpr HardwareEncoderConfig enc_fr_config = {
    .pin_a = PA15,
    .pin_b = PB3,
    .timer = TIM2,
    .dir_cw = true,
    .rad_per_tick = RAD_PER_TICK,
    .frame_id = "fr_wheel_joint",
};

inline constexpr HardwareEncoderConfig enc_rl_config = {
    .pin_a = PC6,
    .pin_b = PC7,
    .timer = TIM3,
    .dir_cw = false,
    .rad_per_tick = RAD_PER_TICK,
    .frame_id = "rl_wheel_joint",
};

inline constexpr HardwareEncoderConfig enc_rr_config = {
    .pin_a = PD12,
    .pin_b = PD13,
    .timer = TIM4,
    .dir_cw = true,
    .rad_per_tick = RAD_PER_TICK,
    .frame_id = "rr_wheel_joint",
};

// ────────────── Fan ──────────────
#define FAN_PP_PIN PC13
#define FAN_PWM_PIN PB_0_ALT1
#define FAN_PWM_TIMER TIM3
#define FAN_PWM_CHANNEL 3
#define FAN_PWM_FREQUENCY 1000
#define FAN_TEMP_THRSH_UP 35
#define FAN_TEMP_THRSH_DOWN 30

// ────────────── IMU ──────────────
static constexpr uint8_t IMU_I2C_SDA = PF0;
static constexpr uint8_t IMU_I2C_SCL = PF1;

inline TwoWire imu_i2c(IMU_I2C_SDA, IMU_I2C_SCL);
inline constexpr ImuBno055Config imu_bno055_config = {
    .bus = &imu_i2c,
    .i2c_addr = 0x29,
    .sensor_id = 0x37,
    .int_pin = PF2,
    .axis_config = Adafruit_BNO055::REMAP_CONFIG_P1,
};

// ────────────── LEDs ──────────────
static constexpr uint8_t RED_LED = PE4;
static constexpr uint8_t GRN_LED = PE3;

inline constexpr LedIndicatorConfig led_status_config = {
    .pin = RED_LED,
    .initial_state = HIGH,
    .blink_period_ms = 500,
    .label = "STATUS",
};

// ────────────── ROS ──────────────
static constexpr const char* NODE_NAME = "rosbot_mcu";
static constexpr uint16_t DOMAIN_ID = 255;  // 255 inherit from Micro ROS Agent
static constexpr uint32_t PING_TIMEOUT_MS = 100;
static constexpr uint8_t PING_ATTEMPTS = 3;

// ────────────── SBC Interface ──────────────
static constexpr uint32_t SBC_CONNECT_TIMEOUT_MS = 10;
static constexpr uint32_t POWEROFF_DELAY_MS = 5000;