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
#include "range_vl53l0.hpp"
#include "ros/publishers/battery_publisher.hpp"
#include "ros/publishers/buttons_publisher.hpp"
#include "ros/publishers/imu_publisher.hpp"
#include "ros/publishers/joint_state_publisher.hpp"
#include "ros/publishers/range_publisher.hpp"
#include "serial_manager.hpp"

// ────────────── Battery ──────────────
inline constexpr BatteryAdcConfig battery_adc_config = {
    .adc_pin = PA5,
    .v_ref = 3.3f,
    .v_min = 9.6f,
    .v_max = 12.6f,
    .divider = (5.6e4 + 1.0e4) / 1.0e4,
    .correction = 0.986f};

// ────────────── Buttons ──────────────
static constexpr uint8_t PUSH_BUTTON1 = PG12;
static constexpr uint8_t PUSH_BUTTON2 = PG13;

// ────────────── Encoders ──────────────
constexpr float GEAR_RATIO = 34.0f;
constexpr uint16_t ENCODER_CPR = 48;
constexpr float TICKS_PER_REVOLUTION = ENCODER_CPR * GEAR_RATIO;
constexpr float RAD_PER_TICK = (2.0f * PI) / TICKS_PER_REVOLUTION;

inline constexpr HardwareEncoderConfig enc_fl_config = {
    .pin_a = PB6,
    .pin_b = PB7,
    .timer = TIM4,
    .dir_cw = false,
    .rad_per_tick = RAD_PER_TICK,
    .frame_id = "fl_wheel_joint",
};

inline constexpr HardwareEncoderConfig enc_fr_config = {
    .pin_a = PA0,
    .pin_b = PA1,
    .timer = TIM2,
    .dir_cw = true,
    .rad_per_tick = RAD_PER_TICK,
    .frame_id = "fr_wheel_joint",
};

inline constexpr HardwareEncoderConfig enc_rl_config = {
    .pin_a = PB4,
    .pin_b = PA7,
    .timer = TIM3,
    .dir_cw = false,
    .rad_per_tick = RAD_PER_TICK,
    .frame_id = "rl_wheel_joint",
};

inline constexpr HardwareEncoderConfig enc_rr_config = {
    .pin_a = PC6,
    .pin_b = PC7,
    .timer = TIM8,
    .dir_cw = true,
    .rad_per_tick = RAD_PER_TICK,
    .frame_id = "rr_wheel_joint",
};

// ────────────── IMU ──────────────
static constexpr uint8_t IMU_POWER_ON = PG4;
static constexpr uint8_t IMU_I2C_SDA = PC9;
static constexpr uint8_t IMU_I2C_SCL = PA8;

inline TwoWire imu_i2c(IMU_I2C_SDA, IMU_I2C_SCL);
inline constexpr ImuBno055Config imu_bno055_config = {
    .bus = &imu_i2c,
    .i2c_addr = 0x29,
    .sensor_id = 0xA0,
    .int_pin = PA6,
    .axis_config = Adafruit_BNO055::REMAP_CONFIG_P0,
};

// ────────────── LEDs ──────────────
static constexpr uint8_t RED_LED = PE2;
static constexpr uint8_t GRN_LED = PE3;
static constexpr uint8_t GRN_LED2 = PE4;

inline constexpr LedIndicatorConfig led_status_config = {
    .pin = RED_LED,
    .initial_state = HIGH,
    .blink_period_ms = 500,
    .label = "STATUS",
};

// ────────────── Motors ──────────────
constexpr uint32_t MOTOR_PWM_FREQ = 20000;  // 20 kHz
constexpr float MAX_VELOCITY = 30.0f;
constexpr float MIN_VELOCITY = 1.0f;

inline constexpr DriverGroupConfig right_motors_driver = {PC13, PE0};
inline constexpr DriverGroupConfig left_motors_driver = {PC14, PE1};
inline constexpr DriverGroupConfig driver_groups[] = {
    right_motors_driver,
    left_motors_driver,
};

inline constexpr MotorDrv8848Config motor_fl_config = {
    .pwm_pin = PF9,
    .in_a_pin = PE5,
    .in_b_pin = PE6,
    .dir_cw = false,
    .max_velocity = MAX_VELOCITY,
    .min_velocity = MIN_VELOCITY,
    .pwm_freq = MOTOR_PWM_FREQ,
    .frame_id = "fl_wheel_joint",
};

inline constexpr MotorDrv8848Config motor_fr_config = {
    .pwm_pin = PF6,
    .in_a_pin = PG10,
    .in_b_pin = PG11,
    .dir_cw = true,
    .max_velocity = MAX_VELOCITY,
    .min_velocity = MIN_VELOCITY,
    .pwm_freq = MOTOR_PWM_FREQ,
    .frame_id = "fr_wheel_joint",
};

inline constexpr MotorDrv8848Config motor_rl_config = {
    .pwm_pin = PF8,
    .in_a_pin = PC15,
    .in_b_pin = PF2,
    .dir_cw = false,
    .max_velocity = MAX_VELOCITY,
    .min_velocity = MIN_VELOCITY,
    .pwm_freq = MOTOR_PWM_FREQ,
    .frame_id = "rl_wheel_joint",
};

inline constexpr MotorDrv8848Config motor_rr_config = {
    .pwm_pin = PF7,
    .in_a_pin = PD3,
    .in_b_pin = PD4,
    .dir_cw = true,
    .max_velocity = MAX_VELOCITY,
    .min_velocity = MIN_VELOCITY,
    .pwm_freq = MOTOR_PWM_FREQ,
    .frame_id = "rr_wheel_joint",
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
    .compensation_up_to_speed = 4.0f,
};

// ────────────── Ranges ──────────────
static constexpr uint8_t RANGE_I2C_SDA = PB9;
static constexpr uint8_t RANGE_I2C_SCL = PB8;

inline TwoWire range_i2c(RANGE_I2C_SDA, RANGE_I2C_SCL);
inline constexpr RangeVl53l0xConfig range_fl_config = {
    .bus = &range_i2c,
    .xshut_pin = PD8,
    .i2c_address = 0x30,
    .frame_id = "fl_range",
};

inline constexpr RangeVl53l0xConfig range_fr_config = {
    .bus = &range_i2c,
    .xshut_pin = PB1,
    .i2c_address = 0x31,
    .frame_id = "fr_range",
};

inline constexpr RangeVl53l0xConfig range_rl_config = {
    .bus = &range_i2c,
    .xshut_pin = PD10,
    .i2c_address = 0x32,
    .frame_id = "rl_range",
};

inline constexpr RangeVl53l0xConfig range_rr_config = {
    .bus = &range_i2c,
    .xshut_pin = PD9,
    .i2c_address = 0x33,
    .frame_id = "rr_range",
};

// ────────────── ROS ──────────────
static constexpr const char* NODE_NAME = "rosbot_mcu";
static constexpr uint16_t DOMAIN_ID = 255;  // 255 inherit from Micro ROS Agent
static constexpr uint32_t PING_TIMEOUT_MS = 100;
static constexpr uint8_t PING_ATTEMPTS = 3;

// ────────────── Publishers ──────────────
inline QueueHandle_t battery_queue;
inline QueueHandle_t imu_queue;
inline QueueHandle_t joint_state_queue;
inline QueueHandle_t ranges_queue;

static constexpr uint8_t BATTERY_NUM_CELLS = 3;
static constexpr float BATTERY_CELL_CAPACITY = 2.6f;  // Ah
static constexpr float BATTERY_DESIGN_CAPACITY =
    BATTERY_NUM_CELLS * BATTERY_CELL_CAPACITY;
inline constexpr BatteryPublisherConfig battery_pub_config = {
    .topic = "battery",
    .queue = battery_queue,
    .frame_id = "base_link",
    .design_capacity = BATTERY_DESIGN_CAPACITY,
    .num_cells = BATTERY_NUM_CELLS,
};

inline constexpr uint8_t buttons_pins[2] = {PUSH_BUTTON2, PUSH_BUTTON1};
inline constexpr ButtonsPublisherConfig buttons_pub_config = {
    .topic = "buttons",
    .pins = buttons_pins,
    .num_buttons = 2,
};

inline constexpr ImuPublisherConfig imu_pub_config = {
    .topic = "_imu/data_raw",
    .queue = imu_queue,
    .frame_id = "imu_link",
};

inline constexpr JointStatePublisherConfig joint_state_pub_config = {
    .topic = "_motors_response",
    .queue = joint_state_queue,
    .frame_id = "base_link",
};

inline constexpr RangePublisherConfig range_pub_config = {
    .topic = "ranges",
    .queue = ranges_queue,
    .fov = 0.26f,
    .min_range = 0.01f,
    .max_range = 0.9f,
};

// ────────────── SBC Interface ──────────────
static constexpr uint32_t SBC_SERIAL_TIMEOUT_MS = 100;
static constexpr uint8_t SBC_STATUS = PG6;  // Detect RPi which is a pullup pin
static constexpr uint8_t RPI_CONSOLE = PG5;
static constexpr uint8_t RPI_BTN = PG7;

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
