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
#include <STM32FreeRTOS.h>

#include "bsp.hpp"

// MOTORS TIMEBASE TIMER
#define TIMEBASE_TIMER TIM6
#define TIMEBASE_TIMER_FREQ 10000
#define TIMEBASE_TIMER_CLOCKSOURCE_FREQ 168000000
#define TIMEBASE_TIMER_PSC \
  ((TIMEBASE_TIMER_CLOCKSOURCE_FREQ / TIMEBASE_TIMER_FREQ) / 2)
#define TIMEBASE_TIMER_OVERFLOW_VALUE 0xFFFF

// PID PARAMETERS
#define PID_FREQ 100       // max 1000Hz
#define PID_DEFAULT_KP 49  // KP * 1000
#define PID_DEFAULT_KI 8   // KI * 1000
#define PID_DEFAULT_KD 0
#define MAX_ERR_SUM (1000000 / PID_DEFAULT_KI)

// MOTORS ENCODERS PARAMETERS
#define ENC_RESOLUTION 64
#define ENCODER_COUNTER_MAX_VALUE 0xFFFF
#define ENCODER_COUNTER_OFFSET (ENCODER_COUNTER_MAX_VALUE / 2)
#define TICK_PER_REAR (ENC_RESOLUTION * GEARBOX_RATIO)
#define TICK_PER_RADIAN TICK_PER_REAR / (2 * PI)
#define TICK_PER_RADIAN_X_1000 TICK_PER_RADIAN * 1000
#define TICK_TO_RAD_X_1000(arg) (int64_t((arg) * 1000 * 2 * PI) / TICK_PER_REAR)

// HARDWARE DEFINES
#define MOTORS_SETPOINT_TIMEOUT 3000  // ms
#define MOTORS_PWM_FREQUENCY 15000    // Hz
#define GEARBOX_RATIO 50
#define MAX_ANG_VEL 20000  // rad/s * 1000
#define MAX_CURRENT 0x01
#define REDUCED_CURRENT 0x00
#define RAMP_ACCELERATION 2000  // rad/s^2 * 1000
#define RAMP_FLAG false         // if true - use ramp, it false - without ramp

/* TYPE DEF */
typedef struct {
  uint8_t size = 4;
  double velocity[4];
  double position[4];
} motor_joint_state_t;

void SetMaxMotorsCurrent(uint32_t Ilim1_, uint32_t Ilim2_, uint32_t Ilim3_,
                         uint32_t Ilim4_);

class TimebaseTimerClass {
 public:
  TimebaseTimerClass();
  explicit TimebaseTimerClass(TIM_TypeDef* arg_timer);
  ~TimebaseTimerClass();
  uint64_t GetAbsTimeValue();
  uint64_t GetTimeChange(uint64_t* arg_last_time);

 private:
  HardwareTimer* timebase_timer_ = 0;
  uint64_t time_counter_ = 0;
};

class MotorClass {
 public:
  MotorClass();
  MotorClass(uint32_t arg_pwm_pin, TIM_TypeDef* arg_pwm_timer,
             uint8_t arg_pwm_tim_channel, uint32_t arg_a_channel_motor_pin,
             uint32_t arg_b_channel_motor_pin, TIM_TypeDef* arg_encoder_timer,
             uint32_t arg_a_channel_encoder_pin,
             uint32_t arg_b_channel_encoder_pin, int8_t arg_default_direction,
             TimebaseTimerClass* arg_timebase_timer);
  ~MotorClass();
  // basic motor control methods
  void SoftStop(void);
  void EmgStop(void);
  void SetMove(int32_t arg_velocity);
  void SetPwm(uint32_t arg_value);
  void SetCurrentLimit(uint8_t arg_current_mode);
  // motor feedback methods
  int32_t GetVelocity(void);
  int64_t GetWheelAbsPosition(void);
  int16_t GetWheelAngle(void);
  int8_t GetDefaultDirection(void);
  // PID methods
  void SetPidSetpoint(int32_t arg_setpoint);
  void SetPidSetpoint(float arg_setpoint);
  void PidLoopHandler();
  void PidLoopHandler(int32_t arg_setpoint);
  void PidLoopHandler(float arg_setpoint);
  void SetPidParameters(uint16_t arg_kp_gain, uint16_t arg_ki_gain,
                        uint16_t arg_kd_gain);
  void SetPidAcceleration(uint16_t arg_ramp_acceleration);

 private:
  int32_t VelocityUpdate(void);
  uint32_t GetPwmTimerOverflow(void);
  int64_t GetEncoderValue(void);
  HardwareTimer* pwm_timer_;
  HardwareTimer* encoder_timer_;
  TimebaseTimerClass* timebase_tim_;
  int64_t last_encoder_value_;
  int64_t actual_encoder_value_;
  int64_t encoder_value_;
  uint64_t last_time_;
  uint64_t time_change_;
  uint16_t acceleration_;
  int32_t input_;
  int32_t actual_input_;
  int32_t actual_velocity_;
  int32_t last_error_;
  int32_t error_sum_;
  int32_t actual_error_;
  uint16_t kp_gain_ = PID_DEFAULT_KP;
  uint16_t ki_gain_ = PID_DEFAULT_KI;
  uint16_t kd_gain_ = PID_DEFAULT_KD;
  int64_t max_error_sum_ = (1000000 / PID_DEFAULT_KI);
  int32_t output_;
  int8_t default_direction_;
  uint8_t a_channel_motor_pin_;
  uint8_t b_channel_motor_pin_;
  uint8_t a_channel_encoder_pin_;
  uint8_t b_channel_encoder_pin_;
  uint8_t pwm_timer_channel_;
  uint8_t pwm_pin_;

 protected:
  ;
};

extern MotorClass wheel_motors[4];
