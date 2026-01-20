#pragma once

#include <Arduino.h>
#include <array>

#include "config_types.hpp"

namespace motors {

constexpr uint8_t RIGHT_WHEELS_SLEEP = PC13;
constexpr uint8_t RIGHT_WHEELS_FAULT = PE0;
constexpr uint8_t LEFT_WHEELS_SLEEP = PC14;
constexpr uint8_t LEFT_WHEELS_FAULT = PE1;

inline const std::array<MotorConfig, static_cast<size_t>(MotorID::COUNT)> CONFIG = {{
  {
    MotorID::FL,
    { PF9, PE5, PE6, PB6, PB7 },
    TIM4,
    Direction::CCW,
    "fl_wheel_joint"
  },

  {
    MotorID::FR,
    { PF6, PG10, PG11, PA0, PA1 },
    TIM2,
    Direction::CW,
    "fr_wheel_joint"
  },

  {
    MotorID::RL,
    { PF8, PC15, PF2, PB4, PA7 },
    TIM3,
    Direction::CCW,
    "rl_wheel_joint"
  },

  {
    MotorID::RR,
    { PF7, PD3, PD4, PC6, PC7 },
    TIM8,
    Direction::CW,
    "rr_wheel_joint"
  }
}};
}

// ============================================================================
// ROBOT PHYSICAL PARAMETERS
// ============================================================================
namespace RobotParams {

// Wheel & Drivetrain
constexpr float GEAR_RATIO = 34.014f;
constexpr uint16_t ENCODER_CPR = 48;
constexpr float TICKS_PER_REVOLUTION = ENCODER_CPR * GEAR_RATIO;

// Computed constants
constexpr float RAD_PER_TICK = (2.0f * PI) / TICKS_PER_REVOLUTION;

}  // namespace RobotParams

// ============================================================================
// CONTROL PARAMETERS
// ============================================================================
namespace ControlParams {

// Timing
constexpr uint32_t CONTROL_LOOP_PERIOD_MS = 10;

// Target PWM frequency (ultrasonic - inaudible)
constexpr uint32_t MOTOR_PWM_FREQ = 20000;  // 20 kHz

// Velocity limits (rad/s)
constexpr float MAX_VELOCITY = 30.0f;
constexpr float MIN_VELOCITY = 0.1f;

}  // namespace ControlParams

