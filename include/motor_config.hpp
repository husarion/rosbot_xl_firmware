#pragma once

// ============================================================================
// MOTOR CONFIGURATION - DRV8848PWPR Driver (Hi-Z Control Scheme)
// Pin definitions and robot parameters for 4-wheel differential drive
// ============================================================================

#include <Arduino.h>

#include <array>

namespace MotorPins {

// ============================================================================
// Motor 1 (Front Right) - Driver 1, Channel A
// ============================================================================
constexpr uint8_t MOT1_PWM = PF6;  // TIM10_CH1
constexpr uint8_t MOT1A_IN = PG10;
constexpr uint8_t MOT1B_IN = PG11;
constexpr uint8_t MOT1A_ENC = PA0;
constexpr uint8_t MOT1B_ENC = PA1;

// ============================================================================
// Motor 2 (Rear Right) - Driver 1, Channel B
// ============================================================================
constexpr uint8_t MOT2_PWM = PF7;  // TIM11_CH1
constexpr uint8_t MOT2A_IN = PD3;
constexpr uint8_t MOT2B_IN = PD4;
constexpr uint8_t MOT2A_ENC = PC6;
constexpr uint8_t MOT2B_ENC = PC7;

// Driver 1 Control
constexpr uint8_t MOT12_SLEEP = PC13;
constexpr uint8_t MOT12_FAULT = PE0;

// ============================================================================
// Motor 3 (Rear Left) - Driver 2, Channel A
// ============================================================================
constexpr uint8_t MOT3_PWM = PF8;  // TIM13_CH1
constexpr uint8_t MOT3A_IN = PC15;
constexpr uint8_t MOT3B_IN = PF2;
constexpr uint8_t MOT3A_ENC = PB4;
constexpr uint8_t MOT3B_ENC = PA7;

// ============================================================================
// Motor 4 (Front Left) - Driver 2, Channel B
// ============================================================================
constexpr uint8_t MOT4_PWM = PF9;  // TIM14_CH1
constexpr uint8_t MOT4A_IN = PE5;
constexpr uint8_t MOT4B_IN = PE6;
constexpr uint8_t MOT4A_ENC = PB6;
constexpr uint8_t MOT4B_ENC = PB7;

// Driver 2 Control
constexpr uint8_t MOT34_SLEEP = PC14;
constexpr uint8_t MOT34_FAULT = PE1;

}  // namespace MotorPins

// ============================================================================
// MOTOR IDENTIFICATION
// ============================================================================
enum class MotorID : uint8_t {
  FRONT_RIGHT = 0,
  REAR_RIGHT = 1,
  REAR_LEFT = 2,
  FRONT_LEFT = 3,
  COUNT = 4
};

enum class Direction : bool { CW = false, CCW = true };

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

// Motor polarity
constexpr std::array<Direction, 4> MOTOR_DIRECTIONS = {
    Direction::CW, Direction::CW, Direction::CCW, Direction::CCW};
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

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================
namespace MotorUtils {

constexpr Direction getDirection(MotorID id) {
  return RobotParams::MOTOR_DIRECTIONS[static_cast<uint8_t>(id)];
}

constexpr const char* getJointName(MotorID id) {
  switch (id) {
    case MotorID::FRONT_LEFT:
      return "fl_wheel_joint";
    case MotorID::FRONT_RIGHT:
      return "fr_wheel_joint";
    case MotorID::REAR_LEFT:
      return "rl_wheel_joint";
    case MotorID::REAR_RIGHT:
      return "rr_wheel_joint";
    default:
      return "unknown";
  }
}

inline TIM_TypeDef* getEncoderTimer(MotorID id) {
  switch (id) {
    case MotorID::FRONT_RIGHT:
      return TIM2;
    case MotorID::REAR_RIGHT:
      return TIM8;
    case MotorID::REAR_LEFT:
      return TIM3;
    case MotorID::FRONT_LEFT:
      return TIM4;
    default:
      return nullptr;
  }
}

}  // namespace MotorUtils

// ============================================================================
// STATE STRUCTURES
// ============================================================================
struct MotorState {
  float position_rad;
  float velocity_rad_s;
  float effort;
  float target_velocity;
  uint32_t last_update_ms;
};

struct WheelsCommand {
  float velocity[4];
};

struct WheelsState {
  float position[4];
  float velocity[4];
  float effort[4];
};
