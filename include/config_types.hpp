#pragma once

#include <Arduino.h>
#include <array>

// ================= MOTOR IDENTIFICATION =================

enum class MotorID : uint8_t {
  FR = 0,
  RR = 1,
  RL = 2,
  FL = 3,
  COUNT = 4
};

enum class Direction : bool { CW = false, CCW = true };

// ================= STRUCTURES =================
struct MotorPins {
  uint8_t pwm;
  uint8_t in_a;
  uint8_t in_b;
  uint8_t enc_a;
  uint8_t enc_b;
};

struct MotorConfig {
  MotorID id;
  MotorPins pins;
  TIM_TypeDef* encoder_timer;
  Direction direction;
  const char* joint_name;
};

namespace motors {
// ================= CONFIG DECLARATION =================
extern const std::array<MotorConfig, static_cast<size_t>(MotorID::COUNT)> CONFIG;

// ================= GETTERS =================
constexpr const MotorConfig& getConfig(MotorID id) {
  return CONFIG[static_cast<size_t>(id)];
}

constexpr uint8_t getPwmPin(MotorID id) {
  return getConfig(id).pins.pwm;
}

constexpr uint8_t getInAPin(MotorID id) {
  return getConfig(id).pins.in_a;
}

constexpr uint8_t getInBPin(MotorID id) {
  return getConfig(id).pins.in_b;
}

constexpr uint8_t getEncAPin(MotorID id) {
  return getConfig(id).pins.enc_a;
}

constexpr uint8_t getEncBPin(MotorID id) {
  return getConfig(id).pins.enc_b;
}

constexpr TIM_TypeDef* getEncoderTimer(MotorID id) {
  return getConfig(id).encoder_timer;
}

constexpr Direction getDirection(MotorID id) {
  return getConfig(id).direction;
}

constexpr const char* getJointName(MotorID id) {
  return getConfig(id).joint_name;
}

}  // namespace motors


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