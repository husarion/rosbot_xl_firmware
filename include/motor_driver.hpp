#pragma once

// ============================================================================
// MOTOR DRIVER CLASS - DRV8848PWPR Controller (Hi-Z Control Scheme)
// Thread-safe motor control with encoder feedback for FreeRTOS + micro-ROS
// ============================================================================

#include <Arduino.h>
#include <HardwareTimer.h>

#include <atomic>

#include "STM32FreeRTOS.h"
#include "encoder.hpp"
#include "pid.hpp"

// Forward declarations
class MotorDriver;

// ============================================================================
// MOTOR DIRECTION ENUM
// ============================================================================
enum class MotorMovement : uint8_t { FORWARD, REVERSE, BRAKE, COAST };

// ============================================================================
// SINGLE MOTOR CONTROLLER - Hi-Z Control Scheme
// ============================================================================
class SingleMotor {
 public:
  SingleMotor() = default;

  void init(uint8_t pwm_pin, uint8_t in_a_pin, uint8_t in_b_pin,
            uint8_t enc_a_pin, uint8_t enc_b_pin, TIM_TypeDef* enc_timer,
            Direction dir);

  // Control methods
  void setVelocity(const float vel);
  void stop();   // Coast stop (PWM = 0)
  void brake();  // Active braking

  // State getters - thread-safe
  float getPosition() const { return encoder_.getPosition(); }
  float getVelocity() { return encoder_.getVelocity(); }
  float getEffort() const {
    return current_effort_.load(std::memory_order_relaxed);
  }
  float getTargetVelocity() const {
    return target_velocity_.load(std::memory_order_relaxed);
  }
  void setTargetVelocity(float vel) {
    target_velocity_.store(vel, std::memory_order_relaxed);
  }

  // Control loop - call from RTOS task
  void update(float dt, bool move = true);

  // Configuration
  void resetPID() { pid_.reset(); }

  // Encoder access for ISR registration
  Encoder& getEncoder() { return encoder_; }

 private:
  void setMovement(MotorMovement dir);
  void applyPWM(float duty);

  // Pin configuration
  uint8_t pwm_pin_ = 0;
  uint16_t pwm_arr_ = 0;
  uint8_t in_a_pin_ = 0;
  uint8_t in_b_pin_ = 0;
  Direction dir_ = Direction::CW;

  // Hardware
  Encoder encoder_;
  PIDController pid_;
  HardwareTimer* pwm_timer_ = nullptr;
  uint32_t pwm_channel_ = 0;

  // State (atomic for thread safety)
  std::atomic<float> target_velocity_{0.0f};
  std::atomic<float> current_effort_{0.0f};

  // Current direction
  MotorMovement current_movement_ = MotorMovement::COAST;
};

// ============================================================================
// MOTOR DRIVER MANAGER - Singleton
// ============================================================================
class MotorDriver {
 public:
  static MotorDriver& getInstance();

  // Initialization
  void init();
  void enableDrivers();
  void disableDrivers();
  bool checkFaults();
  bool isDriversEnabled() const { return drivers_enabled_.load(); }

  // Motor access
  SingleMotor& getMotor(MotorID id);
  SingleMotor& operator[](MotorID id) { return getMotor(id); }

  // Bulk operations (thread-safe)
  void setVelocities(const float velocities[4]);
  void setVelocities(float fr, float rr, float rl, float fl);
  void stopAll();
  void brakeAll();

  // State retrieval
  void getPositions(float positions[4]);
  void getVelocities(float velocities[4]);
  void getEfforts(float efforts[4]);
  void getState(MotorState states[4]);

  // Control loop
  void update();

  // Watchdog
  void feedWatchdog();
  bool isWatchdogExpired() const;
  void disableWatchdog() { watchdog_enabled_.store(false); }
  void enableWatchdog() { watchdog_enabled_.store(true); }

 private:
  MotorDriver() = default;
  MotorDriver(const MotorDriver&) = delete;
  MotorDriver& operator=(const MotorDriver&) = delete;

  SingleMotor motors_[static_cast<uint8_t>(MotorID::COUNT)];

  // Watchdog state
  std::atomic<uint32_t> last_command_time_{0};
  std::atomic<bool> watchdog_enabled_{false};
  std::atomic<bool> drivers_enabled_{false};

  uint32_t last_update_time_ = 0;

  // FreeRTOS synchronization
  SemaphoreHandle_t mutex_ = nullptr;

  static constexpr uint32_t WATCHDOG_TIMEOUT_MS = 500;
};

// Global accessor
extern MotorDriver& Motors;
