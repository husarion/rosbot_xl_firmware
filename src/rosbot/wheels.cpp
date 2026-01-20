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

#include "wheels.hpp"

#include <STM32FreeRTOS.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rmw_microros/rmw_microros.h>

#include "rtos.hpp"

// Global instance
WheelsController& Wheels = WheelsController::getInstance();

// ============================================================================
// WHEELS CONTROLLER IMPLEMENTATION
// ============================================================================

WheelsController& WheelsController::getInstance() {
  static WheelsController instance;
  return instance;
}

void WheelsController::init() {
  // Create state mutex
  state_mutex_ = xSemaphoreCreateMutex();

  // Initialize motor driver
  Motors.init();

  // Initialize cached state
  memset(&cached_state_, 0, sizeof(WheelsState));

  // Create control task (high priority)
  xTaskCreate(controlTask, "WheelsTask", 2048, this, configMAX_PRIORITIES - 1,
              &control_task_handle_);
}

void WheelsController::handleCommand(const float velocities[4]) {
  if (!enabled_) return;
  Motors.setVelocities(velocities);
}

void WheelsController::fillJointStateMsg(sensor_msgs__msg__JointState* msg) {
  if (msg == nullptr) return;

  // Update cached state first
  updateCachedState();

  if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
    // Set timestamp
    int64_t time_ns = rmw_uros_epoch_nanos();
    msg->header.stamp.sec = time_ns / 1000000000;
    msg->header.stamp.nanosec = time_ns % 1000000000;

    // Fill position data (order: FL, FR, RL, RR)
    msg->position.data[0] =
        cached_state_.position[static_cast<uint8_t>(MotorID::FL)];
    msg->position.data[1] =
        cached_state_.position[static_cast<uint8_t>(MotorID::FR)];
    msg->position.data[2] =
        cached_state_.position[static_cast<uint8_t>(MotorID::RL)];
    msg->position.data[3] =
        cached_state_.position[static_cast<uint8_t>(MotorID::RR)];

    // Fill velocity data
    msg->velocity.data[0] =
        cached_state_.velocity[static_cast<uint8_t>(MotorID::FL)];
    msg->velocity.data[1] =
        cached_state_.velocity[static_cast<uint8_t>(MotorID::FR)];
    msg->velocity.data[2] =
        cached_state_.velocity[static_cast<uint8_t>(MotorID::RL)];
    msg->velocity.data[3] =
        cached_state_.velocity[static_cast<uint8_t>(MotorID::RR)];

    // Fill effort data
    // msg->effort.data[0] =
    // cached_state_.effort[static_cast<uint8_t>(MotorID::FL)];
    // msg->effort.data[1] =
    // cached_state_.effort[static_cast<uint8_t>(MotorID::FR)];
    // msg->effort.data[2] =
    // cached_state_.effort[static_cast<uint8_t>(MotorID::RL)];
    // msg->effort.data[3] =
    // cached_state_.effort[static_cast<uint8_t>(MotorID::RR)];

    xSemaphoreGive(state_mutex_);
  }
}

void WheelsController::controlTask(void* params) {
  WheelsController* self = static_cast<WheelsController*>(params);
  TickType_t last_wake = xTaskGetTickCount();

  while (true) {
    if (self->enabled_) {
      Motors.update();
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_PERIOD_MS));
  }
}

WheelsState WheelsController::getState() {
  updateCachedState();

  WheelsState state;
  if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
    state = cached_state_;
    xSemaphoreGive(state_mutex_);
  }
  return state;
}

void WheelsController::enable() {
  Motors.enableDrivers();
  enabled_ = true;
}

void WheelsController::disable() {
  Motors.stopAll();
  Motors.disableDrivers();
  enabled_ = false;
}

void WheelsController::updateCachedState() {
  if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
    Motors.getPositions(cached_state_.position);
    Motors.getVelocities(cached_state_.velocity);
    Motors.getEfforts(cached_state_.effort);
    xSemaphoreGive(state_mutex_);
  }
}
