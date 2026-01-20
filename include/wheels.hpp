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

// ============================================================================
// WHEELS INTERFACE - micro-ROS Integration Layer
// High-level wheel control with ROS message compatibility
// ============================================================================

#include <Arduino.h>

#include "config_types.hpp"
#include "motor_driver.hpp"

// micro-ROS includes
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32_multi_array.h>

// ============================================================================
// WHEELS CONTROLLER CLASS
// ============================================================================
class WheelsController {
 public:
  static WheelsController& getInstance();

  // Initialization
  void init();

  // Command handling
  void handleCommand(const float velocities[4]);

  // State publishing (for micro-ROS publisher)
  void fillJointStateMsg(sensor_msgs__msg__JointState* msg);

  // Task functions for FreeRTOS
  static void controlTask(void* params);

  // State access
  WheelsState getState();
  bool isEnabled() const { return enabled_; }
  void enable();
  void disable();

 private:
  WheelsController() = default;
  WheelsController(const WheelsController&) = delete;
  WheelsController& operator=(const WheelsController&) = delete;

  bool enabled_ = false;

  TaskHandle_t control_task_handle_ = nullptr;
  TaskHandle_t publish_task_handle_ = nullptr;

  // Cached state for thread-safe access
  WheelsState cached_state_;
  SemaphoreHandle_t state_mutex_ = nullptr;

  void updateCachedState();
  static constexpr uint32_t TASK_PERIOD_MS = 10;
};

// ============================================================================
// GLOBAL INSTANCE
// ============================================================================
extern WheelsController& Wheels;
