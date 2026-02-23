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

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

#include "publishers/publisher_interface.hpp"
#include "serial_manager.hpp"
#include "types.hpp"
#include "utils.hpp"

struct RosNodeConfig {
  const char* node_name;  // e.g. "rosbot_mcu"
  uint8_t domain_id;      // 255 = inherit from agent

  PublisherInterface** publishers;
  size_t pub_count;

  SubscriptionEntry* subscriptions;
  size_t sub_count;

  ServiceEntry* services;
  size_t srv_count;

  uint8_t ping_attempts = 3;
  uint16_t ping_timeout_ms = 50;
};

class RosNode {
 public:
  enum State : uint8_t { WAITING, AGENT_AVAILABLE, CONNECTED, DISCONNECTED };

  RosNode() = default;
  explicit RosNode(const RosNodeConfig& cfg) : cfg_(cfg) {}

  void transportInit(const SerialConfig& serial);
  bool pingAgent();

  /// State machine: WAITING → AGENT_AVAILABLE → CONNECTED ⇄ DISCONNECTED
  void loop();

  /// Publish all registered publishers + spin executor.
  void publishLoop();

  State state() const { return state_; }
  bool isConnected() const { return state_ == CONNECTED; }

  void setNamespace(const char* ns) { ns_ = ns; };

 private:
  bool createEntities();
  void destroyEntities();

  RosNodeConfig cfg_ = {};
  State state_ = WAITING;
  const char* ns_ = {};

  // ROS2 internals
  rcl_init_options_t init_options_;
  rclc_support_t support_;
  rcl_allocator_t allocator_;
  rcl_node_t node_;
  rclc_executor_t executor_;
};

/// Global ROS node — defined in board-specific ros_entities.cpp
extern RosNode g_ros_node;
