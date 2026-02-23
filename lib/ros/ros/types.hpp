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

/// One ROS subscription for dynamic registration.
struct SubscriptionEntry {
  rcl_subscription_t sub = {};
  void* msg;
  const rosidl_message_type_support_t* type_support;
  const char* topic_name;
  rclc_subscription_callback_t callback;
  bool best_effort;
};

/// One ROS service for dynamic registration.
struct ServiceEntry {
  rcl_service_t srv;
  void* request;
  void* response;
  const rosidl_service_type_support_t* type_support;
  const char* topic_name;
  rclc_service_callback_t callback;
};
