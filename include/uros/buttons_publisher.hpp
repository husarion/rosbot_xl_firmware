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
#include <micro_ros_utilities/string_utilities.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/u_int8.h>

#include "tasks.hpp"

class ButtonsPublisher {
 public:
  rcl_ret_t init(rcl_node_t& node, const char* topic_name) {
    initMsg();
    return rclc_publisher_init_best_effort(
        &pub_, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        topic_name);
  }

  void publish() {
    uint8_t state = 0;
    state |= (digitalRead(PUSH_BUTTON1) == LOW) << 0;
    state |= (digitalRead(PUSH_BUTTON2) == LOW) << 1;

    if (state != last_state) {
      last_state = state;
      msg_.data = state;
      rcl_publish(&pub_, &msg_, NULL);
    }
  }

  void fini(rcl_node_t& node) { rcl_publisher_fini(&pub_, &node); }

 private:
  rcl_publisher_t pub_;
  std_msgs__msg__UInt8 msg_;
  uint8_t last_state = 0;

  void initMsg() { memset(&msg_, 0, sizeof(msg_)); }
};
