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
#include <std_msgs/msg/u_int8.h>

#include "config.hpp"
#include "publisher_interface.hpp"

class ButtonsPublisher : public PublisherInterface {
 public:
  ButtonsPublisher(const char* topic, const uint8_t* pins, uint8_t count)
      : PublisherInterface(topic), pins_(pins), num_buttons_(count) {}

  rcl_ret_t init(rcl_node_t& node, rcl_allocator_t& allocator) override {
    memset(&msg_, 0, sizeof(msg_));
    return rclc_publisher_init_best_effort(
        &pub_, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        topic_);
  }

  void publish() override {
    uint8_t state = 0;

    for (uint8_t i = 0; i < num_buttons_; ++i) {
      state |= (digitalRead(pins_[i]) == LOW) << i;
    }

    if (state != last_state_) {
      last_state_ = state;
      msg_.data = state;
      rcl_publish(&pub_, &msg_, NULL);
    }
  }

  void fini(rcl_node_t& node) override { rcl_publisher_fini(&pub_, &node); }

 private:
  const uint8_t* pins_;
  uint8_t num_buttons_;
  rcl_publisher_t pub_;
  std_msgs__msg__UInt8 msg_;
  uint8_t last_state_ = 0;
};
