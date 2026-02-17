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

#include <micro_ros_utilities/string_utilities.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/range.h>

#include "rtos.hpp"

inline const std::array<const char*, 4> FRAME_IDS = {
    "fl_range", "fr_range", "rl_range", "rr_range"};

class RangePublisher {
 public:
  rcl_ret_t init(rcl_node_t& node, const char* topic_name) {
    initMsg();
    return rclc_publisher_init_best_effort(
        &pub_, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        topic_name);
  }

  void publish() {
    if (xQueueReceive(rtos::RangesQueue, &data_, 0) != pdPASS) {
      return;
    }

    msg_.header.stamp.sec = data_.timestamp_ns / 1000000000LL;
    msg_.header.stamp.nanosec = data_.timestamp_ns % 1000000000LL;

    for (uint8_t i = 0; i < data_.data.count; i++) {
      msg_.header.frame_id.data = const_cast<char*>(FRAME_IDS[i]);
      float range = data_.data.range[i];
      if (range > msg_.max_range) {
        msg_.range = INFINITY;
      } else if (range < msg_.min_range) {
        msg_.range = -INFINITY;
      } else {
        msg_.range = range;
      }
      rcl_publish(&pub_, &msg_, NULL);
    }
  }

  void fini(rcl_node_t& node) { rcl_publisher_fini(&pub_, &node); }

 private:
  rcl_publisher_t pub_;
  sensor_msgs__msg__Range msg_;
  RangesStamped data_;

  void initMsg() {
    memset(&msg_, 0, sizeof(msg_));
    msg_.radiation_type = sensor_msgs__msg__Range__INFRARED;
    msg_.field_of_view = 0.26;
    msg_.min_range = 0.01;
    msg_.max_range = 0.9;
    msg_.range = NAN;
    msg_.variance = 0.0f;
  }
};
