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
#include <sensor_msgs/msg/range.h>

#include "../utils.hpp"
#include "publisher_interface.hpp"
#include "range_array.hpp"

struct RangesStamped {
  RangesData data;
  int64_t timestamp_ns;
};

struct RangePublisherConfig {
  const char* topic;
  QueueHandle_t& queue;
  float fov;
  float min_range;
  float max_range;
};

class RangePublisher : public PublisherInterface {
 public:
  explicit RangePublisher(RangePublisherConfig cfg)
      : PublisherInterface(cfg.topic), cfg_(cfg) {}

  rcl_ret_t init(rcl_node_t& node, rcl_allocator_t& allocator) override {
    initMsg();
    return rclc_publisher_init_best_effort(
        &pub_, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        topic_);
  }

  void publish() override {
    if (xQueueReceive(cfg_.queue, &data_, 0) != pdPASS) return;

    msg_.header.stamp.sec = data_.timestamp_ns / 1000000000LL;
    msg_.header.stamp.nanosec = data_.timestamp_ns % 1000000000LL;

    for (uint8_t i = 0; i < data_.data.count; i++) {
      msg_.header.frame_id =
          micro_ros_string_utilities_init(data_.data.frame_id[i]);

      float range = data_.data.range[i];
      if (range > msg_.max_range)
        msg_.range = INFINITY;
      else if (range < msg_.min_range)
        msg_.range = -INFINITY;
      else
        msg_.range = range;

      RC_SKIP(rcl_publish(&pub_, &msg_, NULL));
    }
  }

  void fini(rcl_node_t& node) override {
    RC_SKIP(rcl_publisher_fini(&pub_, &node));
  }

 private:
  rcl_publisher_t pub_;
  sensor_msgs__msg__Range msg_;
  RangesStamped data_;
  RangePublisherConfig cfg_;

  void initMsg() {
    memset(&msg_, 0, sizeof(msg_));
    msg_.radiation_type = sensor_msgs__msg__Range__INFRARED;

    msg_.field_of_view = cfg_.fov;
    msg_.min_range = cfg_.min_range;
    msg_.max_range = cfg_.max_range;
    msg_.range = NAN;
    msg_.variance = 0.0f;
  }
};
