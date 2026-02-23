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
#include <sensor_msgs/msg/imu.h>

#include "imu_interface.hpp"
#include "publisher_interface.hpp"
#include "../utils.hpp"

struct ImuStamped {
  ImuData data;
  int64_t timestamp_ns;
};

struct ImuPublisherConfig {
  const char* topic;
  QueueHandle_t& queue;
  const char* frame_id;
};

class ImuPublisher : public PublisherInterface {
 public:
  explicit ImuPublisher(ImuPublisherConfig cfg)
      : PublisherInterface(cfg.topic), cfg_(cfg) {}

  rcl_ret_t init(rcl_node_t& node, rcl_allocator_t& allocator) override {
    initMsg();
    return rclc_publisher_init_best_effort(
        &pub_, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        topic_);
  }

  void publish() override {
    if (xQueueReceive(cfg_.queue, &data_, 0) != pdPASS) return;
    fillMsg(data_);
    RC_SKIP(rcl_publish(&pub_, &msg_, NULL));
  }

  void fini(rcl_node_t& node) override { RC_SKIP(rcl_publisher_fini(&pub_, &node)); }

 private:
  rcl_publisher_t pub_;
  sensor_msgs__msg__Imu msg_;
  ImuStamped data_;
  ImuPublisherConfig cfg_;

  void initMsg() {
    memset(&msg_, 0, sizeof(msg_));
    msg_.header.frame_id = micro_ros_string_utilities_init(cfg_.frame_id);
  }

  void fillMsg(const ImuStamped& d) {
    msg_.header.stamp.sec = d.timestamp_ns / 1000000000LL;
    msg_.header.stamp.nanosec = d.timestamp_ns % 1000000000LL;

    msg_.orientation.x = d.data.orientation[0];
    msg_.orientation.y = d.data.orientation[1];
    msg_.orientation.z = d.data.orientation[2];
    msg_.orientation.w = d.data.orientation[3];

    msg_.angular_velocity.x = d.data.angular_velocity[0];
    msg_.angular_velocity.y = d.data.angular_velocity[1];
    msg_.angular_velocity.z = d.data.angular_velocity[2];

    msg_.linear_acceleration.x = d.data.acceleration[0];
    msg_.linear_acceleration.y = d.data.acceleration[1];
    msg_.linear_acceleration.z = d.data.acceleration[2];
  }
};
