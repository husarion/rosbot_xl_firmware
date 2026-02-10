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
#include <sensor_msgs/msg/imu.h>

#include "sensors/imu.hpp"
#include "rtos.hpp"

class ImuPublisher {
 public:
  rcl_ret_t init(rcl_node_t& node, const char* topic_name) {
    initMsg();
    return rclc_publisher_init_best_effort(
        &pub_, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        topic_name);
  }

  void publish() {
    ImuData data;
    if (xQueueReceive(rtos::ImuQueue, &data, 0) != pdPASS) {
      return;
    }

    fillMsg(data);
    rcl_publish(&pub_, &msg_, NULL);
  }

  void fini(rcl_node_t& node) { rcl_publisher_fini(&pub_, &node); }

 private:
  rcl_publisher_t pub_;
  sensor_msgs__msg__Imu msg_;

  void initMsg() {
    memset(&msg_, 0, sizeof(msg_));
    msg_.header.frame_id =
        micro_ros_string_utilities_set(msg_.header.frame_id, "imu_link");
  }

  void fillMsg(const ImuData& d) {
    msg_.header.stamp.sec = d.timestamp_ns / 1000000000LL;
    msg_.header.stamp.nanosec = d.timestamp_ns % 1000000000LL;

    msg_.orientation.x = d.orientation[0];
    msg_.orientation.y = d.orientation[1];
    msg_.orientation.z = d.orientation[2];
    msg_.orientation.w = d.orientation[3];

    msg_.angular_velocity.x = d.angular_velocity[0];
    msg_.angular_velocity.y = d.angular_velocity[1];
    msg_.angular_velocity.z = d.angular_velocity[2];

    msg_.linear_acceleration.x = d.acceleration[0];
    msg_.linear_acceleration.y = d.acceleration[1];
    msg_.linear_acceleration.z = d.acceleration[2];
  }
};
