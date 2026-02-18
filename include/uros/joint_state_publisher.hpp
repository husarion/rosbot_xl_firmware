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
#include <sensor_msgs/msg/joint_state.h>

#include "motor_array.hpp"
#include "rtos.hpp"

class JointStatePublisher {
 public:
  rcl_ret_t init(rcl_node_t& node, const char* topic_name,
                 rcl_allocator_t& allocator) {
    initMsg(allocator);
    return rclc_publisher_init_best_effort(
        &pub_, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        topic_name);
  }

  void publish() {
    if (xQueueReceive(rtos::EncodersQueue, &data_, 0) != pdPASS) {
      return;
    }

    fillMsg(data_);
    rcl_publish(&pub_, &msg_, NULL);
  }

  void fini(rcl_node_t& node) { rcl_publisher_fini(&pub_, &node); }

 private:
  rcl_publisher_t pub_;
  sensor_msgs__msg__JointState msg_;
  EncodersStamped data_;

  void initMsg(rcl_allocator_t& allocator) {
    // Init nested structures
    memset(&msg_, 0, sizeof(msg_));
    sensor_msgs__msg__JointState__init(&msg_);
    msg_.header.frame_id =
        micro_ros_string_utilities_init(JOINT_STATE_FRAME_ID);

    const uint8_t n = g_motors.count();

    // Allocate name array
    msg_.name.capacity = n;
    msg_.name.size = n;
    msg_.name.data = (rosidl_runtime_c__String*)allocator.allocate(
        n * sizeof(rosidl_runtime_c__String), allocator.state);

    // Set joint names
    for (uint8_t i = 0; i < n; ++i) {
      msg_.name.data[i] =
          micro_ros_string_utilities_init(g_motors[i]->jointName());
    }

    // Allocate position array
    msg_.position.capacity = n;
    msg_.position.size = n;
    msg_.position.data =
        (double*)allocator.allocate(n * sizeof(double), allocator.state);

    // Allocate velocity array
    msg_.velocity.capacity = n;
    msg_.velocity.size = n;
    msg_.velocity.data =
        (double*)allocator.allocate(n * sizeof(double), allocator.state);

    // Allocate effort array
    // msg_.effort.capacity = n;
    // msg_.effort.size = n;
    // msg_.effort.data = (double*)allocator.allocate(n *
    // sizeof(double), allocator.state);

    // Zero initialize
    memset(msg_.position.data, 0, n * sizeof(double));
    memset(msg_.velocity.data, 0, n * sizeof(double));
    // memset(msg_.effort.data, 0, n * sizeof(double)); // not implemented
  }

  void fillMsg(const EncodersStamped& d) {
    msg_.header.stamp.sec = d.timestamp_ns / 1000000000LL;
    msg_.header.stamp.nanosec = d.timestamp_ns % 1000000000LL;

    // Fill position data
    msg_.position.data[0] = d.data.position[0];
    msg_.position.data[1] = d.data.position[1];
    msg_.position.data[2] = d.data.position[2];
    msg_.position.data[3] = d.data.position[3];

    // Fill velocity data
    msg_.velocity.data[0] = d.data.velocity[0];
    msg_.velocity.data[1] = d.data.velocity[1];
    msg_.velocity.data[2] = d.data.velocity[2];
    msg_.velocity.data[3] = d.data.velocity[3];

    // Fill effort data
    // msg_.effort.data[0] = d.data.effort[0];
    // msg_.effort.data[1] = d.data.effort[1];
    // msg_.effort.data[2] = d.data.effort[2];
    // msg_.effort.data[3] = d.data.effort[3];
  }
};
