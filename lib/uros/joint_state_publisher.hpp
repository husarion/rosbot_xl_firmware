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

#include "control/encoders_manager.hpp"
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
    msg_.header.frame_id = micro_ros_string_utilities_init("base_link");

    size_t num_of_joints = static_cast<size_t>(MotorID::COUNT);

    // Allocate name array
    msg_.name.capacity = num_of_joints;
    msg_.name.size = num_of_joints;
    msg_.name.data = (rosidl_runtime_c__String*)allocator.allocate(
        num_of_joints * sizeof(rosidl_runtime_c__String), allocator.state);

    // Set joint names
    msg_.name.data[0] =
        micro_ros_string_utilities_init(control::getJointName(MotorID::FL));
    msg_.name.data[1] =
        micro_ros_string_utilities_init(control::getJointName(MotorID::FR));
    msg_.name.data[2] =
        micro_ros_string_utilities_init(control::getJointName(MotorID::RL));
    msg_.name.data[3] =
        micro_ros_string_utilities_init(control::getJointName(MotorID::RR));

    // Allocate position array
    msg_.position.capacity = num_of_joints;
    msg_.position.size = num_of_joints;
    msg_.position.data = (double*)allocator.allocate(
        num_of_joints * sizeof(double), allocator.state);

    // Allocate velocity array
    msg_.velocity.capacity = num_of_joints;
    msg_.velocity.size = num_of_joints;
    msg_.velocity.data = (double*)allocator.allocate(
        num_of_joints * sizeof(double), allocator.state);

    // Allocate effort array
    // msg_.effort.capacity = num_of_joints;
    // msg_.effort.size = num_of_joints;
    // msg_.effort.data = (double*)allocator.allocate(num_of_joints *
    // sizeof(double), allocator.state);

    // Zero initialize
    memset(msg_.position.data, 0, num_of_joints * sizeof(double));
    memset(msg_.velocity.data, 0, num_of_joints * sizeof(double));
    // memset(msg_.effort.data, 0, num_of_joints * sizeof(double));
  }

  void fillMsg(const EncodersStamped& d) {
    msg_.header.stamp.sec = d.timestamp_ns / 1000000000LL;
    msg_.header.stamp.nanosec = d.timestamp_ns % 1000000000LL;

    // Fill position data
    msg_.position.data[0] = d.data.position[static_cast<uint8_t>(MotorID::FL)];
    msg_.position.data[1] = d.data.position[static_cast<uint8_t>(MotorID::FR)];
    msg_.position.data[2] = d.data.position[static_cast<uint8_t>(MotorID::RL)];
    msg_.position.data[3] = d.data.position[static_cast<uint8_t>(MotorID::RR)];

    // Fill velocity data
    msg_.velocity.data[0] = d.data.velocity[static_cast<uint8_t>(MotorID::FL)];
    msg_.velocity.data[1] = d.data.velocity[static_cast<uint8_t>(MotorID::FR)];
    msg_.velocity.data[2] = d.data.velocity[static_cast<uint8_t>(MotorID::RL)];
    msg_.velocity.data[3] = d.data.velocity[static_cast<uint8_t>(MotorID::RR)];

    // Fill effort data
    // msg_.effort.data[0] = d.data.effort[static_cast<uint8_t>(MotorID::FL)];
    // msg_.effort.data[1] = d.data.effort[static_cast<uint8_t>(MotorID::FR)];
    // msg_.effort.data[2] = d.data.effort[static_cast<uint8_t>(MotorID::RL)];
    // msg_.effort.data[3] = d.data.effort[static_cast<uint8_t>(MotorID::RR)];
  }
};
