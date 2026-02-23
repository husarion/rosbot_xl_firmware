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
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <sensor_msgs/msg/battery_state.h>

#include "../utils.hpp"
#include "battery_interface.hpp"
#include "publisher_interface.hpp"

struct BatteryStamped {
  BatteryData data;
  int64_t timestamp_ns;
};

struct BatteryPublisherConfig {
  const char* topic;
  QueueHandle_t& queue;
  const char* frame_id;
  float design_capacity;
  uint16_t num_cells;
};

class BatteryPublisher : public PublisherInterface {
 public:
  explicit BatteryPublisher(BatteryPublisherConfig cfg)
      : PublisherInterface(cfg.topic), cfg_(cfg) {}

  rcl_ret_t init(rcl_node_t& node, rcl_allocator_t& allocator) override {
    initMsg();
    return rclc_publisher_init_best_effort(
        &pub_, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), topic_);
  }

  void publish() override {
    if (xQueueReceive(cfg_.queue, &data_, 0) != pdPASS) return;
    fillMsg(data_);
    RC_SKIP(rcl_publish(&pub_, &msg_, NULL));
  }

  void fini(rcl_node_t& node) override {
    RC_SKIP(rcl_publisher_fini(&pub_, &node));
  }

  const char* topicName() const override { return topic_; }

 private:
  rcl_publisher_t pub_;
  sensor_msgs__msg__BatteryState msg_;
  BatteryStamped data_;
  BatteryPublisherConfig cfg_;

  void initMsg() {
    memset(&msg_, 0, sizeof(msg_));

    msg_.header.frame_id = micro_ros_string_utilities_init(cfg_.frame_id);

    msg_.voltage = NAN;
    msg_.temperature = NAN;
    msg_.current = NAN;
    msg_.charge = NAN;
    msg_.capacity = NAN;

    msg_.design_capacity = cfg_.design_capacity;

    msg_.percentage = NAN;
    msg_.power_supply_status =
        sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_UNKNOWN;
    msg_.power_supply_health =
        sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN;
    msg_.power_supply_technology =
        sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LION;
    msg_.present = true;

    rosidl_runtime_c__float__Sequence__init(&msg_.cell_voltage, cfg_.num_cells);
    for (size_t i = 0; i < msg_.cell_voltage.size; i++)
      msg_.cell_voltage.data[i] = NAN;

    rosidl_runtime_c__float__Sequence__init(&msg_.cell_temperature,
                                            cfg_.num_cells);
    for (size_t i = 0; i < msg_.cell_temperature.size; i++)
      msg_.cell_temperature.data[i] = NAN;

    msg_.location = micro_ros_string_utilities_set(msg_.location, "internal");
    msg_.serial_number = micro_ros_string_utilities_set(msg_.serial_number, "");
  }

  void fillMsg(const BatteryStamped& d) {
    msg_.header.stamp.sec = d.timestamp_ns / 1000000000LL;
    msg_.header.stamp.nanosec = d.timestamp_ns % 1000000000LL;
    msg_.voltage = d.data.voltage;
    msg_.temperature = d.data.temperature;
    msg_.current = d.data.current;
    msg_.percentage = d.data.percentage;
  }
};
