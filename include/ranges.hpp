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
#include <sensor_msgs/msg/range.h>

#include <MultiDistanceSensor.hpp>

// Optional: if using Arduino ROS 2 client, include sensor_msgs/Range header
// #include <sensor_msgs/msg/range.h>

enum Ranges {
  range_right_front,
  range_left_front,
  range_right_rear,
  range_left_rear,
  RANGES_COUNT
};

static const char* range_frame_names[RANGES_COUNT] = {"fr_range", "fl_range",
                                                      "rr_range", "rl_range"};

typedef struct {
  float range[4];
} ranges_queue_t;

static sensor_msgs__msg__Range range_msgs[RANGES_COUNT];

void init_ranges();
void fill_range_msg(sensor_msgs__msg__Range* msg, uint8_t id);
void fill_range_msg_with_measurements(sensor_msgs__msg__Range* msg,
                                      float range);
