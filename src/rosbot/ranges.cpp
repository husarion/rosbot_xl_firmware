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

#include "ranges.hpp"

void init_ranges() {
  MultiDistanceSensor& distance_sensors = MultiDistanceSensor::getInstance();
  if (distance_sensors.init(nullptr) > 0) {
    // On Arduino, just start measurement directly
    distance_sensors.start();
  }

  for (uint8_t i = 0; i < RANGES_COUNT; ++i) {
    fill_range_msg(&range_msgs[i], i);
  }
}

void fill_range_msg(sensor_msgs__msg__Range* msg, uint8_t id) {
  msg->header.frame_id.data = const_cast<char*>(range_frame_names[id]);

  // Arduino: use millis() as timestamp
  msg->header.stamp.sec = millis() / 1000;
  msg->header.stamp.nanosec = (millis() % 1000) * 1000000;
  msg->radiation_type = 0;  // e.g., 0 = INFRARED
  msg->field_of_view = 0.26f;
  msg->min_range = 0.01f;
  msg->max_range = 0.90f;
}

void fill_range_msg_with_measurements(sensor_msgs__msg__Range* msg,
                                      float range) {
  msg->range = range;

  if (msg->range > msg->max_range || msg->range < msg->min_range) {
    msg->range = NAN;
  }
}
