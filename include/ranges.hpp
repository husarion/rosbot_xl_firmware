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
