#pragma once

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rmw_microros/rmw_microros.h>
#include "sensors/ranges.hpp"
#include "tasks.hpp"

class RangePublisher {
public:
    rcl_ret_t init(rcl_node_t& node, const char* topic_name) {
        initMsg();
        return rclc_publisher_init_best_effort(
            &pub_, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
            topic_name);
    }

    void publish() {
        RangesData data;
        if (xQueueReceive(rtos::RangeQueue, &data, 0) != pdPASS) {
            return;
        }

        msg_.header.stamp.sec = d.timestamp_ns / 1000000000LL;
        msg_.header.stamp.nanosec = d.timestamp_ns % 1000000000LL;

        for (uint8_t i = 0; i < RANGES_COUNT; i++) {
            msg_.header.frame_id.data = (char*)range_frame_names[i];
            msg_.range = d.range[i];
            rcl_publish(&pub_, &msg_, NULL);
        }
    }

    void fini(rcl_node_t& node) {
        rcl_publisher_fini(&pub_, &node);
    }

private:
    rcl_publisher_t pub_;
    sensor_msgs__msg__Range msg_;

    void initMsg() {
        memset(&msg_, 0, sizeof(msg_));
        msg_.radiation_type = sensor_msgs__msg__Range__INFRARED;
        msg_.field_of_view = 0.26;
        msg_.min_range = 0.01;
        msg_.max_range = 0.9;
        msg_.range = NAN;
    }

};
