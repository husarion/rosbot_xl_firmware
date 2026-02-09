#pragma once

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/battery_state.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rmw_microros/rmw_microros.h>
#include "battery.hpp"
#include "tasks.hpp"

class BatteryPublisher {
public:
    rcl_ret_t init(rcl_node_t& node, const char* topic_name) {
        initMsg();
        return rclc_publisher_init_best_effort(
            &pub_, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
            topic_name);
    }

    void publish() {
        BatteryData data;
        if (xQueueReceive(rtos::BatteryQueue, &data, 0) != pdPASS) {
            return;
        }

        fillMsg(data);
        rcl_publish(&pub_, &msg_, NULL);
    }

    void fini(rcl_node_t& node) {
        rcl_publisher_fini(&pub_, &node);
    }

private:
    rcl_publisher_t pub_;
    sensor_msgs__msg__BatteryState msg_;

    void initMsg() {
        memset(&msg_, 0, sizeof(msg_));
        msg_.header.frame_id =
            micro_ros_string_utilities_set(msg_.header.frame_id, "base_link");

        if (rmw_uros_epoch_synchronized()) {
            msg_.header.stamp.sec = (int32_t)(rmw_uros_epoch_nanos() / 1000000000);
            msg_.header.stamp.nanosec = (uint32_t)(rmw_uros_epoch_nanos() % 1000000000);
        }

        msg_.voltage = NAN;
        msg_.temperature = NAN;
        msg_.current = NAN;
        msg_.charge = NAN;
        msg_.capacity = NAN;
        msg_.design_capacity = 3 * 2.6f;
        msg_.percentage = NAN;
        msg_.power_supply_status =
            sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_UNKNOWN;
        msg_.power_supply_health =
            sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN;
        msg_.power_supply_technology =
            sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LION;
        msg_.present = true;
        rosidl_runtime_c__float__Sequence__init(&msg_.cell_voltage, 3);
        for (size_t i = 0; i < msg_.cell_voltage.size; i++) {
            msg_.cell_voltage.data[i] = NAN;
        }
        rosidl_runtime_c__float__Sequence__init(&msg_.cell_temperature, 3);
        for (size_t i = 0; i < msg_.cell_temperature.size; i++) {
            msg_.cell_temperature.data[i] = NAN;
        }
        msg_.location = micro_ros_string_utilities_set(msg_.location, "internal");
        msg_.serial_number = micro_ros_string_utilities_set(msg_.serial_number, "");
    }

    void fillMsg(const BatteryData& data) {
        msg_.header.stamp.sec = data.timestamp_ns / 1000000000LL;
        msg_.header.stamp.nanosec = data.timestamp_ns % 1000000000LL;

        msg_.voltage = data.voltage;
        msg_.temperature = data.temperature;
        msg_.current = data.current;
        msg_.percentage = data.percentage;
    }
};
