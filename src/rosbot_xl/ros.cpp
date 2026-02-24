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

#include <vector>

#include "ros/ros_node.hpp"

/*===== ROS MSGS TYPES =====*/
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>
#include <std_srvs/srv/trigger.h>

#include "config.hpp"
#include "motor_array.hpp"
#include "ros/publishers/battery_publisher.hpp"
#include "ros/publishers/buttons_publisher.hpp"
#include "ros/publishers/imu_publisher.hpp"
#include "ros/publishers/joint_state_publisher.hpp"
#include "ros/subscribers/led_subscriber.hpp"

void rosSetup() {}
// PUBLISHERS
static BatteryPublisher s_battery_pub(battery_pub_config);
static ButtonsPublisher s_buttons_pub(buttons_pub_config);
static ImuPublisher s_imu_pub(imu_pub_config);
static JointStatePublisher s_joint_pub(joint_state_pub_config);

static std::vector<PublisherInterface*> s_publishers = {
    &s_battery_pub, &s_buttons_pub, &s_imu_pub, &s_joint_pub};
uint8_t pub_count = static_cast<uint8_t>(s_publishers.size());

// SUBSCRIBERS
static std_msgs__msg__Float32MultiArray s_mot_msg = {
    .layout = {},
    .data =
        {
            .data = new float[MAX_NUM_MOTORS](),
            .size = MAX_NUM_MOTORS,
            .capacity = MAX_NUM_MOTORS,
        },
};

void motorsCmdCallback(const void* msg_in) {
  if (msg_in == nullptr) return;
  auto msg = static_cast<const std_msgs__msg__Float32MultiArray*>(msg_in);

  const uint8_t n = g_motors.count();
  if (msg->data.size >= n) {
    g_motors.setVelocities(msg->data.data);
  }
}

static LedSubState s_led_left_state;
static LedSubState s_led_right_state;

static std::vector<SubscriptionEntry> s_subscriptions = {
    {
        .msg = &s_mot_msg,
        .type_support =
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        .topic_name = "_motors_cmd",
        .callback = motorsCmdCallback,
        .best_effort = true,
    },
    makeLedSubscription({.pin = GRN_LED, .topic_name = "led/left"},
                        &s_led_left_state),
    makeLedSubscription({.pin = GRN_LED2, .topic_name = "led/right"},
                        &s_led_right_state),
};

// SERVICES
rcl_service_t get_cpu_id_service;
std_srvs__srv__Trigger_Request s_cpuid_req;
std_srvs__srv__Trigger_Response s_cpuid_res;

void getCpuIdCallback(const void* req, void* res) {
  const uint32_t ADDRESS = 0x1FFF7A10;
  const uint8_t NUM_BYTES = 12;
  uint8_t buffer[NUM_BYTES];
  memcpy(buffer, (void*)ADDRESS, NUM_BYTES);

  // Prepare the CPU ID in hexadecimal format
  char cpu_id_buffer[NUM_BYTES * 2 + 1] = {0};
  char* hex_ptr = cpu_id_buffer;
  for (uint8_t i = 0; i < NUM_BYTES; ++i) {
    snprintf(hex_ptr, 3, "%02X", buffer[i]);
    hex_ptr += 2;
  }

  // Prepare the final output buffer with "CPU ID: " prefix
  static char out_buffer[100];  // Ensure this is large enough
  snprintf(out_buffer, sizeof(out_buffer), "{\"cpu_id\": \"%s\"}",
           cpu_id_buffer);

  // Set the response
  std_srvs__srv__Trigger_Response* response =
      (std_srvs__srv__Trigger_Response*)res;
  response->success = true;
  response->message.data = out_buffer;
  response->message.size = strlen(out_buffer);
}

static std::vector<ServiceEntry> s_services = {
    {
        .srv = {},
        .request = &s_cpuid_req,
        .response = &s_cpuid_res,
        .type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
        .topic_name = "/get_cpu_id",
        .callback = getCpuIdCallback,
    },
};

RosNodeConfig ros_node_config = {.node_name = NODE_NAME,
                                 .domain_id = DOMAIN_ID,
                                 .publishers = s_publishers.data(),
                                 .pub_count = s_publishers.size(),
                                 .subscriptions = s_subscriptions.data(),
                                 .sub_count = s_subscriptions.size(),
                                 .services = s_services.data(),
                                 .srv_count = s_services.size(),
                                 .ping_attempts = PING_ATTEMPTS,
                                 .ping_timeout_ms = PING_TIMEOUT_MS};

RosNode g_ros_node(ros_node_config);
