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

#include "u_ros.hpp"

/*===== ROS MSGS TYPES =====*/
#include <builtin_interfaces/msg/time.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <std_srvs/srv/trigger.h>

/*===== MICRO ROS =====*/
#include <micro_ros_utilities/string_utilities.h>
#include <rcl/time.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include "control/encoders_manager.hpp"
#include "control/motors_manager.hpp"
#include "log.hpp"
#include "serial_manager.hpp"
#include "tasks.hpp"
#include "uros/battery_publisher.hpp"
#include "uros/buttons_publisher.hpp"
#include "uros/imu_publisher.hpp"
#include "uros/range_publisher.hpp"
namespace u_ros {

// PUBLISHERS
BatteryPublisher batteryPublisher;
ButtonsPublisher buttonsPublisher;
ImuPublisher imuPublisher;
RangePublisher rangePublisher;
rcl_publisher_t motor_state_pub;
// SUBSCRIPTIONS
rcl_subscription_t motors_cmd_sub;
rcl_subscription_t left_led_sub;
rcl_subscription_t right_led_sub;
// MESSAGES
builtin_interfaces__msg__Time now;
sensor_msgs__msg__JointState joint_state_msg;
std_msgs__msg__Bool led_msg;
std_msgs__msg__Float32MultiArray motors_cmd_msg;
// SERVICES
rcl_service_t get_cpu_id_service;
std_srvs__srv__Trigger_Request get_cpu_id_service_request;
std_srvs__srv__Trigger_Response get_cpu_id_service_response;
// ROS ENTITIES
u_ros_state_t state = WAITING;
rcl_init_options_t init_options;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

bool pingAgent(void) {
  return rmw_uros_ping_agent(uROS_PING_TIMEOUT_MS, uROS_PING_ATTEMPTS) ==
         RMW_RET_OK;
}

void motorsCmdCallback(const void* msg_in) {
  const std_msgs__msg__Float32MultiArray* msg =
      static_cast<const std_msgs__msg__Float32MultiArray*>(msg_in);
  if (msg == nullptr) return;

  if (msg->data.size >= 4) {
    float velocities[4];

    // Map ROS message order to motor order
    // ROS order: [FL, FR, RL, RR] -> Motor order: [FR, RR, RL, FL]
    velocities[static_cast<uint8_t>(MotorID::FR)] = msg->data.data[1];
    velocities[static_cast<uint8_t>(MotorID::RR)] = msg->data.data[3];
    velocities[static_cast<uint8_t>(MotorID::RL)] = msg->data.data[2];
    velocities[static_cast<uint8_t>(MotorID::FL)] = msg->data.data[0];

    motors.setVelocities(velocities);
  }
}

void uRosLeftLedCallback(const void* msg) {
  auto led_msg = (std_msgs__msg__Bool*)msg;
  digitalWrite(GRN_LED, led_msg->data ? HIGH : LOW);
}

void uRosRightLedCallback(const void* msg) {
  auto led_msg = (std_msgs__msg__Bool*)msg;
  digitalWrite(GRN_LED2, led_msg->data ? HIGH : LOW);
}

void uRosGetIdCallback(const void* req, void* res) {
  (void)req;  // Unused parameter

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

bool createEntities(void) {
  /*===== INIT ROS2 =====*/
  uint8_t ros_msgs_cnt = 0;
  allocator = rcl_get_default_allocator();
  // init_options = rcl_get_zero_initialized_init_options();
  // RCCHECK_RETURN(rcl_init_options_init(&init_options, allocator));
  // RCCHECK_RETURN(rcl_init_options_set_domain_id(
  //     &init_options, UXR_CLIENT_DOMAIN_ID_TO_OVERRIDE_WITH_ENV));
  // RCCHECK_RETURN(rclc_support_init_with_options(&support, 0, NULL,
  // &init_options,
  //                                        &allocator));
  RCCHECK_RETURN(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK_RETURN(rclc_node_init_default(
      &node, NODE_NAME, serialManager.getNamespace(), &support));

  /*===== MSGS =====*/
  initMotorsJointStateMsg(&joint_state_msg);
  initMotorsCmdMsg(&motors_cmd_msg);
  std_srvs__srv__Trigger_Request__init(&get_cpu_id_service_request);
  std_srvs__srv__Trigger_Response__init(&get_cpu_id_service_response);

  /*===== SUBSCRIBERS ===== */
  RCCHECK_RETURN(rclc_subscription_init_best_effort(
      &motors_cmd_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "_motors_cmd"));
  ros_msgs_cnt++;
  RCCHECK_RETURN(rclc_subscription_init_default(
      &left_led_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "led/left"));
  ros_msgs_cnt++;
  RCCHECK_RETURN(rclc_subscription_init_default(
      &right_led_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "led/right"));
  ros_msgs_cnt++;
  /*===== PUBLISHERS ===== */
  RCCHECK_RETURN(batteryPublisher.init(node, "battery"));
  RCCHECK_RETURN(buttonsPublisher.init(node, "buttons"));
  RCCHECK_RETURN(imuPublisher.init(node, "_imu/data_raw"));
  RCCHECK_RETURN(rclc_publisher_init_best_effort(
      &motor_state_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "_motors_response"));
  RCCHECK_RETURN(rangePublisher.init(node, "ranges"));
  /*===== SERVICES ===== */
  RCCHECK_RETURN(rclc_service_init_default(
      &get_cpu_id_service, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "/get_cpu_id"));
  ros_msgs_cnt++;

  /*===== EXECUTOR ===== */
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK_RETURN(rclc_executor_init(&executor, &support.context, ros_msgs_cnt,
                                    &allocator));
  RCCHECK_RETURN(rclc_executor_add_subscription(
      &executor, &motors_cmd_sub, &motors_cmd_msg, &motorsCmdCallback,
      ON_NEW_DATA));
  RCCHECK_RETURN(rclc_executor_add_subscription(
      &executor, &left_led_sub, &led_msg, &uRosLeftLedCallback, ON_NEW_DATA));
  RCCHECK_RETURN(rclc_executor_add_subscription(
      &executor, &right_led_sub, &led_msg, &uRosRightLedCallback, ON_NEW_DATA));
  RCCHECK_RETURN(rclc_executor_add_service(
      &executor, &get_cpu_id_service, &get_cpu_id_service_request,
      &get_cpu_id_service_response, uRosGetIdCallback));

  RCCHECK_RETURN(rmw_uros_sync_session(1000));
  delay(50);
  LOG_INFO("uROS communication started");

  return true;
}

void destroyEntities(void) {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  sensor_msgs__msg__JointState__fini(&joint_state_msg);

  batteryPublisher.fini(node);
  imuPublisher.fini(node);
  rcl_publisher_fini(&motor_state_pub, &node);
  rangePublisher.fini(node);
  rcl_subscription_fini(&motors_cmd_sub, &node);
  rcl_subscription_fini(&left_led_sub, &node);
  rcl_subscription_fini(&right_led_sub, &node);
  rcl_service_fini(&get_cpu_id_service, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  // rcl_init_options_fini(&init_options);
  LOG_INFO("uROS communication stopped");
}


void initMotorsJointStateMsg(sensor_msgs__msg__JointState* msg) {
  if (msg == nullptr) return;

  // Initialize message
  sensor_msgs__msg__JointState__init(msg);

  // Allocate frame_id
  msg->header.frame_id = micro_ros_string_utilities_init("base_link");

  // Allocate name array
  msg->name.capacity = 4;
  msg->name.size = 4;
  msg->name.data = (rosidl_runtime_c__String*)allocator.allocate(
      4 * sizeof(rosidl_runtime_c__String), allocator.state);

  // Set joint names
  msg->name.data[0] =
      micro_ros_string_utilities_init(control::getJointName(MotorID::FL));
  msg->name.data[1] =
      micro_ros_string_utilities_init(control::getJointName(MotorID::FR));
  msg->name.data[2] =
      micro_ros_string_utilities_init(control::getJointName(MotorID::RL));
  msg->name.data[3] =
      micro_ros_string_utilities_init(control::getJointName(MotorID::RR));

  // Allocate position array
  msg->position.capacity = 4;
  msg->position.size = 4;
  msg->position.data =
      (double*)allocator.allocate(4 * sizeof(double), allocator.state);

  // Allocate velocity array
  msg->velocity.capacity = 4;
  msg->velocity.size = 4;
  msg->velocity.data =
      (double*)allocator.allocate(4 * sizeof(double), allocator.state);

  // Allocate effort array
  // msg->effort.capacity = 4;
  // msg->effort.size = 4;
  // msg->effort.data = (double*)allocator.allocate(4 * sizeof(double),
  // allocator.state);

  // Zero initialize
  memset(msg->position.data, 0, 4 * sizeof(double));
  memset(msg->velocity.data, 0, 4 * sizeof(double));
  // memset(msg->effort.data, 0, 4 * sizeof(double));
}

void initMotorsCmdMsg(std_msgs__msg__Float32MultiArray* msg) {
  static float data[MOT_CMD_MSG_LEN] = {0, 0, 0, 0};
  msg->data.capacity = MOT_CMD_MSG_LEN;
  msg->data.size = MOT_CMD_MSG_LEN;
  msg->data.data = (float*)data;
}


void publishJointState() {
  Encoder& fl = encoders[MotorID::FL];
  Encoder& fr = encoders[MotorID::FR];
  Encoder& rl = encoders[MotorID::RL];
  Encoder& rr = encoders[MotorID::RR];

  // Set timestamp
  int64_t time_ns = rmw_uros_epoch_nanos();
  joint_state_msg.header.stamp.sec = time_ns / 1000000000;
  joint_state_msg.header.stamp.nanosec = time_ns % 1000000000;

  // Fill position data (order: FL, FR, RL, RR)
  joint_state_msg.position.data[0] = fl.getPosition();
  joint_state_msg.position.data[1] = fr.getPosition();
  joint_state_msg.position.data[2] = rl.getPosition();
  joint_state_msg.position.data[3] = rr.getPosition();

  // Fill velocity data
  joint_state_msg.velocity.data[0] = fl.getVelocity();
  joint_state_msg.velocity.data[1] = fr.getVelocity();
  joint_state_msg.velocity.data[2] = rl.getVelocity();
  joint_state_msg.velocity.data[3] = rr.getVelocity();

  // Fill effort data
  // joint_state_msg.effort.data[0] =
  // effort[static_cast<uint8_t>(MotorID::FL)];
  // joint_state_msg.effort.data[1] =
  // effort[static_cast<uint8_t>(MotorID::FR)];
  // joint_state_msg.effort.data[2] =
  // effort[static_cast<uint8_t>(MotorID::RL)];
  // joint_state_msg.effort.data[3] =
  // effort[static_cast<uint8_t>(MotorID::RR)];

  RCCHECK_WARN(rcl_publish(&motor_state_pub, &joint_state_msg, NULL));
}

void loop() {
  switch (state) {
    case WAITING:
      if (pingAgent()) {
        state = AGENT_AVAILABLE;
      }
      vTaskDelay(pdMS_TO_TICKS(500));
      break;

    case AGENT_AVAILABLE:
      if (createEntities()) {
        state = CONNECTED;
      } else {
        destroyEntities();
        state = WAITING;
      }
      break;

    case CONNECTED:
      if (!pingAgent()) {
        state = DISCONNECTED;
        break;
      }

      batteryPublisher.publish();
      buttonsPublisher.publish();
      imuPublisher.publish();
      rangePublisher.publish();
      publishJointState();

      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
      vTaskDelay(pdMS_TO_TICKS(1));
      break;

    case DISCONNECTED:
      destroyEntities();
      state = WAITING;
      break;
  }
}

}  // namespace u_ros
