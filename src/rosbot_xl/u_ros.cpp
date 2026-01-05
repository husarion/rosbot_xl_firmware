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

#include "battery.hpp"
#include "hardware/imu.hpp"
#include "log.hpp"
#include "rtos/queues.hpp"

/*===== ROS MSGS TYPES =====*/
#include <builtin_interfaces/msg/time.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <std_srvs/srv/trigger.h>

/*===== MICRO ROS =====*/
#include <rcl/time.h>
#include <rclc/executor.h>

#include "bsp.hpp"

namespace u_ros {

// PUBLISHERS
rcl_publisher_t battery_pub;
// rcl_publisher_t buttons_pubs[BUTTONS_COUNT];
rcl_publisher_t imu_pub;
rcl_publisher_t motor_state_pub;
rcl_publisher_t range_pub;
// SUBSCRIPTIONS
rcl_subscription_t motors_cmd_sub;
rcl_subscription_t left_led_sub;
rcl_subscription_t right_led_sub;
// MESSAGES
builtin_interfaces__msg__Time now;
sensor_msgs__msg__BatteryState battery_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__Range range_msg;
sensor_msgs__msg__JointState motors_joint_state_msg;
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
rcl_timer_t timer;

void BlinkRedLed(int duration_ms) {
  SetRedLed(On);
  delay(duration_ms);
  SetRedLed(Off);
  delay(200);
}

void errorLoop(const char* func) {
  LOG_ERROR("In error loop from function %s", func);
  SetRedLed(Off);
  SetGreenLed(Off);
  delay(500);

  // 2 SOS signals: ... --- ...
  for (int i = 0; i < 2; ++i) {
    for (int i = 0; i < 3; ++i) {
      BlinkRedLed(200);
    }
    for (int i = 0; i < 3; ++i) {
      BlinkRedLed(600);
    }
    for (int i = 0; i < 3; ++i) {
      BlinkRedLed(200);
    }
    delay(1000);
  }

  LOG_ERROR("System resetting...");
  NVIC_SystemReset();
}

bool pingAgent(void) {
  return rmw_uros_ping_agent(uROS_PING_TIMEOUT_MS, uROS_PING_ATTEMPTS) ==
         RMW_RET_OK;
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

      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));

      SetRedLed(Off);
      vTaskDelay(pdMS_TO_TICKS(1));
      break;

    case DISCONNECTED:
      destroyEntities();
      SetRedLed(On);
      state = WAITING;
      break;
  }
}

void motorsCmdCallback(const void* arg_input_message) {
  static double setpoint[] = {0, 0, 0, 0};
  static std_msgs__msg__Float32MultiArray* setpoint_msg;
  setpoint_msg = (std_msgs__msg__Float32MultiArray*)arg_input_message;
  if (setpoint_msg->data.size == 4) {
    for (uint8_t i = 0; i < setpoint_msg->data.size; i++) {
      setpoint[i] = (double)setpoint_msg->data.data[i];
    }
  }
  xQueueOverwrite(rtos::queues::SetpointQueue, (void*)setpoint);
}

void timerCallback(rcl_timer_t* arg_timer, int64_t arg_last_call_time) {
  RCLC_UNUSED(arg_last_call_time);
  static imu_data_t queue_imu;
  static motor_joint_state_t motor_state_queue;
  static battery_data_t battery_state_queue;
  if (arg_timer != NULL) {
    // QOS default
    if (xQueueReceive(rtos::queues::BatteryQueue, &battery_state_queue,
                      (TickType_t)0) == pdPASS) {
      if (rmw_uros_epoch_synchronized()) {
        battery_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
        battery_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
      }
      battery_msg.voltage = battery_state_queue.voltage;
      battery_msg.temperature = battery_state_queue.temperature;
      battery_msg.current = battery_state_queue.current;
      battery_msg.charge = battery_state_queue.charge_current;
      battery_msg.capacity = battery_state_queue.capacity;
      battery_msg.design_capacity = battery_state_queue.design_capacity;
      battery_msg.percentage = battery_state_queue.percentage;
      battery_msg.power_supply_status = battery_state_queue.status;
      battery_msg.power_supply_health = battery_state_queue.health;
      battery_msg.power_supply_technology = battery_state_queue.technology;
      battery_msg.present = battery_state_queue.present;
      battery_msg.cell_temperature.capacity =
          BATTERY_STATE_MSG_CELL_TEMPERATURE_ARRAY_SIZE;
      battery_msg.cell_temperature.size =
          BATTERY_STATE_MSG_CELL_TEMPERATURE_ARRAY_SIZE;
      battery_msg.cell_temperature.data = battery_state_queue.cell_temperature;
      battery_msg.cell_voltage.capacity =
          BATTERY_STATE_MSG_CELL_VOLTAGE_ARRAY_SIZE;
      battery_msg.cell_voltage.size = BATTERY_STATE_MSG_CELL_VOLTAGE_ARRAY_SIZE;
      battery_msg.cell_voltage.data = battery_state_queue.cell_voltage;
      RCCHECK_WARN(rcl_publish(&battery_pub, &battery_msg, NULL));
    }
    // QOS best effort
    if (xQueueReceive(rtos::queues::MotorStateQueue, &motor_state_queue,
                      (TickType_t)0) == pdPASS) {
      if (rmw_uros_epoch_synchronized()) {
        motors_joint_state_msg.header.stamp.sec =
            rmw_uros_epoch_millis() / 1000;
        motors_joint_state_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
      }
      motors_joint_state_msg.velocity.data = motor_state_queue.velocity;
      motors_joint_state_msg.position.data = motor_state_queue.position;
      RCCHECK_WARN(
          rcl_publish(&motor_state_pub, &motors_joint_state_msg, NULL));
    }
    // QOS best effort
    if (xQueueReceive(rtos::queues::ImuQueue, &queue_imu, (TickType_t)0) ==
        pdPASS) {
      if (rmw_uros_epoch_synchronized()) {
        imu_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
        imu_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
      }
      imu_msg.header.frame_id.data = const_cast<char*>("imu_link");
      imu_msg.orientation.x = queue_imu.orientation[0];
      imu_msg.orientation.y = queue_imu.orientation[1];
      imu_msg.orientation.z = queue_imu.orientation[2];
      imu_msg.orientation.w = queue_imu.orientation[3];
      imu_msg.angular_velocity.x = queue_imu.angular_velocity[0];
      imu_msg.angular_velocity.y = queue_imu.angular_velocity[1];
      imu_msg.angular_velocity.z = queue_imu.angular_velocity[2];
      imu_msg.linear_acceleration.x = queue_imu.acceleration[0];
      imu_msg.linear_acceleration.y = queue_imu.acceleration[1];
      imu_msg.linear_acceleration.z = queue_imu.acceleration[2];
      RCCHECK_WARN(rcl_publish(&imu_pub, &imu_msg, NULL));
    }
  }
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
  uint8_t ros_msgs_cnt = 0;
  /*===== ALLOCATE MEMORY FOR MSGS =====*/
  initMotorsJointStateMsg(&motors_joint_state_msg);
  initMotorsCmdMsg(&motors_cmd_msg);
  allocator = rcl_get_default_allocator();
  // create init_options
  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(
      &init_options, UXR_CLIENT_DOMAIN_ID_TO_OVERRIDE_WITH_ENV));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options,
                                         &allocator));
  LOG_DEBUG("Created support with option domain_id=%d\r\n",
            UXR_CLIENT_DOMAIN_ID_TO_OVERRIDE_WITH_ENV);

  // create node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));
  LOG_DEBUG("Created node `%s`\r\n", NODE_NAME);
  /*===== INIT TIMERS =====*/
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(10),
                                  timerCallback));
  ros_msgs_cnt++;
  LOG_DEBUG("Created timer\r\n");
  /*===== INIT SUBSCRIBERS ===== */
  RCCHECK(rclc_subscription_init_best_effort(
      &motors_cmd_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "_motors_cmd"));
  ros_msgs_cnt++;
  LOG_DEBUG("Created '_motors_cmd' subscriber\r\n");
  /*===== INIT PUBLISHERS ===== */
  // IMU
  RCCHECK(rclc_publisher_init_best_effort(
      &imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "_imu/data_raw"));
  // ros_msgs_cnt++;
  LOG_DEBUG("Created '_imu/data_raw' publisher.\r\n");
  // MOTORS RESPONSE
  RCCHECK(rclc_publisher_init_best_effort(
      &motor_state_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "_motors_response"));
  // ros_msgs_cnt++;
  LOG_DEBUG("Created '_motors_response' publisher.\r\n");
  // BATTERY STATE
  RCCHECK(rclc_publisher_init_best_effort(
      &battery_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
      "battery_state"));
  // ros_msgs_cnt++;
  LOG_DEBUG("Created 'battery_state' publisher.\r\n");
  /*===== INIT SERVICES ===== */
  std_srvs__srv__Trigger_Request__init(&get_cpu_id_service_request);
  std_srvs__srv__Trigger_Response__init(&get_cpu_id_service_response);
  RCCHECK(rclc_service_init_default(
      &get_cpu_id_service, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "get_cpu_id"));
  ros_msgs_cnt++;
  LOG_DEBUG("Created 'get_cpu_id_service' service.\r\n");
  /*===== CREATE ENTITIES ===== */
  RCCHECK(rclc_executor_init(&executor, &support.context, ros_msgs_cnt,
                             &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &motors_cmd_sub,
                                         &motors_cmd_msg, &motorsCmdCallback,
                                         ON_NEW_DATA));
  RCCHECK(rclc_executor_add_service(
      &executor, &get_cpu_id_service, &get_cpu_id_service_request,
      &get_cpu_id_service_response, uRosGetIdCallback));
  LOG_DEBUG("Executor started\r\n");

  RCCHECK(rmw_uros_sync_session(1000));
  LOG_DEBUG("Clocks synchronised\r\n");
  return true;
}

void destroyEntities(void) {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCCHECK(rcl_publisher_fini(&imu_pub, &node));
  RCCHECK(rcl_publisher_fini(&motor_state_pub, &node));
  RCCHECK(rcl_publisher_fini(&battery_pub, &node));
  RCCHECK(rcl_subscription_fini(&motors_cmd_sub, &node));
  RCCHECK(rcl_service_fini(&get_cpu_id_service, &node));
  RCCHECK(rcl_timer_fini(&timer));
  RCCHECK(rclc_executor_fini(&executor));
  RCCHECK(rcl_node_fini(&node));
  RCCHECK(rclc_support_fini(&support));
  RCCHECK(rcl_init_options_fini(&init_options));
  LOG_DEBUG("Destroyed all microros entities.\r\n");
}

void initMotorsJointStateMsg(sensor_msgs__msg__JointState* arg_message) {
  static rosidl_runtime_c__String msg_name_tab[MOT_RESP_MSG_LEN];
  static double msg_data_tab[3][MOT_RESP_MSG_LEN];
  char* frame_id = (char*)"motors_response";
  arg_message->position.data = msg_data_tab[0];
  arg_message->position.capacity = arg_message->position.size =
      MOT_RESP_MSG_LEN;
  arg_message->velocity.data = msg_data_tab[1];
  arg_message->velocity.capacity = arg_message->velocity.size =
      MOT_RESP_MSG_LEN;
  arg_message->effort.data = msg_data_tab[2];
  arg_message->effort.capacity = arg_message->effort.size = MOT_RESP_MSG_LEN;
  arg_message->header.frame_id.data = frame_id;
  arg_message->header.frame_id.capacity = arg_message->header.frame_id.size =
      strlen((const char*)frame_id);
  msg_name_tab->capacity = msg_name_tab->size = MOT_RESP_MSG_LEN;
  msg_name_tab[0].data = const_cast<char*>(REAR_RIGHT_MOTOR_NAME);
  msg_name_tab[1].data = const_cast<char*>(REAR_LEFT_MOTOR_NAME);
  msg_name_tab[2].data = const_cast<char*>(FRONT_RIGHT_MOTOR_NAME);
  msg_name_tab[3].data = const_cast<char*>(FRONT_LEFT_MOTOR_NAME);
  for (uint8_t i = 0; i < MOT_RESP_MSG_LEN; i++) {
    msg_name_tab[i].capacity = msg_name_tab[i].size =
        strlen(msg_name_tab[i].data);
  }
  arg_message->name.capacity = arg_message->name.size = MOT_RESP_MSG_LEN;
  arg_message->name.data = msg_name_tab;
}

void initMotorsCmdMsg(std_msgs__msg__Float32MultiArray* arg_message) {
  static float data[MOT_CMD_MSG_LEN] = {0, 0, 0, 0};
  arg_message->data.capacity = MOT_CMD_MSG_LEN;
  arg_message->data.size = MOT_CMD_MSG_LEN;
  arg_message->data.data = data;
}
}  // namespace u_ros
