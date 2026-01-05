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

#include "battery_types.hpp"
#include "bsp.hpp"
#include "buttons.hpp"
#include "hardware/imu.hpp"
#include "log.hpp"
#include "ranges.hpp"
#include "rtos/queues.hpp"

namespace u_ros {

// PUBLISHERS
rcl_publisher_t battery_pub;
rcl_publisher_t buttons_pubs[BUTTONS_COUNT];
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

void ErrorLoop(const char* func) {
  LOG_ERROR("In error loop from function %s", func);
  SetRedLed(Off);
  SetGreenLed(Off);
  SetGreenLed2(Off);
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

void transportInit(void) {
  rmw_uros_set_custom_transport(
      /* Enable XRCE framing */
      true,
      /* Arguments for open function */
      NULL,
      /* Open transport callback */
      [](struct uxrCustomTransport* transport) -> bool {
        SBC_SERIAL.setRx(SBC_SERIAL_RX);
        SBC_SERIAL.setTx(SBC_SERIAL_TX);
        SBC_SERIAL.setTimeout(SBC_SERIAL_TIMEOUT);
        SBC_SERIAL.begin(SBC_SERIAL_BAUDRATE);
        return SBC_SERIAL ? true : false;
      },
      /* Close transport callback */
      [](struct uxrCustomTransport* transport) -> bool {
        SBC_SERIAL.end();
        return true;
      },
      /* Write transport callback */
      [](struct uxrCustomTransport* transport, const uint8_t* buf, size_t len,
         uint8_t* errcode) -> unsigned int {
        return SBC_SERIAL.write(buf, len);
      },
      /* Read transport callback */
      [](struct uxrCustomTransport* transport, uint8_t* buf, size_t len,
         int timeout, uint8_t* errcode) -> unsigned int {
        SBC_SERIAL.setTimeout(timeout);
        return SBC_SERIAL.readBytes((char*)buf, len);
      });
}

bool pingAgent(void) {
  return rmw_uros_ping_agent(uROS_PING_TIMEOUT_MS, uROS_PING_ATTEMPTS) ==
         RMW_RET_OK;
}

void motorsCmdCallback(const void* input_message) {
  static double setpoint[] = {0, 0, 0, 0};
  static std_msgs__msg__Float32MultiArray* setpoint_msg;
  setpoint_msg = (std_msgs__msg__Float32MultiArray*)input_message;
  if (setpoint_msg->data.size == 4) {
    for (uint8_t i = 0; i < setpoint_msg->data.size; i++) {
      setpoint[i] = (double)setpoint_msg->data.data[i];
    }
  }
  xQueueOverwrite(rtos::queues::SetpointQueue, (void*)setpoint);
}

void uRosLeftLedCallback(const void* msg) {
  auto led_msg = (std_msgs__msg__Bool*)msg;
  SetGreenLed(led_msg->data ? On : Off);
}

void uRosRightLedCallback(const void* msg) {
  auto led_msg = (std_msgs__msg__Bool*)msg;
  SetGreenLed2(led_msg->data ? On : Off);
}

void timerCallback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    publishBattery();
    publishImu();
    publishRanges();
    publishWheelsJointState();
    UBaseType_t stack_free = uxTaskGetStackHighWaterMark(nullptr);
    LOG_INFO("Free stack size: %lu", stack_free);
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
  /*===== MSGS =====*/
  motorsJointStateInit(&motors_joint_state_msg);
  motorsCmdMsgInit(&motors_cmd_msg);
  std_srvs__srv__Trigger_Request__init(&get_cpu_id_service_request);
  std_srvs__srv__Trigger_Response__init(&get_cpu_id_service_response);

  /*===== INIT ROS2 =====*/
  allocator = rcl_get_default_allocator();
  // init_options = rcl_get_zero_initialized_init_options();
  // RCCHECK(rcl_init_options_init(&init_options, allocator));
  // RCCHECK(rcl_init_options_set_domain_id(
  //     &init_options, UXR_CLIENT_DOMAIN_ID_TO_OVERRIDE_WITH_ENV));
  // RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options,
  //                                        &allocator));

  RCCHECK_RETURN(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK_RETURN(rclc_node_init_default(&node, NODE_NAME, "", &support));

  /*===== TIMERS =====*/
  RCCHECK_RETURN(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(10),
                                         timerCallback));
  ros_msgs_cnt++;
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
  RCCHECK_RETURN(rclc_publisher_init_best_effort(
      &battery_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), "battery"));
  RCCHECK_RETURN(rclc_publisher_init_best_effort(
      &imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "_imu/data_raw"));
  RCCHECK_RETURN(rclc_publisher_init_best_effort(
      &motor_state_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "_motors_response"));
  RCCHECK_RETURN(rclc_publisher_init_best_effort(
      &range_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
      "ranges"));
  /*===== SERVICES ===== */
  RCCHECK_RETURN(rclc_service_init_default(
      &get_cpu_id_service, &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), "/get_cpu_id"));
  ros_msgs_cnt++;

  /*===== EXECUTOR ===== */
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK_RETURN(rclc_executor_init(&executor, &support.context, ros_msgs_cnt,
                                    &allocator));
  RCCHECK_RETURN(rclc_executor_add_timer(&executor, &timer));
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
  LOG_INFO("uROS communication started");

  return true;
}

void destroyEntities(void) {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCCHECK_WARN(rcl_timer_fini(&timer));
  RCCHECK_WARN(rcl_publisher_fini(&battery_pub, &node));
  RCCHECK_WARN(rcl_publisher_fini(&imu_pub, &node));
  RCCHECK_WARN(rcl_publisher_fini(&motor_state_pub, &node));
  RCCHECK_WARN(rcl_publisher_fini(&range_pub, &node));
  RCCHECK_WARN(rcl_subscription_fini(&motors_cmd_sub, &node));
  RCCHECK_WARN(rcl_subscription_fini(&left_led_sub, &node));
  RCCHECK_WARN(rcl_subscription_fini(&right_led_sub, &node));
  RCCHECK_WARN(rcl_service_fini(&get_cpu_id_service, &node));
  RCCHECK_WARN(rclc_executor_fini(&executor));
  RCCHECK_WARN(rcl_node_fini(&node));
  RCCHECK_WARN(rclc_support_fini(&support));
  RCCHECK_WARN(rcl_init_options_fini(&init_options));
  LOG_INFO("uROS communication stopped");
}

void motorsJointStateInit(sensor_msgs__msg__JointState* msg) {
  static rosidl_runtime_c__String msg_name_tab[MOT_RESP_MSG_LEN];
  static double msg_data_tab[3][MOT_RESP_MSG_LEN];
  char* frame_id = (char*)"motors_response";
  msg->position.data = msg_data_tab[0];
  msg->position.capacity = msg->position.size = MOT_RESP_MSG_LEN;
  msg->velocity.data = msg_data_tab[1];
  msg->velocity.capacity = msg->velocity.size = MOT_RESP_MSG_LEN;
  msg->effort.data = msg_data_tab[2];
  msg->effort.capacity = msg->effort.size = MOT_RESP_MSG_LEN;
  msg->header.frame_id.data = frame_id;
  msg->header.frame_id.capacity = msg->header.frame_id.size =
      strlen((const char*)frame_id);
  msg_name_tab->capacity = msg_name_tab->size = MOT_RESP_MSG_LEN;
  msg_name_tab[0].data = (char*)REAR_RIGHT_MOTOR_NAME;
  msg_name_tab[1].data = (char*)REAR_LEFT_MOTOR_NAME;
  msg_name_tab[2].data = (char*)FRONT_RIGHT_MOTOR_NAME;
  msg_name_tab[3].data = (char*)FRONT_LEFT_MOTOR_NAME;
  for (uint8_t i = 0; i < MOT_RESP_MSG_LEN; i++) {
    msg_name_tab[i].capacity = msg_name_tab[i].size =
        strlen(msg_name_tab[i].data);
  }
  msg->name.capacity = MOT_RESP_MSG_LEN;
  msg->name.data = msg_name_tab;
  msg->name.size = MOT_RESP_MSG_LEN;
}

void motorsCmdMsgInit(std_msgs__msg__Float32MultiArray* msg) {
  static float data[MOT_CMD_MSG_LEN] = {0, 0, 0, 0};
  msg->data.capacity = MOT_CMD_MSG_LEN;
  msg->data.size = MOT_CMD_MSG_LEN;
  msg->data.data = (float*)data;
}

void publishBattery() {
  RCCHECK_WARN(rcl_publish(&battery_pub, &battery_msg, NULL));
  return;

  static battery_state_t battery_state_data;
  if (xQueueReceive(rtos::queues::BatteryStateQueue, &battery_state_data,
                    (TickType_t)0) == pdPASS) {
    if (rmw_uros_epoch_synchronized()) {
      battery_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
      battery_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
    }
    battery_msg.voltage = battery_state_data.voltage;
    battery_msg.temperature = battery_state_data.temperature;
    battery_msg.current = battery_state_data.current;
    battery_msg.charge = battery_state_data.charge_current;
    battery_msg.capacity = battery_state_data.capacity;
    battery_msg.design_capacity = battery_state_data.design_capacity;
    battery_msg.percentage = battery_state_data.percentage;
    battery_msg.power_supply_status = battery_state_data.status;
    battery_msg.power_supply_health = battery_state_data.health;
    battery_msg.power_supply_technology = battery_state_data.technology;
    battery_msg.present = battery_state_data.present;
    battery_msg.cell_temperature.capacity =
        BATTERY_STATE_MSG_CELL_TEMPERATURE_ARRAY_SIZE;
    battery_msg.cell_temperature.size =
        BATTERY_STATE_MSG_CELL_TEMPERATURE_ARRAY_SIZE;
    battery_msg.cell_temperature.data = battery_state_data.cell_temperature;
    battery_msg.cell_voltage.capacity =
        BATTERY_STATE_MSG_CELL_VOLTAGE_ARRAY_SIZE;
    battery_msg.cell_voltage.size = BATTERY_STATE_MSG_CELL_VOLTAGE_ARRAY_SIZE;
    battery_msg.cell_voltage.data = battery_state_data.cell_voltage;
    RCCHECK_WARN(rcl_publish(&battery_pub, &battery_msg, NULL));
  }
}

void publishImu() {
  static imu_data_t imu_data;
  if (xQueueReceive(rtos::queues::ImuQueue, &imu_data, (TickType_t)0) ==
      pdPASS) {
    if (rmw_uros_epoch_synchronized()) {
      imu_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
      imu_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
    } else {
      LOG_DEBUG("!rmw_uros_epoch_synchronized");
    }
    imu_msg.header.frame_id.data = const_cast<char*>("imu_link");
    imu_msg.orientation.x = imu_data.orientation[0];
    imu_msg.orientation.y = imu_data.orientation[1];
    imu_msg.orientation.z = imu_data.orientation[2];
    imu_msg.orientation.w = imu_data.orientation[3];
    imu_msg.angular_velocity.x = imu_data.angular_velocity[0];
    imu_msg.angular_velocity.y = imu_data.angular_velocity[1];
    imu_msg.angular_velocity.z = imu_data.angular_velocity[2];
    imu_msg.linear_acceleration.x = imu_data.acceleration[0];
    imu_msg.linear_acceleration.y = imu_data.acceleration[1];
    imu_msg.linear_acceleration.z = imu_data.acceleration[2];
    RCCHECK_WARN(rcl_publish(&imu_pub, &imu_msg, NULL));
  }
}

void publishRanges() {
  RCCHECK_WARN(rcl_publish(&range_pub, &range_msg, NULL));
  return;

  static motor_joint_state_t ranges_data;
  if (xQueueReceive(rtos::queues::RangeQueue, &ranges_data, (TickType_t)0) ==
      pdPASS) {
    if (rmw_uros_epoch_synchronized()) {
      range_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
      range_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
    }

    range_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
    range_msg.field_of_view = 0.26;
    range_msg.min_range = 0.01;
    range_msg.max_range = 0.90;
    for (uint8_t i = 0; i < ranges_data.size; i++) {
      range_msg.header.frame_id.data = (char*)range_frame_names[i];
      range_msg.range = ranges_data.position[i];
      RCCHECK_WARN(rcl_publish(&range_pub, &range_msg, NULL));
    }
  }
}

void publishWheelsJointState() {
  RCCHECK_WARN(rcl_publish(&motor_state_pub, &motors_joint_state_msg, NULL));
  return;

  static motor_joint_state_t motor_joint_state;
  if (xQueueReceive(rtos::queues::MotorStateQueue, &motor_joint_state,
                    (TickType_t)0) == pdPASS) {
    if (rmw_uros_epoch_synchronized()) {
      motors_joint_state_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
      motors_joint_state_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
    }
    motors_joint_state_msg.velocity.data = motor_joint_state.velocity;
    motors_joint_state_msg.position.data = motor_joint_state.position;
    RCCHECK_WARN(rcl_publish(&motor_state_pub, &motors_joint_state_msg, NULL));
  }
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

}  // namespace u_ros
