/**
 * @file micro_ros_cfg.cpp
 * @author Maciej Kurcius
 * @brief
 * @version 0.1
 * @date 2022-04-05
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <micro_ros_cfg.h>
#include <battery_types.h>

// ROS PUBLISHERS
rcl_publisher_t imu_publisher;
rcl_publisher_t motor_state_publisher;
rcl_publisher_t battery_state_publisher;
// ROS SUBSCRIPTIONS
rcl_subscription_t subscriber;
rcl_subscription_t motors_cmd_subscriber;
#if defined(BOARD_ROSBOT_2)
rcl_subscription_t left_led_subscriber;
#endif
// ROS MESSAGES
sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__String msgs;
std_msgs__msg__Float32MultiArray motors_cmd_msg;
#if defined(BOARD_ROSBOT_2)
std_msgs__msg__Bool led_msg;
#endif
sensor_msgs__msg__JointState motors_response_msg;
sensor_msgs__msg__BatteryState battery_state_msg;
// ROS SERVICES
rcl_service_t get_cpu_id_service;
// ROS REQUESTS AND RESPONSES
std_srvs__srv__Trigger_Request get_cpu_id_service_request;
std_srvs__srv__Trigger_Response get_cpu_id_service_response;
// ROS
rcl_init_options_t init_options;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
uRosFunctionStatus ping_agent_status;
// REST
extern FirmwareModeTypeDef firmware_mode;

void ErrorLoop(const char * func)
{
  for (int i = 0; i < 4; ++i) {
    PRINT_DEBUG("In error loop from function %s", func)
    SetRedLed(Toggle);
    SetGreenLed(Off);
    delay(500);
  }
  // Reset the uC when microros fails
  NVIC_SystemReset();
}


void uRosTransportInit(void)
{
#if defined(BOARD_ROSBOT_XL)
  IPAddress client_ip;
  IPAddress agent_ip;
  byte mac[] = {CLIENT_MAC_ADDR};
  client_ip.fromString(CLIENT_IP);
  agent_ip.fromString(SBC_AGENT_IP);
  set_microros_native_ethernet_udp_transports(mac, client_ip, agent_ip, AGENT_PORT);
#elif defined(BOARD_ROSBOT_2)
  rmw_uros_set_custom_transport(
    /* Enable XRCE framing */
    true,
    /* Arguments for open function */
    NULL,
    /* Open transport callback */
    [](struct uxrCustomTransport * transport) -> bool {
      SBC_SERIAL.setRx(SBC_SERIAL_RX);
      SBC_SERIAL.setTx(SBC_SERIAL_TX);
      SBC_SERIAL.setTimeout(SBC_SERIAL_TIMEOUT);
      SBC_SERIAL.begin(SBC_SERIAL_BAUDRATE);
      return SBC_SERIAL ? true : false;
    },
    /* Close transport callback */
    [](struct uxrCustomTransport * transport) -> bool {
      SBC_SERIAL.end();
      return true;
    },
    /* Write transport callback */
    [](struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode) -> unsigned int {
      return SBC_SERIAL.write(buf, len);
    },
    /* Read transport callback */
    [](struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode) -> unsigned int {
      SBC_SERIAL.setTimeout(timeout);
      return SBC_SERIAL.readBytes((char *)buf, len);
    }
  );
#endif
}

uRosFunctionStatus uRosPingAgent(void)
{
  if (rmw_uros_ping_agent(PING_AGENT_TIMEOUT, PING_AGENT_ATTEMPTS) == RMW_RET_OK)
    return Ok;
  else
    return Error;  // if false
}

uRosFunctionStatus uRosPingAgent(uint8_t arg_timeout, uint8_t arg_attempts)
{
  if (rmw_uros_ping_agent((int)arg_timeout, arg_attempts) == RMW_RET_OK)
    return Ok;
  else
    return Error;  // if false
}

uRosFunctionStatus uRosLoopHandler(uRosFunctionStatus arg_agent_ping_status)
{
  static uRosEntitiesStatus entities_status = NotCreated;
  if (arg_agent_ping_status == Ok) {
    if (entities_status != Created) {
      entities_status = uRosCreateEntities();
      return Pending;
    } else {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      return Ok;
    }
  } else {
    if (entities_status != Destroyed && entities_status != NotCreated) {
      entities_status = uRosDestroyEntities();
      entities_status = Destroyed;
      return Error;
    }
  }
  return Default;
}

void uRosMotorsCmdCallback(const void * arg_input_message)
{
  static double setpoint[] = {0, 0, 0, 0};
  static std_msgs__msg__Float32MultiArray * setpoint_msg;
  setpoint_msg = (std_msgs__msg__Float32MultiArray *)arg_input_message;
  if (setpoint_msg->data.size == 4) {
    for (uint8_t i = 0; i < setpoint_msg->data.size; i++) {
      setpoint[i] = (double)setpoint_msg->data.data[i];
    }
  }
  xQueueSendToFront(SetpointQueue, (void *)setpoint, (TickType_t)0);
}

#if defined(BOARD_ROSBOT_2)
void uRosLeftLedCallback(const void * arg_led_msg)
{
  std_msgs__msg__Bool * led_msg = (std_msgs__msg__Bool *)arg_led_msg;
  if (led_msg->data == true) {
    SetGreenLed(On);
  } else {
    SetGreenLed(Off);
  }
}
#endif

void uRosTimerCallback(rcl_timer_t * arg_timer, int64_t arg_last_call_time)
{
  RCLC_UNUSED(arg_last_call_time);
  static imu_queue_t queue_imu;
  static motor_state_queue_t motor_state_queue;
  static battery_state_queue_t battery_state_queue;
  if (arg_timer != NULL) {
    // QOS default
    if (xQueueReceive(BatteryStateQueue, &battery_state_queue, (TickType_t)0) == pdPASS) {
      if (rmw_uros_epoch_synchronized()) {
        battery_state_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
        battery_state_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
      }
      battery_state_msg.voltage = battery_state_queue.voltage;
      battery_state_msg.temperature = battery_state_queue.temperature;
      battery_state_msg.current = battery_state_queue.current;
      battery_state_msg.charge = battery_state_queue.charge_current;
      battery_state_msg.capacity = battery_state_queue.capacity;
      battery_state_msg.design_capacity = battery_state_queue.design_capacity;
      battery_state_msg.percentage = battery_state_queue.percentage;
      battery_state_msg.power_supply_status = battery_state_queue.status;
      battery_state_msg.power_supply_health = battery_state_queue.health;
      battery_state_msg.power_supply_technology = battery_state_queue.technology;
      battery_state_msg.present = battery_state_queue.present;
      battery_state_msg.cell_temperature.capacity = BATTERY_STATE_MSG_CELL_TEMPERATURE_ARRAY_SIZE;
      battery_state_msg.cell_temperature.size = BATTERY_STATE_MSG_CELL_TEMPERATURE_ARRAY_SIZE;
      battery_state_msg.cell_temperature.data = battery_state_queue.cell_temperature;
      battery_state_msg.cell_voltage.capacity = BATTERY_STATE_MSG_CELL_VOLTAGE_ARRAY_SIZE;
      battery_state_msg.cell_voltage.size = BATTERY_STATE_MSG_CELL_VOLTAGE_ARRAY_SIZE;
      battery_state_msg.cell_voltage.data = battery_state_queue.cell_voltage;
      RCSOFTCHECK(rcl_publish(&battery_state_publisher, &battery_state_msg, NULL));
    }
    // QOS best effort
    if (xQueueReceive(MotorStateQueue, &motor_state_queue, (TickType_t)0) == pdPASS) {
      if (rmw_uros_epoch_synchronized()) {
        motors_response_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
        motors_response_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
      }
      motors_response_msg.velocity.data = motor_state_queue.velocity;
      motors_response_msg.position.data = motor_state_queue.position;
      RCSOFTCHECK(rcl_publish(&motor_state_publisher, &motors_response_msg, NULL));
    }
    // QOS best effort
    if (xQueueReceive(ImuQueue, &queue_imu, (TickType_t)0) == pdPASS) {
      if (rmw_uros_epoch_synchronized()) {
        imu_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
        imu_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
      } else {
        PRINT_DEBUG("!rmw_uros_epoch_synchronized");
      }
      imu_msg.header.frame_id.data = (char *)"imu_link";
      imu_msg.orientation.x = queue_imu.Orientation[0];
      imu_msg.orientation.y = queue_imu.Orientation[1];
      imu_msg.orientation.z = queue_imu.Orientation[2];
      imu_msg.orientation.w = queue_imu.Orientation[3];
      imu_msg.angular_velocity.x = queue_imu.AngularVelocity[0];
      imu_msg.angular_velocity.y = queue_imu.AngularVelocity[1];
      imu_msg.angular_velocity.z = queue_imu.AngularVelocity[2];
      imu_msg.linear_acceleration.x = queue_imu.LinearAcceleration[0];
      imu_msg.linear_acceleration.y = queue_imu.LinearAcceleration[1];
      imu_msg.linear_acceleration.z = queue_imu.LinearAcceleration[2];
      RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    }
  }
}

void uRosGetIdCallback(const void * req, void * res)
{
  (void)req;  // Unused parameter

  const uint32_t ADDRESS = 0x1FFF7A10;
  const uint8_t NUM_BYTES = 12;
  uint8_t buffer[NUM_BYTES];
  memcpy(buffer, (void *)ADDRESS, NUM_BYTES);

  // Prepare the CPU ID in hexadecimal format
  char cpu_id_buffer[NUM_BYTES * 2 + 1] = {0};
  char * hex_ptr = cpu_id_buffer;
  for (uint8_t i = 0; i < NUM_BYTES; ++i) {
    snprintf(hex_ptr, 3, "%02X", buffer[i]);
    hex_ptr += 2;
  }

  // Prepare the final output buffer with "CPU ID: " prefix
  static char out_buffer[100];  // Ensure this is large enough
  snprintf(out_buffer, sizeof(out_buffer), "{\"cpu_id\": \"%s\"}", cpu_id_buffer);

  // Set the response
  std_srvs__srv__Trigger_Response * response = (std_srvs__srv__Trigger_Response *)res;
  response->success = true;
  response->message.data = out_buffer;
  response->message.size = strlen(out_buffer);
}

uRosEntitiesStatus uRosCreateEntities(void)
{
  uint8_t ros_msgs_cnt = 0;
  /*===== ALLCOATE MEMORY FOR MSGS =====*/
  MotorsResponseMsgInit(&motors_response_msg);
  MotorsCmdMsgInit(&motors_cmd_msg);
  allocator = rcl_get_default_allocator();
  // create init_options
  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, UXR_CLIENT_DOMAIN_ID_TO_OVERRIDE_WITH_ENV));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  PRINT_DEBUG("Created support with option domain_id=%d", UXR_CLIENT_DOMAIN_ID_TO_OVERRIDE_WITH_ENV);

  // create node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));
  PRINT_DEBUG("Created node '%s'", NODE_NAME)
  /*===== INIT TIMERS =====*/
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(10), uRosTimerCallback));
  ros_msgs_cnt++;
  PRINT_DEBUG("Created timer")
  /*===== INIT SUBSCRIBERS ===== */
  RCCHECK(rclc_subscription_init_best_effort(
    &motors_cmd_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    MOTORS_CMD_TOPIC_NAME));
  PRINT_DEBUG("Created '%s' subscriber.", MOTORS_CMD_TOPIC_NAME)
  ros_msgs_cnt++;
#if defined(BOARD_ROSBOT_2)
  RCCHECK(rclc_subscription_init_default(
    &left_led_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    LEFT_LED_TOPIC_NAME));
  PRINT_DEBUG("Created '%s' subscriber.", LEFT_LED_TOPIC_NAME)
  ros_msgs_cnt++;
#endif
  /*===== INIT PUBLISHERS ===== */
  // IMU
  RCCHECK(rclc_publisher_init_best_effort(
    &imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), IMU_TOPIC_NAME));
  PRINT_DEBUG("Created '%s' publisher.", IMU_TOPIC_NAME)
  // MOTORS RESPONSE
  RCCHECK(rclc_publisher_init_best_effort(
    &motor_state_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), MOTOR_STATE_TOPIC_NAME));
  PRINT_DEBUG("Created '%s' publisher.", MOTOR_STATE_TOPIC_NAME)
  // BATTERY STATE
  RCCHECK(rclc_publisher_init_best_effort(
    &battery_state_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), BATTERY_TOPIC_NAME));
  PRINT_DEBUG("Created '%s' publisher.", BATTERY_TOPIC_NAME)
  /*===== INIT SERVICES ===== */
  std_srvs__srv__Trigger_Request__init(&get_cpu_id_service_request);
  std_srvs__srv__Trigger_Response__init(&get_cpu_id_service_response);
  RCCHECK(rclc_service_init_default(
    &get_cpu_id_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger), GET_CPU_ID_SERVICE_NAME));
  ros_msgs_cnt++;
  PRINT_DEBUG("Created '%s' service.", GET_CPU_ID_SERVICE_NAME)
  /*===== CREATE ENTITIES ===== */
  RCCHECK(rclc_executor_init(&executor, &support.context, ros_msgs_cnt, &allocator));
  PRINT_DEBUG("rclc_executor_init")
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  PRINT_DEBUG("rclc_executor_add_timer")
  RCCHECK(rclc_executor_add_subscription(
    &executor, &motors_cmd_subscriber, &motors_cmd_msg, &uRosMotorsCmdCallback, ON_NEW_DATA));
  PRINT_DEBUG("rclc_executor_add_subscription uRosMotorsCmdCallback")
#if defined(BOARD_ROSBOT_2)
  RCCHECK(rclc_executor_add_subscription(
    &executor, &left_led_subscriber, &led_msg, &uRosLeftLedCallback, ON_NEW_DATA));
  PRINT_DEBUG("rclc_executor_add_subscription left_led_subscriber")
#endif
  RCCHECK(rclc_executor_add_service(
    &executor, &get_cpu_id_service, &get_cpu_id_service_request, &get_cpu_id_service_response,
    uRosGetIdCallback));
  PRINT_DEBUG("Executor started")

  RCCHECK(rmw_uros_sync_session(1000));
  PRINT_DEBUG("Clocks synchronised")

  return Created;
}

uRosEntitiesStatus uRosDestroyEntities(void)
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCCHECK(rcl_publisher_fini(&imu_publisher, &node));
  RCCHECK(rcl_publisher_fini(&motor_state_publisher, &node));
  RCCHECK(rcl_publisher_fini(&battery_state_publisher, &node));
  RCCHECK(rcl_subscription_fini(&motors_cmd_subscriber, &node));
#if defined(BOARD_ROSBOT_2)
  RCCHECK(rcl_subscription_fini(&left_led_subscriber, &node));
#endif
  RCCHECK(rcl_service_fini(&get_cpu_id_service, &node));
  RCCHECK(rcl_timer_fini(&timer));
  RCCHECK(rclc_executor_fini(&executor));
  RCCHECK(rcl_node_fini(&node));
  RCCHECK(rclc_support_fini(&support));
  RCCHECK(rcl_init_options_fini(&init_options));
  PRINT_DEBUG("Destroyed all microros entities.")
  return Destroyed;
}

void MotorsResponseMsgInit(sensor_msgs__msg__JointState * arg_message)
{
  static rosidl_runtime_c__String msg_name_tab[MOT_RESP_MSG_LEN];
  static double msg_data_tab[3][MOT_RESP_MSG_LEN];
  char * frame_id = (char *)"motors_response";
  arg_message->position.data = msg_data_tab[0];
  arg_message->position.capacity = arg_message->position.size = MOT_RESP_MSG_LEN;
  arg_message->velocity.data = msg_data_tab[1];
  arg_message->velocity.capacity = arg_message->velocity.size = MOT_RESP_MSG_LEN;
  arg_message->effort.data = msg_data_tab[2];
  arg_message->effort.capacity = arg_message->effort.size = MOT_RESP_MSG_LEN;
  arg_message->header.frame_id.data = frame_id;
  arg_message->header.frame_id.capacity = arg_message->header.frame_id.size =
    strlen((const char *)frame_id);
  msg_name_tab->capacity = msg_name_tab->size = MOT_RESP_MSG_LEN;
  msg_name_tab[0].data = (char *)REAR_RIGHT_MOTOR_NAME;
  msg_name_tab[1].data = (char *)REAR_LEFT_MOTOR_NAME;
  msg_name_tab[2].data = (char *)FRONT_RIGHT_MOTOR_NAME;
  msg_name_tab[3].data = (char *)FRONT_LEFT_MOTOR_NAME;
  for (uint8_t i = 0; i < MOT_RESP_MSG_LEN; i++) {
    msg_name_tab[i].capacity = msg_name_tab[i].size = strlen(msg_name_tab[i].data);
  }
  arg_message->name.capacity = arg_message->name.size = MOT_RESP_MSG_LEN;
  arg_message->name.data = msg_name_tab;
}

void MotorsCmdMsgInit(std_msgs__msg__Float32MultiArray * arg_message)
{
  static float data[MOT_CMD_MSG_LEN] = {0, 0, 0, 0};
  arg_message->data.capacity = MOT_CMD_MSG_LEN;
  arg_message->data.size = MOT_CMD_MSG_LEN;
  arg_message->data.data = (float *)data;
}
