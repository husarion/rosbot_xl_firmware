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

//ROS PUBLISHERS
rcl_publisher_t imu_publisher;
rcl_publisher_t motor_state_publisher;
rcl_publisher_t battery_state_publisher;
//ROS SUBSCRIPTIONS
rcl_subscription_t subscriber;
rcl_subscription_t motors_cmd_subscriber;
//ROS MESSAGES

std_msgs__msg__String msgs;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__JointState motors_cmd_msg;
sensor_msgs__msg__JointState motors_response_msg;
sensor_msgs__msg__BatteryState battery_state_msg;
//ROS
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;


void ErrorLoop(void){
  while (1) {
    if(BOARD_MODE_DEBUG) Serial.printf("In error loop");
    SetRedLed(Toggle);
    SetGreenLed(Off);
    delay(1000);
  }
}

uRosFunctionStatus uRosPingAgent(void){
  if(rmw_uros_ping_agent(AGENT_RECONNECTION_TIMEOUT, AGENT_RECONNECTION_ATTEMPTS) == RMW_RET_OK)
      return Ok;
  else
    return Error;  //if false
}

uRosFunctionStatus uRosPingAgent(uint8_t Timeout_, uint8_t Attempts_){
  if(rmw_uros_ping_agent((int)Timeout_, Attempts_) == RMW_RET_OK)
      return Ok;
    else
      return Error;  //if false
}

uRosFunctionStatus uRosLoopHandler(void){
  static uRosEntitiesStatus EntitiesStatus = NotCreated;
  if(uRosPingAgent() == Ok){
    if(EntitiesStatus != Created){
      EntitiesStatus = uRosCreateEntities();
      return Pending;
    }
    else{
      if(EntitiesStatus == Created)
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
      return Ok;
    }
  }
  else{
    if(EntitiesStatus != Destroyed)
      EntitiesStatus = uRosDestroyEntities();
    return Error;
  }
}

void uRosMotorsCmdCallback(const void *msgin){
  static double Setpoint[] = {0,0,0,0};
  static sensor_msgs__msg__JointState * setpoint_msg;
  setpoint_msg = (sensor_msgs__msg__JointState *)msgin;
  String motor_name;
  for(uint8_t i = 0; i < (uint8_t)setpoint_msg->name.size; i++){
    motor_name = (String)setpoint_msg->name.data[i].data;
    if(motor_name == REAR_RIGHT_MOTOR_NAME)  Setpoint[0] = (double)setpoint_msg->velocity.data[i];
    if(motor_name == REAR_LEFT_MOTOR_NAME)   Setpoint[1] = (double)setpoint_msg->velocity.data[i];
    if(motor_name == FRONT_RIGHT_MOTOR_NAME) Setpoint[2] = (double)setpoint_msg->velocity.data[i];
    if(motor_name == FRONT_LEFT_MOTOR_NAME)  Setpoint[3] = (double)setpoint_msg->velocity.data[i];
  }
  xQueueSendToFront(SetpointQueue, (void*) Setpoint, (TickType_t) 0);
}

void uRosTimerCallback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  static imu_queue_t queue_imu;
  static motor_state_queue_t motor_state_queue;
  static battery_state_queue_t battery_state_queue;
  struct timespec ts = {0};
  if (timer != NULL) {
    //QOS default
    if(xQueueReceive(BatteryStateQueue, &battery_state_queue, (TickType_t)0) == pdPASS){
      clock_gettime(CLOCK_REALTIME, &ts);
      battery_state_msg.header.stamp.sec = ts.tv_sec;
      battery_state_msg.header.stamp.nanosec = ts.tv_nsec;
      battery_state_msg.voltage = battery_state_queue.Voltage;
      battery_state_msg.temperature = battery_state_queue.Temperature;
      battery_state_msg.current = battery_state_queue.Current;
      battery_state_msg.charge = battery_state_queue.ChargeCurrent;
      battery_state_msg.capacity = battery_state_queue.Capacity;
      battery_state_msg.design_capacity = battery_state_queue.DesignCapacity;
      battery_state_msg.percentage = battery_state_queue.Percentage;
      battery_state_msg.power_supply_status = battery_state_queue.Status;
      battery_state_msg.power_supply_health = battery_state_queue.Health;
      battery_state_msg.power_supply_technology = battery_state_queue.Technology;
      battery_state_msg.present = battery_state_queue.Present;
      RCSOFTCHECK(rcl_publish(&battery_state_publisher, &battery_state_msg, NULL));
    }
    //QOS best effort
    if(xQueueReceive(MotorStateQueue, &motor_state_queue, (TickType_t) 0) == pdPASS){
      clock_gettime(CLOCK_REALTIME, &ts);
      motors_response_msg.header.stamp.sec = ts.tv_sec;
      motors_response_msg.header.stamp.nanosec = ts.tv_nsec;
      motors_response_msg.velocity.data = motor_state_queue.velocity;
      motors_response_msg.position.data = motor_state_queue.positon;
      RCSOFTCHECK(rcl_publish(&motor_state_publisher, &motors_response_msg, NULL));
    }
    //QOS best effort
    if(xQueueReceive(ImuQueue, &queue_imu, (TickType_t) 0) == pdPASS){
      clock_gettime(CLOCK_REALTIME, &ts);
      imu_msg.header.stamp.sec = ts.tv_sec;
      imu_msg.header.stamp.nanosec = ts.tv_nsec;
      imu_msg.header.frame_id.data = (char *) "imu";
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

uRosEntitiesStatus uRosCreateEntities(void){
  uint8_t ros_msgs_cnt = 0;
  /*===== ALLCOATE MEMORY FOR MSGS =====*/
  MotorsResponseMsgInit(&motors_response_msg);
  MotorsCmdMsgInit(&motors_cmd_msg);
  allocator = rcl_get_default_allocator();
  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator))
  // create node

  // Get default node options and modify them
  // rcl_node_options_t node_ops = rcl_node_get_default_options();
  // node_ops.domain_id = (size_t)(138);

  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));
  /*===== INIT TIMERS =====*/
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(10),
                                  uRosTimerCallback));
  ros_msgs_cnt++;
  if(BOARD_MODE_DEBUG) Serial.printf("Created timer\r\n");
  /*===== INIT SUBSCRIBERS ===== */
  RCCHECK(rclc_subscription_init_default(
      &motors_cmd_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "motors_cmd"));
  ros_msgs_cnt++;
  if(BOARD_MODE_DEBUG) Serial.printf("Created 'motors_cmd' subscriber\r\n");
  /*===== INIT PUBLISHERS ===== */
  //IMU
  RCCHECK(rclc_publisher_init_best_effort(
      &imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu/data_raw"));
  ros_msgs_cnt++;
  if(BOARD_MODE_DEBUG) Serial.printf("Created 'sensor_msgs/Imu' publisher.\r\n");
  //MOTORS RESPONSE
  RCCHECK(rclc_publisher_init_best_effort(
      &motor_state_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "motors_response"));
  ros_msgs_cnt++;
  if(BOARD_MODE_DEBUG) Serial.printf("Created 'motors_response' publisher.\r\n");
  //BATTERY STATE
  RCCHECK(rclc_publisher_init_default(
      &battery_state_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
      "battery_state"));
  ros_msgs_cnt++;
  if(BOARD_MODE_DEBUG) Serial.printf("Created 'battery_state' publisher.\r\n");
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, ros_msgs_cnt, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &motors_cmd_subscriber, &motors_cmd_msg,
                                      &uRosMotorsCmdCallback, ON_NEW_DATA));                                 
  if(BOARD_MODE_DEBUG) Serial.printf("Executor started\r\n");
  RCCHECK(rmw_uros_sync_session(1000));
  if(BOARD_MODE_DEBUG) Serial.printf("Clocks synchronised\r\n");
  return Created;
}

uRosEntitiesStatus uRosDestroyEntities(void){
  rcl_publisher_fini(&imu_publisher, &node);
  rcl_publisher_fini(&motor_state_publisher, &node);
	rcl_node_fini(&node);
	rcl_timer_fini(&timer);
	rclc_executor_fini(&executor);
	rclc_support_fini(&support);
  return Destroyed;
}

void MotorsResponseMsgInit(sensor_msgs__msg__JointState * msg){
  static double msg_data_tab[3][MOT_RESP_MSG_LEN];
  static rosidl_runtime_c__String msg_name_tab[MOT_RESP_MSG_LEN];
  char* frame_id = (char*)"motors_response";
  msg->position.data = msg_data_tab[0];
  msg->position.capacity = msg->position.size = MOT_RESP_MSG_LEN;
  msg->velocity.data = msg_data_tab[1];
  msg->velocity.capacity = msg->velocity.size = MOT_RESP_MSG_LEN;
  msg->effort.data = msg_data_tab[2];
  msg->effort.capacity = msg->effort.size = MOT_RESP_MSG_LEN;
  msg->header.frame_id.data = frame_id;
  msg->header.frame_id.capacity = msg->header.frame_id.size = strlen((const char*)frame_id);

  msg_name_tab->capacity = msg_name_tab->size = MOT_RESP_MSG_LEN;
  msg_name_tab[0].data = (char*)"rear_right_wheel_joint";
  msg_name_tab[1].data = (char*)"rear_left_wheel_joint";
  msg_name_tab[2].data = (char*)"front_right_wheel_joint";
  msg_name_tab[3].data = (char*)"front_left_wheel_joint";
  for(uint8_t i = 0; i < MOT_RESP_MSG_LEN; i++){
      msg_name_tab[i].capacity = msg_name_tab[i].size = strlen(msg_name_tab[i].data);
  }
  msg->name.capacity = msg->name.size = MOT_RESP_MSG_LEN;
  msg->name.data = msg_name_tab;
}

void MotorsCmdMsgInit(sensor_msgs__msg__JointState * msg){
  static char tab[4][24];
  static double pos[10]; 
  static double vel[10];
  static double eff[10];
  static rosidl_runtime_c__String str_name_tab[10];

  msg->position.capacity = 10;
  msg->position.data = pos;
  msg->effort.capacity = 10;
  msg->effort.data = eff;
  msg->velocity.capacity = 10;
  msg->velocity.data = vel;
  msg->header.frame_id.capacity = 20;

  str_name_tab->capacity = 4;

  str_name_tab[0].capacity = 24;
  str_name_tab[0].data = tab[0];

  str_name_tab[1].capacity = 24;
  str_name_tab[1].data = tab[1];

  str_name_tab[2].capacity = 24;
  str_name_tab[2].data = tab[2];

  str_name_tab[3].capacity = 24;
  str_name_tab[3].data = tab[3];

  msg->name.capacity = 4;
  msg->name.data->capacity = 24;
  msg->name.data = str_name_tab;
}

// void MotorsCmdMsgInit(sensor_msgs__msg__JointState * msg){
//     static double msg_data_tab[3][MOT_CMD_MSG_LEN];
//     static rosidl_runtime_c__String msg_name_tab[MOT_CMD_MSG_LEN];
//     static char msg_name_data_tab[MOT_CMD_MSG_LEN][MOT_CMD_MSG_NAMES_LEN];
//     static char msg_frame_id_data[MOT_CMD_MSG_FR_ID_LEN];

//     msg->position.data = msg_data_tab[0];
//     msg->position.capacity = MOT_CMD_MSG_LEN;
//     msg->velocity.data = msg_data_tab[1];
//     msg->velocity.capacity = MOT_CMD_MSG_LEN;
//     msg->effort.data = msg_data_tab[2];
//     msg->effort.capacity = MOT_CMD_MSG_LEN;
//     msg->header.frame_id.data = msg_frame_id_data;
//     msg->header.frame_id.capacity = MOT_CMD_MSG_FR_ID_LEN;
//     msg_name_tab->capacity = msg_name_tab->size = MOT_CMD_MSG_LEN;

//     for(uint8_t i = 0; i < MOT_CMD_MSG_LEN; i++){
//         msg_name_tab->capacity = MOT_CMD_MSG_NAMES_LEN;
//         msg_name_tab->data = (char*)msg_name_data_tab[i];
//     }
//     msg->name.capacity = MOT_CMD_MSG_LEN;
//     msg->name.data = msg_name_tab;
// }