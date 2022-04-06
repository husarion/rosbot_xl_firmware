/**
 * @file micro_ros_cfg.h
 * @author Maciej Kurcius
 * @brief 
 * @version 0.1
 * @date 2022-04-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef MICRO_ROS_CFG_H
#define MICRO_ROS_CFG_H

/*===== MICRO ROS =====*/
#include <micro_ros_arduino.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
/*===== ROS MSGS TYPES =====*/
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int64.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/joint_state.h>
/*===== REST =====*/
#include "hardware_cfg.h"

//Motors msgs defines
#define MOT_CMD_MSG_LEN         4
#define MOT_CMD_MSG_NAMES_LEN   25
#define MOT_CMD_MSG_FR_ID_LEN   20
#define MOT_RESP_MSG_LEN        4
#define FRONT_LEFT_MOTOR_NAME   "front_left_wheel_joint"
#define FRONT_RIGHT_MOTOR_NAME  "front_right_wheel_joint"
#define REAR_LEFT_MOTOR_NAME    "rear_left_wheel_joint"
#define REAR_RIGHT_MOTOR_NAME   "rear_right_wheel_joint"
#define MOTORS_RESPONSE_FREQ    50
//Anoter defines
#define NODE_NAME "stm32_node"

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      Serial.printf("o");          \
    }                              \
  }
#define RCSOFTCHECK(fn)            \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      Serial.printf("!");          \
    }                              \
  }

// extern void error_loop()
void uRosMotorsCmdCallback(const void *msgin);
void uRosTimerCallback(rcl_timer_t *timer, int64_t last_call_time);
bool uRosCreateEntities(void);
bool uRosDestroyEntities(void);
void uRosExecutorLoopHandler(void);
void MotorsResponseMsgInit(sensor_msgs__msg__JointState* msg);
void MotorsCmdMsgInit(sensor_msgs__msg__JointState* msg);

#endif /* MICRO_ROC_CFG_H */