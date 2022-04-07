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
#include <hardware_cfg.h>
#include <STM32FreeRTOS.h>
#include <ImuLib_cfg.h>
#include <bsp.h>

/* DEFINES */
#define NODE_NAME                   "stm32_node"
#define AGENT_RECONNECTION_TIMEOUT  50
#define AGENT_RECONNECTION_ATTEMPTS 2
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

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      ErrorLoop();                \
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

/* TYPE DEF */
typedef struct {
  uint8_t size = 4;
  double velocity[4];
  double positon[4];
} motor_state_queue_t;

typedef enum{
  Ok            = 0,
  Error         = 1,
  InvalidInput  = 2,
  Pending       = 3
}uRosFunctionStatus;

/* EXTERN */
//extern variables
extern QueueHandle_t SetpointQueue;
extern QueueHandle_t MotorStateQueue;
extern QueueHandle_t ImuQueue;\
//extern functions
extern "C" int clock_gettime(clockid_t unused, struct timespec *tp);

/* FUNCTIONS */
void ErrorLoop(void);
uRosFunctionStatus uRosPingAgent(void);
uRosFunctionStatus uRosPingAgent(uint8_t Timeout_, uint8_t Attempts_);
uRosFunctionStatus uRosLoopHandler(void);
void uRosMotorsCmdCallback(const void *msgin);
void uRosTimerCallback(rcl_timer_t *timer, int64_t last_call_time);
bool uRosCreateEntities(void);
bool uRosDestroyEntities(void);
void MotorsResponseMsgInit(sensor_msgs__msg__JointState* msg);
void MotorsCmdMsgInit(sensor_msgs__msg__JointState* msg);

#endif /* MICRO_ROC_CFG_H */