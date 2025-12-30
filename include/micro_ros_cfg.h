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
#include <std_msgs/msg/bool.h>
// #include <std_msgs/msg/int64.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32_multi_array.h>
/*===== ROS SRVS TYPES =====*/
#include <std_srvs/srv/trigger.h>
/*===== REST =====*/
#include <ImuLib_cfg.h>
#include <STM32FreeRTOS.h>
#include <bsp.h>
#include <hardware_cfg.h>

#define UXR_CLIENT_DOMAIN_ID_TO_OVERRIDE_WITH_ENV 255  // get ROS_DOMAIN_ID from Micro ROS Agent

/* DEFINES */
#define PING_AGENT_TIMEOUT 10
#define PING_AGENT_ATTEMPTS 100
#define PING_AGENT_FREQUENCY (double)1  // Hz
// Motors msgs defines
#define MOT_CMD_MSG_LEN 4
#define MOT_RESP_MSG_LEN 4
#define FRONT_LEFT_MOTOR_NAME "fl_wheel_joint"
#define FRONT_RIGHT_MOTOR_NAME "fr_wheel_joint"
#define REAR_LEFT_MOTOR_NAME "rl_wheel_joint"
#define REAR_RIGHT_MOTOR_NAME "rr_wheel_joint"
#define MOTORS_RESPONSE_FREQ 50
// uRos topics
#define IMU_TOPIC_NAME              "_imu/data_raw"
#define MOTORS_CMD_TOPIC_NAME       "_motors_cmd"
#define MOTOR_STATE_TOPIC_NAME      "_motors_response"
#define GET_CPU_ID_SERVICE_NAME     "/get_cpu_id"
#if defined(BOARD_ROSBOT_XL)
  #define BATTERY_TOPIC_NAME        "battery_state"
  #define NODE_NAME                 "stm32_node"
#elif defined(BOARD_CORE_2)
  #define BATTERY_TOPIC_NAME        "battery"
  #define LEFT_LED_TOPIC_NAME       "led/left"
  #define RIGHT_LED_TOPIC_NAME      "led/right"
  #define NODE_NAME                 "rosbot_ros2_firmware"
#endif

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      PRINT_DEBUG("RCCHECK FAILED due to return code: %d in function %s()", temp_rc, __FUNCTION__);          \
      ErrorLoop(__FUNCTION__);     \
    }                              \
  }
#define RCSOFTCHECK(fn)            \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      PRINT_DEBUG("RCSOFTCHECK FAILED due to return code: %d in function %s()", temp_rc, __FUNCTION__);          \
    }                              \
  }

/* TYPE DEF */
typedef struct
{
  uint8_t size = 4;
  double velocity[4];
  double position[4];
} motor_state_queue_t;

typedef enum { Default = 0, Ok = 1, Error = 2, InvalidInput = 3, Pending = 4 } uRosFunctionStatus;

typedef enum { NotCreated = 0, Created = 1, Destroyed = 3 } uRosEntitiesStatus;

/* EXTERN */
// extern variables
extern QueueHandle_t SetpointQueue;
extern QueueHandle_t MotorStateQueue;
extern QueueHandle_t ImuQueue;  // extern functions
extern "C" int clock_gettime(clockid_t unused, struct timespec * tp);

/* FUNCTIONS */
void ErrorLoop(const char * func);
void uRosTransportInit(void);
uRosFunctionStatus uRosPingAgent(void);
uRosFunctionStatus uRosPingAgent(uint8_t arg_timeout, uint8_t arg_attempts);
uRosFunctionStatus uRosLoopHandler(uRosFunctionStatus arg_agent_ping_status);
void uRosMotorsCmdCallback(const void * arg_input_message);
void uRosTimerCallback(rcl_timer_t * arg_timer, int64_t arg_last_call_time);
uRosEntitiesStatus uRosCreateEntities(void);
uRosEntitiesStatus uRosDestroyEntities(void);
void MotorsResponseMsgInit(sensor_msgs__msg__JointState * arg_message);
void MotorsCmdMsgInit(std_msgs__msg__Float32MultiArray * arg_message);

#endif /* MICRO_ROC_CFG_H */
