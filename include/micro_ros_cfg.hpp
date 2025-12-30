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
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
// #include <std_msgs/msg/int64.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/float32_multi_array.h>
/*===== ROS SRVS TYPES =====*/
#include <std_srvs/srv/trigger.h>
/*===== REST =====*/
#include <STM32FreeRTOS.h>

#include "bsp.hpp"
#include "hardware/imu.hpp"
#include "hardware_cfg.hpp"
#include "log.hpp"

#define UXR_CLIENT_DOMAIN_ID_TO_OVERRIDE_WITH_ENV \
  255  // get ROS_DOMAIN_ID from Micro ROS Agent

/* DEFINES */
#define uROS_PING_TIMEOUT_MS 50
#define uROS_PING_ATTEMPTS 2
#define uROS_PING_FREQUENCY 1.0
#define uROS_SPIN_DELAY_MS 1
// Motors msgs defines
#define MOT_CMD_MSG_LEN 4
#define MOT_RESP_MSG_LEN 4
#define FRONT_LEFT_MOTOR_NAME "fl_wheel_joint"
#define FRONT_RIGHT_MOTOR_NAME "fr_wheel_joint"
#define REAR_LEFT_MOTOR_NAME "rl_wheel_joint"
#define REAR_RIGHT_MOTOR_NAME "rr_wheel_joint"
#define MOTORS_RESPONSE_FREQ 50
// uRos topics
#define NODE_NAME "stm32_node"

#define RCCHECK(fn)                                                       \
  {                                                                       \
    rcl_ret_t temp_rc = fn;                                               \
    if ((temp_rc != RCL_RET_OK)) {                                        \
      LOG_DEBUG("RCCHECK FAILED due to return code: %d in function %s()", \
                temp_rc, __FUNCTION__);                                   \
      ErrorLoop(__FUNCTION__);                                            \
    }                                                                     \
  }
#define RCSOFTCHECK(fn)                                                       \
  {                                                                           \
    rcl_ret_t temp_rc = fn;                                                   \
    if ((temp_rc != RCL_RET_OK)) {                                            \
      LOG_DEBUG("RCSOFTCHECK FAILED due to return code: %d in function %s()", \
                temp_rc, __FUNCTION__);                                       \
    }                                                                         \
  }

/* TYPE DEF */
typedef struct {
  uint8_t size = 4;
  double velocity[4];
  double position[4];
} motor_joint_state_t;

typedef enum {
  WAITING,
  AGENT_AVAILABLE,
  CONNECTED,
  DISCONNECTED
} u_ros_status_t;

typedef enum { NotCreated = 0, Created = 1, Destroyed = 3 } u_ros_entities_status_t;

/* EXTERN */
// extern variables
extern QueueHandle_t SetpointQueue;
extern QueueHandle_t MotorStateQueue;
extern QueueHandle_t ImuQueue;  // extern functions
extern QueueHandle_t RangeQueue;
extern "C" int clock_gettime(clockid_t unused, struct timespec* tp);

/* FUNCTIONS */
void ErrorLoop(const char* func);
void uRosTransportInit(void);
bool uRosPingAgent(void);
bool uRosPingAgent(int timeout_ms, uint8_t attempts);
void uRosLoopHandler(bool connected);
void uRosMotorsCmdCallback(const void* input_message);
void uRosTimerCallback(rcl_timer_t* timer, int64_t last_call_time);
bool uRosCreateEntities(void);
void uRosDestroyEntities(void);
void MotorsJointStateInit(sensor_msgs__msg__JointState* msg);
void MotorsCmdMsgInit(std_msgs__msg__Float32MultiArray* msg);
void publishBattery();
void publishImu();
void publishRanges();
void publishWheelsJointState();

#endif /* MICRO_ROC_CFG_H */
