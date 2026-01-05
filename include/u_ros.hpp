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

#pragma once

/*===== MICRO ROS =====*/
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

/*===== ROS MSGS TYPES =====*/
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32_multi_array.h>

/*===== REST =====*/
#include "hardware_cfg.hpp"
#include "log.hpp"

namespace u_ros {
#define UXR_CLIENT_DOMAIN_ID_TO_OVERRIDE_WITH_ENV \
  255  // get ROS_DOMAIN_ID from Micro ROS Agent

/* DEFINES */
#define uROS_PING_TIMEOUT_MS 50
#define uROS_PING_ATTEMPTS 2
#define uROS_PING_FREQUENCY 5.0
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
#define NODE_NAME "rosbot_hw"

#define RCCHECK_RETURN(fn)              \
  {                                     \
    rcl_ret_t rc = fn;                  \
    if (rc != RCL_RET_OK) return false; \
  }

#define RCCHECK(fn)                                                           \
  {                                                                           \
    rcl_ret_t rc = fn;                                                        \
    if ((rc != RCL_RET_OK)) {                                                 \
      LOG_DEBUG("RCCHECK FAILED due to return code: %d in function %s()", rc, \
                __FUNCTION__);                                                \
      errorLoop(__FUNCTION__);                                                \
    }                                                                         \
  }
#define RCCHECK_WARN(fn)                                                     \
  {                                                                          \
    rcl_ret_t rc = fn;                                                       \
    if ((rc != RCL_RET_OK)) {                                                \
      LOG_WARN("RCSOFTCHECK FAILED due to return code: %d in function %s()", \
               rc, __FUNCTION__);                                            \
    }                                                                        \
  }

typedef enum {
  WAITING,
  AGENT_AVAILABLE,
  CONNECTED,
  DISCONNECTED
} u_ros_state_t;

/* EXTERN */
extern "C" int clock_gettime(clockid_t unused, struct timespec* tp);

/* FUNCTIONS */
void errorLoop(const char* func);
void transportInit(void);
bool pingAgent(void);
void loop();
bool createEntities(void);
void destroyEntities(void);

void motorsCmdCallback(const void* input_message);
void timerCallback(rcl_timer_t* timer, int64_t last_call_time);

void initBatteryMsg(sensor_msgs__msg__BatteryState* msg);
void initImuMsg(sensor_msgs__msg__Imu* msg);
void initMotorsCmdMsg(std_msgs__msg__Float32MultiArray* msg);
void initMotorsJointStateMsg(sensor_msgs__msg__JointState* msg);
void initRangeMsg(sensor_msgs__msg__Range* msg);

void publishBattery();
void publishImu();
void publishRanges();
void publishWheelsJointState();

}  // namespace u_ros
