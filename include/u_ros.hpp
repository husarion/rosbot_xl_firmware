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
#include <std_msgs/msg/float32_multi_array.h>

/*===== REST =====*/
#include "log.hpp"
#include "robot_config.hpp"
namespace u_ros {
#define UXR_CLIENT_DOMAIN_ID_TO_OVERRIDE_WITH_ENV \
  255  // get ROS_DOMAIN_ID from Micro ROS Agent

/* DEFINES */
#define uROS_PING_TIMEOUT_MS 50
#define uROS_PING_ATTEMPTS 10
// Motors msgs defines
#define MOT_CMD_MSG_LEN 4
// uRos topics
#define NODE_NAME "rosbot_mcu"

#define RCCHECK_RETURN(fn)              \
  {                                     \
    rcl_ret_t rc = fn;                  \
    if (rc != RCL_RET_OK) return false; \
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
extern u_ros_state_t state;

/* FUNCTIONS */
void transportInit(const SerialConfig& config);
bool pingAgent();
void loop();
bool createEntities();
void destroyEntities();

void motorsCmdCallback(const void* msg_in);

void initMotorsCmdMsg(std_msgs__msg__Float32MultiArray* msg);

}  // namespace u_ros
