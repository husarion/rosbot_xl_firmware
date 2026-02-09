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
#include <sensor_msgs/msg/joint_state.h>
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
#define uROS_PING_FREQUENCY 5.0
#define uROS_SPIN_DELAY_MS 1
// Motors msgs defines
#define MOT_CMD_MSG_LEN 4
#define MOT_RESP_MSG_LEN 4
#define MOTORS_RESPONSE_FREQ 50
// uRos topics
#define NODE_NAME "rosbot_hw"

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

// ============================================================================
// TRANSPORT CONFIGURATION STRUCTURE
// ============================================================================
struct TransportConfig {
  HardwareSerial* serial;
  uint8_t rx_pin;
  uint8_t tx_pin;
  uint32_t baudrate;
  uint32_t timeout_ms;
};

// ============================================================================
// PREDEFINED CONFIGURATIONS
// ============================================================================
namespace TransportConfigs {

// SBC Communication (Raspberry Pi / Jetson)
inline constexpr TransportConfig SBC = {.serial = &Serial1,
                                        .rx_pin = PA10,
                                        .tx_pin = PA9,
                                        .baudrate = 921600,
                                        .timeout_ms = 10};

// FTDI Interface
inline constexpr TransportConfig FTDI = {.serial = &Serial3,
                                         .rx_pin = PA3,
                                         .tx_pin = PA2,
                                         .baudrate = 921600,
                                         .timeout_ms = 10};

}  // namespace TransportConfigs

/* EXTERN */
extern "C" int clock_gettime(clockid_t unused, struct timespec* tp);
extern u_ros_state_t state;

/* FUNCTIONS */
inline void transportInit(const SerialConfig& config) {
  rmw_uros_set_custom_transport(
      /* Enable XRCE framing */
      true,
      /* Arguments for callbacks - pass config pointer */
      (void*)&config,

      /* Open transport callback */
      [](struct uxrCustomTransport* transport) -> bool {
        const SerialConfig* cfg = (const SerialConfig*)transport->args;
        cfg->serial->setRx(cfg->rxPin);
        cfg->serial->setTx(cfg->txPin);
        cfg->serial->setTimeout(cfg->timeout_ms);
        cfg->serial->begin(cfg->baudrate);
        return cfg->serial->operator bool();
      },

      /* Close transport callback */
      [](struct uxrCustomTransport* transport) -> bool {
        const SerialConfig* cfg = (const SerialConfig*)transport->args;
        cfg->serial->end();
        return true;
      },

      /* Write transport callback */
      [](struct uxrCustomTransport* transport, const uint8_t* buf, size_t len,
         uint8_t* errcode) -> unsigned int {
        const SerialConfig* cfg = (const SerialConfig*)transport->args;
        return cfg->serial->write(buf, len);
      },

      /* Read transport callback */
      [](struct uxrCustomTransport* transport, uint8_t* buf, size_t len,
         int timeout, uint8_t* errcode) -> unsigned int {
        const SerialConfig* cfg = (const SerialConfig*)transport->args;
        cfg->serial->setTimeout(timeout);
        return cfg->serial->readBytes((char*)buf, len);
      });
}

bool pingAgent();
void loop();
bool createEntities();
void destroyEntities();

void motorsCmdCallback(const void* msg_in);

void initMotorsCmdMsg(std_msgs__msg__Float32MultiArray* msg);
void initMotorsJointStateMsg(sensor_msgs__msg__JointState* msg);

void publishJointState();

}  // namespace u_ros
