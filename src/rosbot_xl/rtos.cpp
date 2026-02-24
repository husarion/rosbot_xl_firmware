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

#include "rtos.hpp"

#include <STM32FreeRTOS.h>

#include "battery_interface.hpp"
#include "config.hpp"
#include "encoder_array.hpp"
#include "imu_interface.hpp"
#include "led_indicator.hpp"
#include "motor_array.hpp"
#include "ros/publishers/battery_publisher.hpp"
#include "ros/publishers/imu_publisher.hpp"
#include "ros/publishers/joint_state_publisher.hpp"
#include "ros/ros_node.hpp"
#include "serial_manager.hpp"


// ===== Queues =====
void createQueues() {
  battery_queue = xQueueCreate(1, sizeof(BatteryStamped));
  imu_queue = xQueueCreate(1, sizeof(ImuStamped));
  joint_state_queue = xQueueCreate(1, sizeof(EncodersStamped));
}

// ===== Create all tasks =====
void batteryTask(void* p);
void encoderTask(void* p);
void imuTask(void* p);
void ledIndicatorTask(void* p);
void monitorTask(void* p);
void motorControlTask(void* p);
void uRosTask(void* p);
void uRosPingTask(void* p);

inline TaskConfig tasks[] = {
    {"Battery", Priority::SENSORS, Stack::SMALL, 10, batteryTask},
    {"Encoder", Priority::CONTROL, Stack::SMALL, 500, encoderTask},
    {"Imu", Priority::SENSORS, Stack::SMALL, 50, imuTask},
    {"LedIndicator", Priority::OBSERVING, Stack::XSMALL, 20, ledIndicatorTask},
    {"Monitor", Priority::OBSERVING, Stack::MEDIUM, 1, monitorTask},
    {"MotorControl", Priority::CONTROL, Stack::MEDIUM, 200, motorControlTask},
    {"uRos", Priority::COMMUNICATION, Stack::LARGE, 100, uRosTask},
    {"uRosPing", Priority::OBSERVING, Stack::MEDIUM, 2, uRosPingTask},
};

inline TaskHandleWrapper taskHandles[sizeof(tasks) / sizeof(tasks[0])];

void createTasks() {
  for (size_t i = 0; i < sizeof(tasks) / sizeof(tasks[0]); i++) {
    taskHandles[i].create(tasks[i]);
  }
}

// ===== Task functions =====
void batteryTask(void* p) {
  TickType_t period = taskGetPeriod(p);
  TickType_t wake_time = xTaskGetTickCount();
  BatteryStamped data = {};

  while (true) {
    bool connected = rtos_get_timestamp_ns(data.timestamp_ns);
    g_battery->update();  // TODO: DMA should be used
    data.data = g_battery->getData();

    if (connected) {
      xQueueOverwrite(battery_queue, &data);
    }
    vTaskDelayUntil(&wake_time, period);
  }
}

void encoderTask(void* p) {
  TickType_t period = taskGetPeriod(p);
  TickType_t wake_time = xTaskGetTickCount();
  EncodersStamped data = {};

  while (true) {
    bool connected = rtos_get_timestamp_ns(data.timestamp_ns);
    g_encoders.update();
    data.data = g_encoders.getData();

    if (connected) {
      xQueueOverwrite(joint_state_queue, &data);
    }
    vTaskDelayUntil(&wake_time, period);
  }
}

void imuTask(void* p) {
  TickType_t period = taskGetPeriod(p);
  TickType_t wake_time = xTaskGetTickCount();
  ImuStamped data = {};

  while (true) {
    bool connected = rtos_get_timestamp_ns(data.timestamp_ns);
    g_imu->update();  // TODO: DMA should be used
    data.data = g_imu->getData();

    if (connected) {
      xQueueOverwrite(imu_queue, &data);
    }
    vTaskDelayUntil(&wake_time, period);
  }
}

void ledIndicatorTask(void* p) {
  TickType_t period = taskGetPeriod(p);
  TickType_t wake_time = xTaskGetTickCount();

  while (true) {
    bool battery_low = g_battery->isLow();
    bool error_state = false;

    g_indicator.update(battery_low, !g_ros_node.isConnected(), error_state);
    vTaskDelayUntil(&wake_time, period);
  }
}

void monitorTask(void* p) {
  TickType_t period = taskGetPeriod(p);
  TickType_t wake_time = xTaskGetTickCount();
  char buf[1000];

  while (true) {
    vTaskGetRunTimeStats(buf);
    g_serialManager.debug().printf("%s\r\n", buf);

    vTaskDelayUntil(&wake_time, period);
  }
}

void motorControlTask(void* p) {
  TickType_t period = taskGetPeriod(p);
  TickType_t wake_time = xTaskGetTickCount();

  while (true) {
    g_motors.update();

    vTaskDelayUntil(&wake_time, period);
  }
}

void uRosTask(void* p) {
  TickType_t period = taskGetPeriod(p);
  TickType_t wake_time = xTaskGetTickCount();
  while (true) {
    g_ros_node.publishLoop();
    vTaskDelayUntil(&wake_time, period);
  }
}

void uRosPingTask(void* p) {
  TickType_t period = taskGetPeriod(p);
  TickType_t wake_time = xTaskGetTickCount();

  while (true) {
    g_ros_node.loop();
    vTaskDelayUntil(&wake_time, period);
  }
}
