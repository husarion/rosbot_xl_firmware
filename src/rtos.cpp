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
#include "control/encoders_manager.hpp"
#include "control/motors_manager.hpp"
#include "led_indicator.hpp"
#include "log.hpp"
#include "uros.hpp"
#include "imu_interface.hpp"

namespace rtos {

void createQueues() {
  BatteryQueue = xQueueCreate(1, sizeof(BatteryStamped));
  EncodersQueue = xQueueCreate(1, sizeof(EncodersData));
  ImuQueue = xQueueCreate(1, sizeof(ImuStamped));
  RangesQueue = xQueueCreate(1, sizeof(RangesStamped));
}

// ===== Config for all tasks =====
inline TaskConfig tasks[] = {
    {"Battery", Priority::SENSORS, Stack::SMALL, 10, batteryTask},
    {"Encoder", Priority::CONTROL, Stack::SMALL, 500, encoderTask},
    {"Imu", Priority::SENSORS, Stack::SMALL, 50, imuTask},
    {"LedIndicator", Priority::OBSERVING, Stack::XSMALL, 20, ledIndicatorTask},
    {"Monitor", Priority::OBSERVING, Stack::MEDIUM, 1, monitorTask},
    {"MotorControl", Priority::CONTROL, Stack::MEDIUM, 200, motorControlTask},
    {"Range", Priority::SENSORS, Stack::SMALL, 10, rangeTask},
    {"uRos", Priority::COMMUNICATION, Stack::MEDIUM, 200, uRosTask},
    {"uRosPing", Priority::OBSERVING, Stack::MEDIUM, 2, uRosPingTask},
};

// ===== Handles =====
inline TaskHandleWrapper taskHandles[sizeof(tasks) / sizeof(tasks[0])];

void createTasks() {
  for (size_t i = 0; i < sizeof(tasks) / sizeof(tasks[0]); i++) {
    taskHandles[i].create(tasks[i]);
  }
}

void destroyTasks() {
  for (size_t i = 0; i < sizeof(tasks) / sizeof(tasks[0]); i++) {
    taskHandles[i].destroy(tasks[i].name);
  }
}

// ===== Task functions =====
void batteryTask(void* p) {
  TickType_t period = taskGetPeriod(p);
  TickType_t wake_time = xTaskGetTickCount();
  BatteryStamped data = {};

  while (true) {
    data.timestamp_ns = rtos_get_timestamp_ns();
    g_battery->update();
    data.data = g_battery->getData();

    xQueueOverwrite(BatteryQueue, &data);
    vTaskDelayUntil(&wake_time, period);
  }
}

void encoderTask(void* p) {
  TickType_t period = taskGetPeriod(p);
  TickType_t wake_time = xTaskGetTickCount();

  while (true) {
    encoders.update();
    EncodersData data = encoders.getData();
    data.timestamp_ns = rtos_get_timestamp_ns();

    xQueueOverwrite(EncodersQueue, &data);
    vTaskDelayUntil(&wake_time, period);
  }
}

void imuTask(void* p) {
  TickType_t period = taskGetPeriod(p);
  TickType_t wake_time = xTaskGetTickCount();
  ImuStamped data = {};

  while (true) {
    data.timestamp_ns = rtos_get_timestamp_ns();
    g_imu->update();
    data.data = g_imu->getData();

    xQueueOverwrite(ImuQueue, &data);
    vTaskDelayUntil(&wake_time, period);
  }
}

void ledIndicatorTask(void* p) {
  TickType_t period = taskGetPeriod(p);
  TickType_t wake_time = xTaskGetTickCount();

  while (true) {
    bool battery_low = g_battery->isLow();
    bool uros_disconnected = (u_ros::state != u_ros::CONNECTED);
    bool error_state = false;

    ledIndicator.update(battery_low, uros_disconnected, error_state);
    vTaskDelayUntil(&wake_time, period);
  }
}

void monitorTask(void* p) {
  TickType_t period = taskGetPeriod(p);
  TickType_t wake_time = xTaskGetTickCount();
  char buf[1000];

  while (true) {
    if (g_firmware_log_level <= LOG_LEVEL_INFO) {
      vTaskGetRunTimeStats(buf);
      LOG_INFO("\r\n%s", buf);
    }

    vTaskDelayUntil(&wake_time, period);
  }
}

void motorControlTask(void* p) {
  TickType_t period = taskGetPeriod(p);
  TickType_t wake_time = xTaskGetTickCount();

  while (true) {
    motors.update();

    vTaskDelayUntil(&wake_time, period);
  }
}

void rangeTask(void* p) {
  TickType_t period = taskGetPeriod(p);
  TickType_t wake_time = xTaskGetTickCount();
  RangesStamped data = {};

  while (true) {
    data.timestamp_ns = rtos_get_timestamp_ns();
    g_ranges.update();
    data.data = g_ranges.getData();

    xQueueOverwrite(RangesQueue, &data);
    vTaskDelayUntil(&wake_time, period);
  }
}

void uRosTask(void* p) {
  TickType_t period = taskGetPeriod(p);
  TickType_t wake_time = xTaskGetTickCount();
  while (true) {
    u_ros::publishLoop();
    vTaskDelayUntil(&wake_time, period);
  }
}

void uRosPingTask(void* p) {
  TickType_t period = taskGetPeriod(p);
  TickType_t wake_time = xTaskGetTickCount();

  while (true) {
    u_ros::loop();
    vTaskDelayUntil(&wake_time, period);
  }
}

}  // namespace rtos
