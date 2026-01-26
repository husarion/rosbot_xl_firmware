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

#include "rosbot/tasks.hpp"

#include <STM32FreeRTOS.h>

#include "battery.hpp"
#include "encoder.hpp"
#include "hardware/imu.hpp"
#include "log.hpp"
#include "motors.hpp"
#include "ranges.hpp"
#include "u_ros.hpp"
#include "motor_driver.hpp"


#define BATTERY_TASK_FREQ 10
#define BUTTON_TASK_FREQ 5
#define ENCODER_TASK_FREQ 250
#define IMU_TASK_FREQ 50
#define MONITOR_TASK_FREQ 1
#define MOTOR_CONTROL_TASK_FREQ 100
#define RANGE_TASK_FREQ 10
#define UROS_TASK_FREQ 0  // Run as fast as possible

namespace rtos {

QueueHandle_t BatteryQueue;
QueueHandle_t ButtonsQueue;
QueueHandle_t ImuQueue;
QueueHandle_t MotorStateQueue;
QueueHandle_t RangeQueue;
QueueHandle_t SetpointQueue;

void createQueues() {
  BatteryQueue = xQueueCreate(1, sizeof(battery_data_t));
  ButtonsQueue = xQueueCreate(1, sizeof(uint8_t));
  ImuQueue = xQueueCreate(1, sizeof(imu_data_t));
  MotorStateQueue = xQueueCreate(1, sizeof(motor_joint_state_t));
  RangeQueue = xQueueCreate(1, sizeof(ranges_data_t));
  SetpointQueue = xQueueCreate(1, sizeof(float) * 4);
}

// ===== Config for all tasks =====
inline TaskConfig tasks[] = {
    {"Battery", Priority::SENSORS, Stack::SMALL, BATTERY_TASK_FREQ, batteryTask},
    {"Buttons", Priority::SENSORS, Stack::XSMALL, BUTTON_TASK_FREQ, buttonsTask},
    {"Encoder", Priority::CONTROL, Stack::SMALL, ENCODER_TASK_FREQ, encoderTask},
    {"Imu",     Priority::SENSORS, Stack::SMALL, IMU_TASK_FREQ, imuTask},
    {"Monitor", Priority::STATS,   Stack::XLARGE, MONITOR_TASK_FREQ, monitorTask},
    {"MotorControl", Priority::CONTROL, Stack::MEDIUM, MOTOR_CONTROL_TASK_FREQ, motorControlTask},
    {"Range",   Priority::SENSORS, Stack::SMALL, RANGE_TASK_FREQ, rangeTask},
    {"uRos",  Priority::COMMUNICATION, Stack::XLARGE, UROS_TASK_FREQ, uRosTask},
};

// ===== Handles =====
inline TaskHandleWrapper taskHandles[sizeof(tasks)/sizeof(tasks[0])];

void createTasks() {
    for (size_t i = 0; i < sizeof(tasks)/sizeof(tasks[0]); i++) {
        taskHandles[i].create(tasks[i]);
    }
}

void destroyTasks() {
    for (size_t i = 0; i < sizeof(tasks)/sizeof(tasks[0]); i++) {
        taskHandles[i].destroy(tasks[i].name);
    }
}

// ===== Task functions =====
void batteryTask(void* pvParameters) {
    UNUSED(pvParameters);
    TickType_t wake_time = xTaskGetTickCount();
    battery_data_t data;

    while (true) {
        data = battery::loop();
        xQueueOverwrite(BatteryQueue, &data);
        vTaskDelayUntil(&wake_time, frequencyToTicks(BATTERY_TASK_FREQ));
    }
}

void buttonsTask(void* pvParameters) {
    UNUSED(pvParameters);
    TickType_t wake_time = xTaskGetTickCount();
    uint8_t last_state = 0;

    while (true) {
        uint8_t buttons = 0;
        buttons |= (digitalRead(PUSH_BUTTON1) == LOW) << 0;
        buttons |= (digitalRead(PUSH_BUTTON2) == LOW) << 1;
        if (buttons != last_state) {
            last_state = buttons;
            xQueueOverwrite(ButtonsQueue, &buttons);
        }
        vTaskDelayUntil(&wake_time, frequencyToTicks(BUTTON_TASK_FREQ));
    }
}

void encoderTask(void* pvParameters) {
  UNUSED(pvParameters);
  TickType_t wake_time = xTaskGetTickCount();

  while (true) {
    encoderManager.updateAll();
    vTaskDelayUntil(&wake_time, frequencyToTicks(ENCODER_TASK_FREQ));
  }
}

void imuTask(void* pvParameters) {
  UNUSED(pvParameters);
  TickType_t wake_time = xTaskGetTickCount();
  imu_data_t imu_data;

  while (true) {
    imu_data = imuDriver.loopHandler();
    xQueueOverwrite(rtos::ImuQueue, &imu_data);
    vTaskDelayUntil(&wake_time, frequencyToTicks(IMU_TASK_FREQ));
  }
}

void monitorTask(void* pvParameters) {
  UNUSED(pvParameters);
  char buf[1000];

  while (true) {
    if (firmware_log_level <= LOG_LEVEL_INFO) {
      vTaskGetRunTimeStats(buf);
      LOG_INFO("\r\n%s", buf);
    }

    vTaskDelay(frequencyToTicks(MONITOR_TASK_FREQ));
  }
}

void motorControlTask(void* pvParameters) {
    UNUSED(pvParameters);
    TickType_t wake_time = xTaskGetTickCount();

    while (true) {
        Motors.update();

        vTaskDelayUntil(&wake_time, frequencyToTicks(MOTOR_CONTROL_TASK_FREQ));
    }
}

void rangeTask(void* pvParameters) {
    UNUSED(pvParameters);
    TickType_t wake_time = xTaskGetTickCount();
    ranges_data_t ranges_data;

    while (true) {
        rangeSensorsManager.readAll();
        for (size_t i = 0; i < rangeSensorsManager.count(); i++) {
            VL53L0XSensor& s = rangeSensorsManager.getSensor(i);
            ranges_data.range[i] = s.timeout ? NAN : s.lastRange / 1000.0;
        }

        xQueueOverwrite(RangeQueue, &ranges_data);
        vTaskDelayUntil(&wake_time, frequencyToTicks(RANGE_TASK_FREQ));
    }
}

void uRosTask(void* pvParameters) {
    UNUSED(pvParameters);
    while (true) {
        u_ros::loop();
    }
}

}  // namespace rtos
