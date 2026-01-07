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

#include "battery.hpp"
#include "hardware/imu.hpp"
#include "log.hpp"
#include "motors.hpp"
#include "ranges.hpp"
#include "u_ros.hpp"

#define BATTERY_TASK_FREQ 10
#define BUTTON_TASK_FREQ 5
#define IMU_TASK_FREQ 50
#define RUNTIME_STATS_TASK_FREQ 1


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

// ================= BATTERY TASK ======================
namespace BatteryTask {
TaskHandle_t handle = nullptr;
void create() {
  auto result = xTaskCreate(task, "BatteryTask", configMINIMAL_STACK_SIZE + 500,
                            nullptr, 1, &handle);
  if (result != pdPASS) {
    LOG_ERROR("Battery task creation failed!");
  } else {
    LOG_INFO("Battery task started");
  }
}

void destroy() {
  if (handle != nullptr) {
    vTaskDelete(handle);
    handle = nullptr;
    LOG_INFO("Battery task stopped");
  }
}

void task(void* pvParameters) {
  UNUSED(pvParameters);
  TickType_t wake_time = xTaskGetTickCount();
  battery_data_t battery_data;

  while (1) {
    battery_data = battery::loop();
    xQueueOverwrite(rtos::BatteryQueue, &battery_data);
    vTaskDelayUntil(&wake_time, TASK_FREQ(BATTERY_TASK_FREQ));
  }
}

}  // namespace BatteryTask

// ================= BUTTONS TASK ======================
namespace ButtonsTask {
TaskHandle_t handle = nullptr;
void create() {
  auto result = xTaskCreate(task, "ButtonsTask", configMINIMAL_STACK_SIZE + 200,
                            nullptr, 1, &handle);
  if (result != pdPASS) {
    LOG_ERROR("Button task creation failed!");
  } else {
    LOG_INFO("Button task started");
  }
}

void destroy() {
  if (handle != nullptr) {
    vTaskDelete(handle);
    handle = nullptr;
    LOG_INFO("Button task stopped");
  }
}

void task(void* pvParameters) {
  UNUSED(pvParameters);
  TickType_t wake_time = xTaskGetTickCount();
  static uint8_t last_state = 0;

  while (1) {
    uint8_t buttons = 0;
    buttons |= (digitalRead(PUSH_BUTTON1) == LOW) << 0;
    buttons |= (digitalRead(PUSH_BUTTON2) == LOW) << 1;
    if (buttons != last_state) {
      last_state = buttons;
      xQueueOverwrite(rtos::ButtonsQueue, &buttons);
    }
    vTaskDelayUntil(&wake_time, TASK_FREQ(BUTTON_TASK_FREQ));
  }
}

}  // namespace ButtonsTask

// ================= IMU TASK ======================
namespace ImuTask {

TaskHandle_t handle = nullptr;

void create() {
  auto result = xTaskCreate(task, "ImuTask", configMINIMAL_STACK_SIZE + 750,
                            nullptr, 1, &handle);
  if (result != pdPASS) {
    LOG_ERROR("IMU task creation failed!");
  } else {
    LOG_INFO("IMU task started");
  }
}

void destroy() {
  if (handle != nullptr) {
    vTaskDelete(handle);
    handle = nullptr;
    LOG_INFO("IMU task stopped");
  }
}

void task(void* pvParameters) {
  UNUSED(pvParameters);
  TickType_t wake_time = xTaskGetTickCount();
  imu_data_t imu_data;

  while (1) {
    imu_data = imuDriver.loopHandler();
    xQueueOverwrite(rtos::ImuQueue, &imu_data);
    vTaskDelayUntil(&wake_time, TASK_FREQ(IMU_TASK_FREQ));
  }
}

}  // namespace ImuTask

// ================= PID TASK ======================
namespace PidTask {

TaskHandle_t handle = nullptr;

void create() {
  auto result = xTaskCreate(task, "PidTask", configMINIMAL_STACK_SIZE + 1000,
                            nullptr, 3, &handle);
  if (result != pdPASS) {
    LOG_ERROR("PID task creation failed!");
  } else {
    LOG_INFO("PID task started");
  }
}

void destroy() {
  if (handle != nullptr) {
    vTaskDelete(handle);
    handle = nullptr;
    LOG_INFO("PID task stopped");
  }
}

void task(void* pvParameters) {
  UNUSED(pvParameters);
  TickType_t wake_time = xTaskGetTickCount();
  TickType_t last_update_time = xTaskGetTickCount();
  float setpoint[4] = {0, 0, 0, 0};
  motor_joint_state_t motor_state;
  uint8_t freq_div_ptr = 0;

  while (1) {
    vTaskDelayUntil(&wake_time, TASK_FREQ(PID_FREQ));
    if (xQueueReceive(rtos::SetpointQueue, &setpoint, 0)) {
      last_update_time = xTaskGetTickCount();
    }

    // timeout check
    if (xTaskGetTickCount() - last_update_time > MOTORS_SETPOINT_TIMEOUT) {
      for (int i = 0; i < 4; i++) setpoint[i] = 0;
    }

    for (int i = 0; i < 4; i++) {
      wheel_motors[i].PidLoopHandler((float)setpoint[i]);
    }

    if (freq_div_ptr > (PID_FREQ / MOTORS_RESPONSE_FREQ)) {
      for (int i = 0; i < 4; i++) {
        motor_state.velocity[i] = wheel_motors[i].GetVelocity() / 1000.0;
        motor_state.position[i] =
            wheel_motors[i].GetWheelAbsPosition() / 1000.0;
      }
      xQueueOverwrite(rtos::MotorStateQueue, &motor_state);
      freq_div_ptr = 0;
    }
    freq_div_ptr++;
  }
}

}  // namespace PidTask

// ================= RANGE TASK ======================
namespace RangeTask {

TaskHandle_t handle = nullptr;

void create() {
  auto result = xTaskCreate(task, "RangeTask", configMINIMAL_STACK_SIZE + 700,
                            nullptr, 1, &handle);
  if (result != pdPASS) {
    LOG_ERROR("Range task creation failed!");
  } else {
    LOG_INFO("Range task started");
  }
}

void destroy() {
  if (handle != nullptr) {
    vTaskDelete(handle);
    handle = nullptr;
    LOG_INFO("Range task stopped");
  }
}

void task(void* pvParameters) {
  UNUSED(pvParameters);
  TickType_t wake_time = xTaskGetTickCount();
  ranges_data_t ranges_data;

  while (1) {
    rangeSensorsManager.readAll();
    for (size_t i = 0; i < rangeSensorsManager.count(); i++) {
        VL53L0XSensor& s = rangeSensorsManager.getSensor(i);
        ranges_data.range[i] = s.timeout ? NAN : s.lastRange / 1000.0;
    }

    xQueueOverwrite(rtos::RangeQueue, &ranges_data);
    vTaskDelayUntil(&wake_time, TASK_FREQ(10));
  }
}
}  // namespace RangeTask

// ================= RUNTIME STATS TASK ======================
namespace RuntimeStatsTask {

TaskHandle_t handle = nullptr;

void create() {
  auto result =
      xTaskCreate(task, "RuntimeStatsTask", configMINIMAL_STACK_SIZE + 500,
                  nullptr, 1, &handle);
  if (result != pdPASS) {
    LOG_ERROR("RuntimeStatsTask creation failed!");
  } else {
    LOG_INFO("RuntimeStatsTask started");
  }
}

void destroy() {
  if (handle != nullptr) {
    vTaskDelete(handle);
    handle = nullptr;
    LOG_INFO("RuntimeStatsTask stopped");
  }
}

void task(void* pvParameters) {
  UNUSED(pvParameters);
  char buf[2000];

  while (1) {
    if (firmware_log_level <= LOG_LEVEL_INFO) {
      vTaskGetRunTimeStats(buf);
      LOG_INFO("\r\n%s", buf);
    }

    vTaskDelay(TASK_FREQ(RUNTIME_STATS_TASK_FREQ));
  }
}

}  // namespace RuntimeStatsTask

// ================= uROS TASK ======================
namespace uRosTask {

TaskHandle_t handle = nullptr;

void create() {
  auto result = xTaskCreate(task, "uRosTask", configMINIMAL_STACK_SIZE + 2500,
                            nullptr, 2, &handle);
  if (result != pdPASS) {
    LOG_ERROR("uRosTask creation failed!");
  } else {
    LOG_INFO("uRosTask started");
  }
}

void destroy() {
  if (handle != nullptr) {
    vTaskDelete(handle);
    handle = nullptr;
    LOG_INFO("uRosTask stopped");
  }
}

void task(void* pvParameters) {
  UNUSED(pvParameters);

  while (1) {
    u_ros::loop();
    // vTaskDelayUntil(&wake_time, uROS_SPIN_DELAY_MS);
  }
}

}  // namespace uRosTask

void createTasks() {
  BatteryTask::create();
  ButtonsTask::create();
  ImuTask::create();
  // PidTask::create();
  RangeTask::create();
  RuntimeStatsTask::create();
  uRosTask::create();
}

}  // namespace rtos
