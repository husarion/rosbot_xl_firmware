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

#include <STM32FreeRTOS.h>

#include "log.hpp"

namespace rtos {

inline QueueHandle_t ImuQueue;
inline QueueHandle_t RangeQueue;

void createQueues();
void createTasks();
void destroyTasks();

// Priority levels
// 7 - Highest (configMAX_PRIORITIES)
// 0 - Idle (tskIDLE_PRIORITY)
enum Priority : UBaseType_t {
  STATS = 1,
  COMMUNICATION = 2,
  SENSORS = 3,
  CONTROL = 4,
  SAFETY = 5
};

enum Stack : uint16_t {
  MINIMAL = 0,
  XSMALL = 128,
  SMALL = 256,
  MEDIUM = 512,
  LARGE = 1024,
  XLARGE = 2048
};

struct TaskConfig {
  const char* name;
  Priority priority;
  Stack stack;
  float frequency;
  void (*function)(void*);
};

struct TaskHandleWrapper {
  TaskHandle_t handle = nullptr;

  void create(const TaskConfig& cfg) {
    auto result = xTaskCreate(cfg.function, cfg.name,
                              configMINIMAL_STACK_SIZE + cfg.stack, nullptr,
                              cfg.priority, &handle);
  }

  void destroy(const char* name) {
    if (handle != nullptr) {
      vTaskDelete(handle);
      handle = nullptr;
    }
  }
};

inline TickType_t frequencyToTicks(float freq) {
  return freq == 0 ? 0 : (TickType_t)(configTICK_RATE_HZ / freq);
}

void batteryTask(void* pvParameters);
void encoderTask(void* pvParameters);
void imuTask(void* pvParameters);
void ledIndicatorTask(void* pvParameters);
void monitorTask(void* pvParameters);
void motorControlTask(void* pvParameters);
void rangeTask(void* pvParameters);
void uRosTask(void* pvParameters);

}  // namespace rtos
