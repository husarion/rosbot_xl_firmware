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
#include <micro_ros_arduino.h>

static inline bool rtos_get_timestamp_ns(int64_t& timestamp_ns) {
  if (rmw_uros_epoch_synchronized()) {
    timestamp_ns = rmw_uros_epoch_nanos();
    return true;
  }
  return false;
}

void createQueues();
void createTasks();

// Priority levels
// 7 - Highest (configMAX_PRIORITIES)
// 0 - Idle (tskIDLE_PRIORITY)
enum Priority : UBaseType_t {
  OBSERVING = 1,
  SENSORS = 2,
  COMMUNICATION = 3,
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

inline TickType_t frequencyToTicks(float freq) {
  return freq == 0 ? 0 : (TickType_t)(configTICK_RATE_HZ / freq);
}

static inline uint16_t taskGetFreq(void* params) {
  return static_cast<uint16_t>(reinterpret_cast<uintptr_t>(params));
}

static inline TickType_t taskGetPeriod(void* params) {
  return frequencyToTicks(taskGetFreq(params));
}

struct TaskHandleWrapper {
  TaskHandle_t handle = nullptr;

  void create(const TaskConfig& cfg) {
    void* freq_param =
        reinterpret_cast<void*>(static_cast<uintptr_t>(cfg.frequency));
    auto result = xTaskCreate(cfg.function, cfg.name,
                              configMINIMAL_STACK_SIZE + cfg.stack, freq_param,
                              cfg.priority, &handle);
  }

  void destroy() {
    if (handle != nullptr) {
      vTaskDelete(handle);
      handle = nullptr;
    }
  }
};
