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

#define RTOS_FREQUENCY 1000  // hz
#define FREQ_TO_TIME(freq) \
  (TickType_t)(RTOS_FREQUENCY / freq * portTICK_PERIOD_MS)
#define FREQ_TO_TICKS(freq) (TickType_t)(configTICK_RATE_HZ / freq)

namespace rtos {

extern QueueHandle_t BatteryQueue;
extern QueueHandle_t ButtonsQueue;
extern QueueHandle_t ImuQueue;
extern QueueHandle_t MotorStateQueue;
extern QueueHandle_t RangeQueue;
extern QueueHandle_t SetpointQueue;
extern QueueHandle_t uRosAgentConectionQueue;

namespace BatteryTask {
extern TaskHandle_t handle;
void create();
void destroy();
void task(void* pvParameters);
}  // namespace BatteryTask

namespace ButtonsTask {
extern TaskHandle_t handle;
void create();
void destroy();
void task(void* pvParameters);
}  // namespace ButtonsTask

namespace ImuTask {
extern TaskHandle_t handle;
void create();
void destroy();
void task(void* pvParameters);
}  // namespace ImuTask

namespace PidTask {
extern TaskHandle_t handle;
void create();
void destroy();
void task(void* pvParameters);
}  // namespace PidTask

namespace RuntimeStatsTask {
extern TaskHandle_t handle;
void create();
void destroy();
void task(void* pvParameters);
}  // namespace RuntimeStatsTask

namespace uRosTask {
extern TaskHandle_t handle;
void create();
void destroy();
void task(void* pvParameters);
}  // namespace uRosTask

void createQueues();
void createTasks();

}  // namespace rtos
