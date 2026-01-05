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

namespace rtos::tasks {

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

void createAll();

}  // namespace rtos::tasks
