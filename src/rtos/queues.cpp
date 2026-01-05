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

#include "rtos/queues.hpp"

#include <STM32FreeRTOS.h>

#include "battery.hpp"
#include "hardware/imu.hpp"
#include "motors.hpp"
#include "ranges.hpp"
#include "u_ros.hpp"

namespace rtos::queues {

QueueHandle_t SetpointQueue;
QueueHandle_t MotorStateQueue;
QueueHandle_t ImuQueue;
QueueHandle_t RangeQueue;
QueueHandle_t BatteryQueue;
QueueHandle_t uRosAgentConectionQueue;

void createAll() {
  SetpointQueue = xQueueCreate(1, sizeof(double) * 4);
  MotorStateQueue = xQueueCreate(1, sizeof(motor_joint_state_t));
  ImuQueue = xQueueCreate(1, sizeof(imu_data_t));
  RangeQueue = xQueueCreate(1, sizeof(ranges_queue_t));
  BatteryQueue = xQueueCreate(1, sizeof(battery_data_t));
  uRosAgentConectionQueue = xQueueCreate(1, sizeof(bool));
}

}  // namespace rtos::queues
