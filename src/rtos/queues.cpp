
#include "rtos/queues.hpp"

#include <STM32FreeRTOS.h>

#include "battery_types.h"
#include "ImuLib_cfg.h"
#include "log.hpp"
#include "micro_ros_cfg.h"
#include "motors.h"
#include "rtos/queues.hpp"

namespace rtos::queues {

QueueHandle_t SetpointQueue;
QueueHandle_t MotorStateQueue;
QueueHandle_t ImuQueue;
QueueHandle_t RangeQueue;
QueueHandle_t BatteryStateQueue;
QueueHandle_t uRosAgentConectionQueue;

void createAll() {
  SetpointQueue = xQueueCreate(1, sizeof(double) * 4);
  MotorStateQueue = xQueueCreate(1, sizeof(motor_joint_state_t));
  ImuQueue = xQueueCreate(1, sizeof(imu_data_t));
  // RangeQueue = xQueueCreate(1, sizeof(ranges_queue_t));
  BatteryStateQueue = xQueueCreate(1, sizeof(battery_state_t));
  uRosAgentConectionQueue = xQueueCreate(1, sizeof(bool));
}

}  // namespace rtos::queues
