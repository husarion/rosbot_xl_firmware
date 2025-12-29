#pragma once
#include <STM32FreeRTOS.h>

namespace rtos::queues {

extern QueueHandle_t SetpointQueue;
extern QueueHandle_t MotorStateQueue;
extern QueueHandle_t ImuQueue;
extern QueueHandle_t RangeQueue;
extern QueueHandle_t BatteryStateQueue;
extern QueueHandle_t uRosPingAgentStatusQueue;

void createAll();

}