#include <STM32FreeRTOS.h>

#include "hardware/imu.h"
#include "rtos/queues.h"

namespace rtos::queues {

QueueHandle_t SetpointQueue;
QueueHandle_t MotorStateQueue;
QueueHandle_t ImuQueue;
QueueHandle_t RangeQueue;
QueueHandle_t BatteryStateQueue;
QueueHandle_t uRosPingAgentStatusQueue;

void createAll() {
    // SetpointQueue = xQueueCreate(1, sizeof(double) * 4);
    // MotorStateQueue = xQueueCreate(1, sizeof(motor_state_queue_t));
    ImuQueue = xQueueCreate(1, sizeof(imu_data_t));
    // RangeQueue = xQueueCreate(1, sizeof(ranges_queue_t));
    // BatteryStateQueue = xQueueCreate(1, sizeof(battery_state_queue_t));
    // uRosPingAgentStatusQueue = xQueueCreate(1, sizeof(uRosFunctionStatus));
}

} // namespace rtos::queues
