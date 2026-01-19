#pragma once

#include <STM32FreeRTOS.h>

#define RTOS_FREQUENCY 1000  // hz
#define FREQ_TO_TIME(freq) \
  (TickType_t)(RTOS_FREQUENCY / freq * portTICK_PERIOD_MS)
#define TASK_FREQ(freq) (TickType_t)(configTICK_RATE_HZ / freq)

// PRIORITY LEVELS 
// 7 - Highest (configMAX_PRIORITIES)
// 0 - Idle (tskIDLE_PRIORITY)
namespace TaskPriorities {
    constexpr UBaseType_t SAFETY = 4;
    constexpr UBaseType_t CONTROL = 3;
    constexpr UBaseType_t COMMUNICATION = 2;
    constexpr UBaseType_t SENSORS = 1;
    constexpr UBaseType_t STATS = 0;
}

namespace TaskSizes {
    constexpr uint32_t MINIMAL     = configMINIMAL_STACK_SIZE;
    constexpr uint32_t SMALL       = configMINIMAL_STACK_SIZE + 256;
    constexpr uint32_t MEDIUM      = configMINIMAL_STACK_SIZE + 512;
    constexpr uint32_t LARGE       = configMINIMAL_STACK_SIZE + 1024;
    constexpr uint32_t XLARGE      = configMINIMAL_STACK_SIZE + 2048;
}

namespace TaskFreq {
    constexpr uint32_t BATTERY    = 1;
    constexpr uint32_t BUTTONS    = 10;
    constexpr uint32_t IMU        = 50;
    constexpr uint32_t RANGE      = 20;
    constexpr uint32_t RUNTIME_STATS = 0.2;
    constexpr uint32_t UROS       = 100;
    constexpr uint32_t WHEEL_CTRL    = 100;
}