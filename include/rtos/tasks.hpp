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

namespace uRosPingTask {
extern TaskHandle_t handle;
void create();
void destroy();
void task(void* pvParameters);
}  // namespace uRosPingTask

namespace uRosSpinTask {
extern TaskHandle_t handle;
void create();
void destroy();
void task(void* pvParameters);
}  // namespace uRosSpinTask

void createAll();

}  // namespace rtos::tasks
