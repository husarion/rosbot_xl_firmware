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

namespace RclcSpinTask {
extern TaskHandle_t handle;
void create();
void destroy();
void task(void* pvParameters);
}  // namespace RclcSpinTask

namespace RuntimeStatsTask {
extern TaskHandle_t handle;
void create();
void destroy();
void task(void* pvParameters);
}  // namespace RuntimeStatsTask

namespace URosPingTask {
extern TaskHandle_t handle;
void create();
void destroy();
void task(void* pvParameters);
}  // namespace URosPingTask

void createAll();

}  // namespace rtos::tasks
