#include "rtos/tasks.hpp"

#include <STM32FreeRTOS.h>
#include <micro_ros_arduino.h>

#include "imu.hpp"
#include "log.h"
#include "micro_ros_cfg.h"
#include "motors.hpp"
#include "rtos/queues.hpp"

namespace rtos::tasks {

// ================= IMU TASK ======================
namespace ImuTask {

TaskHandle_t handle = nullptr;

void create() {
  auto result = xTaskCreate(task, "ImuTask", configMINIMAL_STACK_SIZE + 750,
                            nullptr, tskIDLE_PRIORITY + 1, &handle);
  if (result != pdPASS) {
    LOG_ERROR("IMU task creation failed!");
  } else {
    LOG_INFO("IMU task started");
  }
}

void destroy() {
  if (handle != nullptr) {
    vTaskDelete(handle);
    handle = nullptr;
    LOG_INFO("IMU task stopped");
  }
}

void task(void* pvParameters) {
  UNUSED(pvParameters);
  TickType_t wake_time = xTaskGetTickCount();
  imu_data_t imu_data;

  while (1) {
    imu_data = imuDriver.loopHandler();
    xQueueSendToFront(rtos::queues::ImuQueue, &imu_data, 0);
    vTaskDelayUntil(&wake_time, FREQ_TO_TICKS(IMU_SAMPLE_FREQ));
  }
}

}  // namespace ImuTask

// ================= PID TASK ======================
namespace PidTask {

TaskHandle_t handle = nullptr;

void create() {
  auto result = xTaskCreate(task, "PidTask", configMINIMAL_STACK_SIZE + 1000,
                            nullptr, tskIDLE_PRIORITY + 3, &handle);
  if (result != pdPASS) {
    LOG_ERROR("PID task creation failed!");
  } else {
    LOG_INFO("PID task started");
  }
}

void destroy() {
  if (handle != nullptr) {
    vTaskDelete(handle);
    handle = nullptr;
    LOG_INFO("PID task stopped");
  }
}

void task(void* pvParameters) {
  UNUSED(pvParameters);
  TickType_t wake_time = xTaskGetTickCount();
  TickType_t last_update_time = xTaskGetTickCount();
  double setpoint[4] = {0, 0, 0, 0};
  motor_joint_state_t motor_state;
  uint8_t freq_div_ptr = 0;

  while (1) {
    vTaskDelayUntil(&wake_time, FREQ_TO_TICKS(PID_FREQ));
    if (xQueueReceive(rtos::queues::SetpointQueue, &setpoint, 0)) {
      last_update_time = xTaskGetTickCount();
    }

    // timeout check
    if (xTaskGetTickCount() - last_update_time > MOTORS_SETPOINT_TIMEOUT) {
      for (int i = 0; i < 4; i++) setpoint[i] = 0;
    }

    for (int i = 0; i < 4; i++) {
      wheel_motors[i].PidLoopHandler((float)setpoint[i]);
    }

    if (freq_div_ptr > (PID_FREQ / MOTORS_RESPONSE_FREQ)) {
      for (int i = 0; i < 4; i++) {
        motor_state.velocity[i] = wheel_motors[i].GetVelocity() / 1000.0;
        motor_state.position[i] =
            wheel_motors[i].GetWheelAbsPosition() / 1000.0;
      }
      xQueueSendToFront(rtos::queues::MotorStateQueue, &motor_state, 0);
      freq_div_ptr = 0;
    }
    freq_div_ptr++;
  }
}

}  // namespace PidTask

// ================= RUNTIME STATS TASK ======================
namespace RuntimeStatsTask {

TaskHandle_t handle = nullptr;

void create() {
  auto result =
      xTaskCreate(task, "RuntimeStatsTask", configMINIMAL_STACK_SIZE + 500,
                  nullptr, tskIDLE_PRIORITY + 1, &handle);
  if (result != pdPASS) {
    LOG_ERROR("RuntimeStatsTask creation failed!");
  } else {
    LOG_INFO("RuntimeStatsTask started");
  }
}

void destroy() {
  if (handle != nullptr) {
    vTaskDelete(handle);
    handle = nullptr;
    LOG_INFO("RuntimeStatsTask stopped");
  }
}

void task(void* pvParameters) {
  UNUSED(pvParameters);
  char buf[2000];

  while (1) {
    if (firmware_log_level >= LOG_LEVEL_INFO) {
      vTaskGetRunTimeStats(buf);
      LOG_INFO("\r\n-------------\r\n%s", buf);
    }

    vTaskDelay(1000);
  }
}

}  // namespace RuntimeStatsTask

// ================= UROS PING TASK ======================
namespace uRosPingTask {

TaskHandle_t handle = nullptr;

void create() {
  auto result =
      xTaskCreate(task, "uRosPingTask", configMINIMAL_STACK_SIZE + 500, nullptr,
                  tskIDLE_PRIORITY + 1, &handle);
  if (result != pdPASS) {
    LOG_ERROR("uRosPingTask creation failed!");
  } else {
    LOG_INFO("uRosPingTask started");
  }
}

void destroy() {
  if (handle != nullptr) {
    vTaskDelete(handle);
    handle = nullptr;
    LOG_INFO("uRosPingTask stopped");
  }
}

void task(void* pvParameters) {
  UNUSED(pvParameters);
  uRosFunctionStatus connected;

  while (1) {
    connected = uRosPingAgent();
    xQueueSendToFront(rtos::queues::uRosAgentConectionQueue, &connected, 0);

    if (connected) {
      SetGreenLed(On);
    } else {
      SetGreenLed(Off);
    }
    vTaskDelay(FREQ_TO_TICKS(uROS_PING_FREQUENCY));
  }
}

}  // namespace uRosPingTask

// ================= uROS SPIN TASK ======================
namespace uRosSpinTask {

TaskHandle_t handle = nullptr;

void create() {
  auto result =
      xTaskCreate(task, "uRosSpinTask", configMINIMAL_STACK_SIZE + 2500,
                  nullptr, tskIDLE_PRIORITY + 1, &handle);
  if (result != pdPASS) {
    LOG_ERROR("uRosSpinTask creation failed!");
  } else {
    LOG_INFO("uRosSpinTask started");
  }
}

void destroy() {
  if (handle != nullptr) {
    vTaskDelete(handle);
    handle = nullptr;
    LOG_INFO("uRosSpinTask stopped");
  }
}

void task(void* pvParameters) {
  UNUSED(pvParameters);
  TickType_t wake_time = xTaskGetTickCount();
  uRosFunctionStatus connected;

  while (1) {
    xQueueReceive(rtos::queues::uRosAgentConectionQueue, &connected, 0);
    uRosLoopHandler(connected);
    vTaskDelayUntil(&wake_time, uROS_SPIN_DELAY_MS);
  }
}

}  // namespace uRosSpinTask

void createAll() {
  ImuTask::create();
  // PidTask::create();
  RuntimeStatsTask::create();
  uRosPingTask::create();
  uRosSpinTask::create();
}

}  // namespace rtos::tasks
