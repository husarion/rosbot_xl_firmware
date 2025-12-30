#include "rtos/tasks.hpp"

#include <STM32FreeRTOS.h>

#include "hardware/imu.hpp"
#include "log.hpp"
#include "micro_ros_cfg.hpp"
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
    LOG_DEBUG(
        "Orient 10x: [%d, %d, %d, %d] \n Gyro 10x: [%d, %d, %d] rad/s\n Accel "
        "10x: [%d, %d, %d] "
        "m/s^2",
        (int)(imu_data.orientation[0] * 10),
        (int)(imu_data.orientation[1] * 10),
        (int)(imu_data.orientation[2] * 10),
        (int)(imu_data.orientation[3] * 10),
        (int)(imu_data.angular_velocity[0] * 10),
        (int)(imu_data.angular_velocity[1] * 10),
        (int)(imu_data.angular_velocity[2] * 10),
        (int)(imu_data.acceleration[0] * 10),
        (int)(imu_data.acceleration[1] * 10),
        (int)(imu_data.acceleration[2] * 10));

    xQueueSendToFront(rtos::queues::ImuQueue, &imu_data, 0);

    UBaseType_t stack_free = uxTaskGetStackHighWaterMark(nullptr);
    if (stack_free < 50) {
      LOG_WARN("Warning: IMU task stack low: %d words remaining", stack_free);
    }

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
  motor_state_queue_t motor_state;
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

// ================= RCLC SPIN TASK ======================
namespace RclcSpinTask {

TaskHandle_t handle = nullptr;

void create() {
  auto result =
      xTaskCreate(task, "RclcSpinTask", configMINIMAL_STACK_SIZE + 2500,
                  nullptr, tskIDLE_PRIORITY + 1, &handle);
  if (result != pdPASS) {
    LOG_ERROR("RclcSpinTask creation failed!");
  } else {
    LOG_INFO("RclcSpinTask started");
  }
}

void destroy() {
  if (handle != nullptr) {
    vTaskDelete(handle);
    handle = nullptr;
    LOG_INFO("RclcSpinTask stopped");
  }
}

void task(void* pvParameters) {
  UNUSED(pvParameters);
  TickType_t wake_time = xTaskGetTickCount();
  uRosFunctionStatus status;

  while (1) {
    xQueueReceive(rtos::queues::uRosPingAgentStatusQueue, &status, 0);
    uRosLoopHandler(status);
    vTaskDelayUntil(&wake_time, 1);
  }
}

}  // namespace RclcSpinTask

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
    if (firmware_log_level >= LOG_LEVEL_DEBUG) {
      vTaskGetRunTimeStats(buf);
      LOG_DEBUG("\r\n-------------\r\n%s", buf);
    }
    vTaskDelay(100);
  }
}

}  // namespace RuntimeStatsTask

// ================= UROS PING TASK ======================
namespace URosPingTask {

TaskHandle_t handle = nullptr;

void create() {
  auto result =
      xTaskCreate(task, "URosPingTask", configMINIMAL_STACK_SIZE + 500, nullptr,
                  tskIDLE_PRIORITY + 1, &handle);
  if (result != pdPASS) {
    LOG_ERROR("URosPingTask creation failed!");
  } else {
    LOG_INFO("URosPingTask started");
  }
}

void destroy() {
  if (handle != nullptr) {
    vTaskDelete(handle);
    handle = nullptr;
    LOG_INFO("URosPingTask stopped");
  }
}

void task(void* pvParameters) {
  UNUSED(pvParameters);
  uRosFunctionStatus status;

  while (1) {
    status = uRosPingAgent(PING_AGENT_TIMEOUT, PING_AGENT_ATTEMPTS);
    xQueueSendToFront(rtos::queues::uRosPingAgentStatusQueue, &status, 0);

    switch (status) {
      case Ok:
        SetGreenLed(Toggle);
        SetRedLed(Off);
        break;
      case Error:
        LOG_ERROR("rmw_uros_ping_agent() error!");
        SetGreenLed(Off);
        SetRedLed(On);
        break;
      case Default:
        SetGreenLed(On);
        SetRedLed(On);
        break;
      default:
        SetGreenLed(Off);
        SetRedLed(Off);
        break;
    }

    vTaskDelay(FREQ_TO_TICKS(PING_AGENT_FREQUENCY));
  }
}

}  // namespace URosPingTask

void createAll() {
  ImuTask::create();
  // PidTask::create();
  // RclcSpinTask::create();
  // RuntimeStatsTask::create();
  // URosPingTask::create();
}

}  // namespace rtos::tasks
