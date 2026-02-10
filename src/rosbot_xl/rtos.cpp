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

#include "rtos.hpp"

#include <STM32FreeRTOS.h>
#include <micro_ros_arduino.h>

#include "battery.hpp"
#include "control/encoders_manager.hpp"
#include "control/motors_manager.hpp"
#include "led_indicator.hpp"
#include "log.hpp"
#include "motors.hpp"
#include "sensors/imu.hpp"
#include "sensors/ranges.hpp"
#include "u_ros.hpp"

#define BATTERY_TASK_FREQ 10
#define BUTTON_TASK_FREQ 5
#define ENCODER_TASK_FREQ 500
#define FAN_TASK_FREQ 10
#define IMU_TASK_FREQ 50
#define LED_INDICATOR_TASK_FREQ 20
#define LED_STRIP_TASK_FREQ 0.5
#define MONITOR_TASK_FREQ 1
#define MOTOR_CONTROL_TASK_FREQ 200
#define SHUTDOWN_TASK_FREQ 5
#define UROS_TASK_FREQ 0  // Run as fast as possible

namespace rtos {

void createQueues() {
  BatteryQueue = xQueueCreate(1, sizeof(BatteryData));
  EncodersQueue = xQueueCreate(1, sizeof(EncodersData));
  ImuQueue = xQueueCreate(1, sizeof(ImuData));
}

// ===== Config for all tasks =====
inline TaskConfig tasks[] = {
    {"Battery", Priority::SENSORS, Stack::SMALL, BATTERY_TASK_FREQ,
     batteryTask}, // PowerBoard is responsible for battery data
    {"Encoder", Priority::CONTROL, Stack::SMALL, ENCODER_TASK_FREQ,
     encoderTask},
    {"Fan", Priority::SENSORS, Stack::SMALL, FAN_TASK_FREQ, fanTask},
    {"Imu", Priority::SENSORS, Stack::SMALL, IMU_TASK_FREQ, imuTask},
    {"LedIndicator", Priority::STATS, Stack::XSMALL, LED_INDICATOR_TASK_FREQ,
     ledIndicatorTask},
    {"LedStrip", Priority::STATS, Stack::SMALL, LED_STRIP_TASK_FREQ, ledStripTask},
    {"Monitor", Priority::STATS, Stack::MEDIUM, MONITOR_TASK_FREQ, monitorTask},
    {"MotorControl", Priority::CONTROL, Stack::MEDIUM, MOTOR_CONTROL_TASK_FREQ,
     motorControlTask},
    {"Shutdown", Priority::COMMUNICATION, Stack::SMALL, SHUTDOWN_TASK_FREQ, shutdownTask},
    {"uRos", Priority::COMMUNICATION, Stack::XLARGE, UROS_TASK_FREQ, uRosTask},
};

// ===== Handles =====
inline TaskHandleWrapper taskHandles[sizeof(tasks) / sizeof(tasks[0])];

void createTasks() {
  for (size_t i = 0; i < sizeof(tasks) / sizeof(tasks[0]); i++) {
    taskHandles[i].create(tasks[i]);
  }
}

void destroyTasks() {
  for (size_t i = 0; i < sizeof(tasks) / sizeof(tasks[0]); i++) {
    taskHandles[i].destroy(tasks[i].name);
  }
}

// ===== Task functions =====
void batteryTask(void* p) {
  UNUSED(p);
  TickType_t wake_time = xTaskGetTickCount();

  uint16_t TimeDivider = 0;
  while (true) {
    if (PowerBoardFirmwareVersion.length() == 0 ||
        PowerBoardVersion.length() == 0) {
      PbInfoRequest();
    }
    TimeDivider++;
    if (TimeDivider % 5 != 0) BatteryInfoRequest();
    PowerBoardSerial.UartProtocolLoopHandler();
    vTaskDelay(150);
  }
}

void encoderTask(void* p) {
  UNUSED(p);
  TickType_t wake_time = xTaskGetTickCount();

  while (true) {
    int64_t timestamp_ns = 0;
    if (rmw_uros_epoch_synchronized()) {
      timestamp_ns = rmw_uros_epoch_nanos();
    }

    encoders.update();
    EncodersData data = encoders.getData();
    data.timestamp_ns = timestamp_ns;

    xQueueOverwrite(rtos::EncodersQueue, &data);
    vTaskDelayUntil(&wake_time, frequencyToTicks(ENCODER_TASK_FREQ));
  }
}

void fanTask(void* p) {
  UNUSED(p);
  FanHardwareInit();
  while (true) {
    FanLoopHanlder();
    vTaskDelay(100);
  }
}

void imuTask(void* p) {
  UNUSED(p);
  TickType_t wake_time = xTaskGetTickCount();

  while (true) {
    int64_t timestamp_ns = 0;
    if (rmw_uros_epoch_synchronized()) {
      timestamp_ns = rmw_uros_epoch_nanos();
    }

    imuDriver.update();
    ImuData data = imuDriver.getData();
    data.timestamp_ns = timestamp_ns;

    xQueueOverwrite(rtos::ImuQueue, &data);
    vTaskDelayUntil(&wake_time, frequencyToTicks(IMU_TASK_FREQ));
  }
}

void ledIndicatorTask(void* p) {
  UNUSED(p);
  TickType_t wake_time = xTaskGetTickCount();

  while (true) {
    bool battery_low = battery.isLow();
    bool uros_connected = (u_ros::state == u_ros::CONNECTED);
    bool error_state = false;

    ledIndicator.update(battery_low, uros_connected, error_state);
    vTaskDelayUntil(&wake_time, frequencyToTicks(LED_INDICATOR_TASK_FREQ));
  }
}

void ledStripTask(void* p) {
  UNUSED(p);
  TickType_t wake_time = xTaskGetTickCount();

  while (true) {
    vTaskDelayUntil(&wake_time, frequencyToTicks(LED_STRIP_TASK_FREQ));
    PixelIddleAnimation(&PixelStrip, 0x0F, 0x0F, 0x0F, 0x0F, 50);
    vTaskDelayUntil(&wake_time, frequencyToTicks(LED_STRIP_TASK_FREQ));
    PixelIddleAnimation(&PixelStrip, 0x0F, 0x00, 0x00, 0x0F, 50);
  }
}

void monitorTask(void* p) {
  UNUSED(p);
  char buf[1000];

  while (true) {
    if (firmware_log_level <= LOG_LEVEL_INFO) {
      vTaskGetRunTimeStats(buf);
      LOG_INFO("\r\n%s", buf);
    }

    vTaskDelay(frequencyToTicks(MONITOR_TASK_FREQ));
  }
}

void motorControlTask(void* p) {
  UNUSED(p);
  TickType_t wake_time = xTaskGetTickCount();

  while (true) {
    motors.update();

    vTaskDelayUntil(&wake_time, frequencyToTicks(MOTOR_CONTROL_TASK_FREQ));
  }
}

void shutdownTask(void* p) {
  UNUSED(p);
  TickType_t wake_time = xTaskGetTickCount();

  while (true) {
    if (PowerOffSignalLoopHandler() == Shutdown) {
      if (EthClient.connect(SbcIpAddr, SHUTDOWN_PORT, SBC_ETH_CONNECT_TIMEOUT)) {
        EthClient.println("GET /shutdown HTTP/1.1");
        EthClient.stop();
        vTaskDelay(POWEROFF_DELAY);
        digitalWrite(PWR_BRD_GPIO_OUTPUT, HIGH);
      }
    }
    vTaskDelayUntil(&wake_time, frequencyToTicks(SHUTDOWN_TASK_FREQ));
  }
}

void uRosTask(void* p) {
  UNUSED(p);
  while (true) {
    u_ros::loop();
  }
}

}  // namespace rtos
