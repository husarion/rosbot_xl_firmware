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

#include <Arduino.h>

#include "battery.hpp"
#include "bsp.hpp"
#include "control/encoders_manager.hpp"
#include "control/motors_manager.hpp"
#include "led_indicator.hpp"
#include "namespace_config.hpp"
#include "robot_config.hpp"
#include "rosbot/tasks.hpp"
#include "sensors/imu.hpp"
#include "sensors/ranges.hpp"
#include "u_ros.hpp"

/* EXTERN VARIABLES */
Log_level_t firmware_log_level = LOG_LEVEL_DEBUG;
FirmwareModeTypeDef firmware_mode = (FirmwareModeTypeDef)DEFAULT_FIRMWARE_MODE;

const SerialConfig* g_serialConfig = nullptr;
char g_namespace[NS_MAX_LENGTH] = {0};

/*==================== SETUP ========================*/
void setup() {
  // Hardware configuration
  BoardPheripheralsInit();

  battery.init(BATTERY_ADC_PIN);
  encoders.init();
  imuDriver.init();
  ledIndicator.init(RD_LED);
  motors.init();
  rangeSensorsManager.init();

  g_serialConfig = &serial_selector::selectSerialConfig();
  ns_config::configure(*g_serialConfig, g_namespace, NS_MAX_LENGTH);
  u_ros::transportInit(*g_serialConfig);

  // RTOS
  rtos::createQueues();
  rtos::createTasks();
  vTaskStartScheduler();
}

/*============== LOOP ===============*/
void loop() {}

/*=========== Runtime stats ====================*/
HardwareTimer RuntimeStatsTimer(TIM5);

void vConfigureTimerForRunTimeStats(void) {
  // Prescaler: 168 MHz / 1680 = 100 kHz (10 µs per tick)
  RuntimeStatsTimer.setPrescaleFactor(1680);

  // Auto-reload set to maximum 32-bit
  RuntimeStatsTimer.setOverflow(0xFFFFFFFF);

  // Apply changes and start timer
  RuntimeStatsTimer.refresh();
  RuntimeStatsTimer.resume();
}

uint32_t vGetTimerValueForRunTimeStats(void) {
  return RuntimeStatsTimer.getCount();
}
