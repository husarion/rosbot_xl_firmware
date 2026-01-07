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
#include "hardware/imu.hpp"
#include "hardware_cfg.hpp"
#include "rtos.hpp"
#include "u_ros.hpp"

/* EXTERN VARIABLES */
Log_level_t firmware_log_level = LOG_LEVEL_DEBUG;
FirmwareModeTypeDef firmware_mode = (FirmwareModeTypeDef)DEFAULT_FIRMWARE_MODE;

/*==================== SETUP ========================*/
void setup() {
  // Hardware configuration
  BoardPheripheralsInit();

  // RTOS init
  u_ros::transportInit();
  rtos::createQueues();
  rtos::createTasks();

  vTaskStartScheduler();
}

/*============== LOOP ===============*/
void loop() {}

/*=========== Runtime stats ====================*/
HardwareTimer RuntimeStatsTimer(TIM5);  // TIM5 - 32 bit

void vConfigureTimerForRunTimeStats(void) {
  RuntimeStatsTimer.setPrescaleFactor(
      1680);  // Set prescaler to 2564 => timer frequency = 168MHz/1680 = 100000
              // Hz (from prediv'd by 1 clocksource of 168 MHz)
  RuntimeStatsTimer.setOverflow(
      0xffffffff);              // Set overflow to 32761 => timer
                                // frequency = 65522 Hz / 32761 = 2 Hz
  RuntimeStatsTimer.refresh();  // Make register changes take effect
  RuntimeStatsTimer.resume();   // Start
}

uint32_t vGetTimerValueForRunTimeStats(void) {
  return RuntimeStatsTimer.getCount();
}
