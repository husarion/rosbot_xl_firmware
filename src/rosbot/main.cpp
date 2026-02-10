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
#include "control/encoders_manager.hpp"
#include "control/motors_manager.hpp"
#include "led_indicator.hpp"
#include "robot_config.hpp"
#include "rtos.hpp"
#include "sensors/imu.hpp"
#include "sensors/ranges.hpp"
#include "serial_manager.hpp"
#include "u_ros.hpp"

/* EXTERN VARIABLES */
Log_level_t firmware_log_level = LOG_LEVEL_DEBUG;

SerialManager serialManager;

void BoardPheripheralsInit() {
  // Initialize Buttons
  pinMode(PUSH_BUTTON1, INPUT_PULLUP);
  pinMode(PUSH_BUTTON2, INPUT_PULLUP);

  // Initialize LEDs
  pinMode(RED_LED, OUTPUT);
  pinMode(GRN_LED, OUTPUT);
  pinMode(GRN_LED2, OUTPUT);
  digitalWrite(RED_LED, HIGH);

  // Enable power for IMU sensor
  pinMode(IMU_POWER_ON, OUTPUT);
  digitalWrite(IMU_POWER_ON, HIGH);
}

/*==================== SETUP ========================*/
void setup() {
  // Peripherals initialization
  BoardPheripheralsInit();

  // Pre-communication
  serialManager.init();
  const auto& selected_serial = serialManager.selectActive();
  serialManager.configureNamespace();

  // Sensors initialization
  battery.init(BATTERY_ADC_PIN);
  encoders.init();
  imuDriver.init(IMU_ID, IMU_ADDR_B);
  ledIndicator.init(RED_LED, HIGH);
  motors.init();
  rangeSensorsManager.init();
  u_ros::transportInit(selected_serial);

  // RTOS
  rtos::createQueues();
  rtos::createTasks();
  vTaskStartScheduler();
}

/*============== LOOP ===============*/
void loop() {}

/*=========== Runtime stats ====================*/
HardwareTimer RunTimeStatsTimer(TIM5);

void vConfigureTimerForRunTimeStats(void) {
  RunTimeStatsTimer.setPrescaleFactor(
      1680);  // every 10 µs (168MHz / 1680 = 100kHz)
  RunTimeStatsTimer.setOverflow(0xFFFFFFFF);
  RunTimeStatsTimer.refresh();
  RunTimeStatsTimer.resume();
}

uint32_t vGetTimerValueForRunTimeStats(void) {
  return RunTimeStatsTimer.getCount();
}
