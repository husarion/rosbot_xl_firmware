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

#include <vector>

#include "battery.hpp"
#include "bsp.hpp"
#include "control/encoders_manager.hpp"
#include "control/motors_manager.hpp"
#include "led_indicator.hpp"
#include "robot_config.hpp"
#include "rosbot/tasks.hpp"
#include "sensors/imu.hpp"
#include "sensors/ranges.hpp"
#include "serial_manager.hpp"
#include "u_ros.hpp"

/* EXTERN VARIABLES */
Log_level_t firmware_log_level = LOG_LEVEL_DEBUG;

SerialManager serialManager;
std::vector<uint8_t> ranges_shd_pins = {RANGE_FR_SHD_PIN, RANGE_FL_SHD_PIN,
                                        RANGE_RR_SHD_PIN, RANGE_RL_SHD_PIN};

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
  imuDriver.init(IMU_ID, IMU_ADDR_B, &imu_i2c);
  ledIndicator.init(RED_LED, HIGH);
  motors.init();
  rangeSensorsManager.init(ranges_shd_pins);
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
