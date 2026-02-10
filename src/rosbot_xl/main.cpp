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
#include <STM32FreeRTOS.h>

#include "u_ros.hpp"
/*===== HARDEWARE =====*/
#include "bsp.hpp"
#include "robot_config.hpp"
// MOTORS
#include "motors.hpp"
// IMU
#include "hardware/imu.hpp"
// PIXEL
#include "pixel_led.hpp"
/*===== CONNECTIVITY =====*/
#include <LwIP.h>
#include <STM32Ethernet.h>
#include <hal_conf_custom.h>

#include "log.hpp"
#include "rtos.hpp"
#include "uart.hpp"

/* EXTERN VARIABLES */
Log_level_t firmware_log_level = LOG_LEVEL_DEBUG;
extern UartProtocolClass PowerBoardSerial;

// ETHERNET
EthernetClient EthClient;
byte mac[] = {0x02, 0x47, 0x00, 0x00, 0x00, 0x01};

// REST
extern String PowerBoardFirmwareVersion;
extern String PowerBoardVersion;


/*==================== SETUP ========================*/
void setup() {
  // Hardware init
  BoardPheripheralsInit();
  PixelStrip.Init();
  imuDriver.init(IMU_ID, IMU_ADDR_B);

  // RTOS
  rtos::createQueues();
  rtos::createTasks();
  vTaskStartScheduler();
}

static void PidHandlerTask(void* p) {
  TickType_t x_last_wake_time = xTaskGetTickCount();
  TickType_t actual_setpoint_update_time = xTaskGetTickCount();
  TickType_t last_setpoint_update_time = xTaskGetTickCount();
  float setpoint[] = {0, 0, 0, 0};
  static motor_joint_state_t motor_state;
  static uint8_t freq_div_ptr = 0;
  while (1) {
    vTaskDelayUntil(&x_last_wake_time, FREQ_TO_TIME(PID_FREQ));
    if (xQueueReceive(rtos::SetpointQueue, (void*)setpoint, (TickType_t)0)) {
      last_setpoint_update_time = xTaskGetTickCount();
    }
    actual_setpoint_update_time = xTaskGetTickCount();
    if (actual_setpoint_update_time - last_setpoint_update_time >
        MOTORS_SETPOINT_TIMEOUT) {
      for (uint8_t i = 0; i < 4; i++) setpoint[i] = 0;
    }
    for (uint8_t i = 0; i < 4; i++) {
      wheel_motors[i].PidLoopHandler((float)setpoint[i]);
    }
    if (freq_div_ptr > (PID_FREQ / MOTORS_RESPONSE_FREQ)) {
      for (uint8_t i = 0; i < 4; i++) {
        motor_state.velocity[i] =
            ((double)wheel_motors[i].GetVelocity()) / 1000;
        motor_state.position[i] =
            ((double)(wheel_motors[i].GetWheelAbsPosition()) / 1000);
      }
      xQueueOverwrite(rtos::MotorStateQueue, (void*)&motor_state);
      freq_div_ptr = 0;
    }
    freq_div_ptr++;
  }
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

