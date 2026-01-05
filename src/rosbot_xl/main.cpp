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

#include "micro_ros_cfg.hpp"
/*===== HARDEWARE =====*/
#include "bsp.hpp"
#include "hardware_cfg.hpp"
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
#include "rtos/queues.hpp"
#include "uart.hpp"

/* VARIABLES */
bool uRosInitSuccesfull = false;
// RTOS
QueueHandle_t SetpointQueue;
QueueHandle_t MotorStateQueue;
QueueHandle_t ImuQueue;
QueueHandle_t BatteryStateQueue;
QueueHandle_t uRosAgentConectionQueue;
portBASE_TYPE s1, s2, s3, s4, s5, s6, s7, s8, s9, s10;

/* EXTERN VARIABLES */
Log_level_t firmware_log_level = LOG_LEVEL_DEBUG;
extern UartProtocolClass PowerBoardSerial;

// microROS
extern std_msgs__msg__String msgs;
extern sensor_msgs__msg__Imu imu_msg;
extern sensor_msgs__msg__JointState motors_cmd_msg;
extern sensor_msgs__msg__JointState motors_response_msg;
extern rcl_publisher_t imu_pub;
extern rcl_publisher_t motor_state_pub;
// MOTORS
extern TimebaseTimerClass timebase_timer;
// LED
extern PixelLedClass pixel_strip;

// ETHERNET
IPAddress client_ip;
IPAddress agent_ip;
EthernetClient EthClient;
byte mac[] = {0x02, 0x47, 0x00, 0x00, 0x00, 0x01};

// REST
FirmwareModeTypeDef firmware_mode = (FirmwareModeTypeDef)DEFAULT_FIRMWARE_MODE;
extern String PowerBoardFirmwareVersion;
extern String PowerBoardVersion;

/* RTOS TASKS DECLARATIONS */
static void uRosSpinTask(void* p);
static void ImuTask(void* p);
static void PidHandlerTask(void* p);
static void PixelLedTask(void* p);
static void SbcShutdownTask(void* p);
static void PowerBoardTask(void* p);
static void uRosPingTask(void* p);
static void HardwareLoopTask(void* p);
static void RuntimeStatsTask(void* p);

/* FUNCTIONS */

/*==================== SETUP ========================*/
void setup() {
  // Hardware init
  BoardPheripheralsInit();
  PixelStrip.Init();
  imuDriver.init();
  SetGreenLed(On);
  delay(150);
  SetGreenLed(Off);

  /* RTOS QUEUES CREATION */
  SetpointQueue = xQueueCreate(1, sizeof(double) * 4);
  MotorStateQueue = xQueueCreate(1, sizeof(motor_joint_state_t));
  ImuQueue = xQueueCreate(1, sizeof(imu_data_t));
  BatteryStateQueue = xQueueCreate(1, sizeof(battery_state_t));
  uRosAgentConectionQueue = xQueueCreate(1, sizeof(u_ros_state_t));
  LOG_DEBUG("Queues created");
  /* RTOS TASKS CREATION */
  s1 =
      xTaskCreate(uRosSpinTask, "uRosSpinTask", configMINIMAL_STACK_SIZE + 2500,
                  NULL, tskIDLE_PRIORITY + 1, NULL);
  if (s1 != pdPASS) LOG_DEBUG("S1 creation problem");
  s2 = xTaskCreate(ImuTask, "ImuTask", configMINIMAL_STACK_SIZE + 750, NULL,
                   tskIDLE_PRIORITY + 1, NULL);
  if (s2 != pdPASS) LOG_DEBUG("S2 creation problem");
  s3 = xTaskCreate(RuntimeStatsTask, "RuntimeStatsTask",
                   configMINIMAL_STACK_SIZE + 500, NULL, tskIDLE_PRIORITY + 1,
                   NULL);
  if (s3 != pdPASS) LOG_DEBUG("S3 creation problem");
  s4 = xTaskCreate(PidHandlerTask, "PidHandlerTask",
                   configMINIMAL_STACK_SIZE + 1000, NULL, tskIDLE_PRIORITY + 3,
                   NULL);
  if (s4 != pdPASS) LOG_DEBUG("S4 creation problem");
  s5 = xTaskCreate(PixelLedTask, "PixelLedTask", configMINIMAL_STACK_SIZE + 750,
                   NULL, tskIDLE_PRIORITY + 1, NULL);
  if (s5 != pdPASS) LOG_DEBUG("S5 creation problem");
  s7 = xTaskCreate(SbcShutdownTask, "SbcShutdownTask",
                   configMINIMAL_STACK_SIZE + 500, NULL, tskIDLE_PRIORITY + 1,
                   NULL);
  if (s7 != pdPASS) LOG_DEBUG("S7 creation problem");
  s8 = xTaskCreate(PowerBoardTask, "PowerBoardTask",
                   configMINIMAL_STACK_SIZE + 500, NULL, tskIDLE_PRIORITY + 1,
                   NULL);
  if (s8 != pdPASS) LOG_DEBUG("S8 creation problem");
  s9 = xTaskCreate(uRosPingTask, "uRosPingTask", configMINIMAL_STACK_SIZE + 500,
                   NULL, tskIDLE_PRIORITY + 1, NULL);
  if (s9 != pdPASS) LOG_DEBUG("S9 creation problem");
  s10 = xTaskCreate(HardwareLoopTask, "BoardHardwareLoopTask",
                    configMINIMAL_STACK_SIZE + 500, NULL, tskIDLE_PRIORITY + 1,
                    NULL);
  if (s10 != pdPASS) LOG_DEBUG("S10 creation problem");
  /* START RTOS */
  LOG_DEBUG("Tasks starting");
  vTaskStartScheduler();
}

static void uRosSpinTask(void* p) {
  UNUSED(p);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static u_ros_state_t uRosPingAgentStatus;
  while (1) {
    xQueueReceive(rtos::queues::uRosAgentConectionQueue, &uRosPingAgentStatus,
                  (TickType_t)0);
    vTaskDelayUntil(&xLastWakeTime, 1);
    uRosLoopHandler(uRosPingAgentStatus);
  }
}

static void ImuTask(void* p) {
  static imu_data_t queue_imu;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {
    queue_imu = imuDriver.loopHandler();
    xQueueOverwrite(rtos::queues::ImuQueue, (void*)&queue_imu);
    vTaskDelayUntil(&xLastWakeTime, FREQ_TO_TIME(IMU_SAMPLE_FREQ));
  }
}

static void PidHandlerTask(void* p) {
  TickType_t x_last_wake_time = xTaskGetTickCount();
  TickType_t actual_setpoint_update_time = xTaskGetTickCount();
  TickType_t last_setpoint_update_time = xTaskGetTickCount();
  double setpoint[] = {0, 0, 0, 0};
  static motor_joint_state_t motor_state;
  static uint8_t freq_div_ptr = 0;
  while (1) {
    vTaskDelayUntil(&x_last_wake_time, FREQ_TO_TIME(PID_FREQ));
    if (xQueueReceive(rtos::queues::SetpointQueue, (void*)setpoint,
                      (TickType_t)0)) {
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
      xQueueOverwrite(rtos::queues::MotorStateQueue, (void*)&motor_state);
      freq_div_ptr = 0;
    }
    freq_div_ptr++;
  }
}

static void PixelLedTask(void* p) {
  while (1) {
    vTaskDelay(FREQ_TO_TIME(PIXEL_ANIMATION_FREQ));
    PixelIddleAnimation(&PixelStrip, 0x0F, 0x0F, 0x0F, 0x0F, 50);
    vTaskDelay(FREQ_TO_TIME(PIXEL_ANIMATION_FREQ));
    PixelIddleAnimation(&PixelStrip, 0x0F, 0x00, 0x00, 0x0F, 50);
  }
}

static void SbcShutdownTask(void* p) {
  IPAddress SbcIpAddr;
  SbcIpAddr.fromString(SBC_AGENT_IP);
  while (1) {
    vTaskDelay(250);
    if (PowerOffSignalLoopHandler() == Shutdown) {
      while (1) {
        if (EthClient.connect(SbcIpAddr, SHUTDOWN_PORT,
                              SBC_ETH_CONNECT_TIMEOUT)) {
          EthClient.println("GET /shutdown HTTP/1.1");
          EthClient.stop();
          vTaskDelay(POWEROFF_DELAY);
          digitalWrite(PWR_BRD_GPIO_OUTPUT, HIGH);
        }
        vTaskDelay(10);
      }
    }
  }
}

static void PowerBoardTask(void* p) {
  uint16_t TimeDivider = 0;
  while (1) {
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

static void uRosPingTask(void* p) {
  static u_ros_state_t uRosPingAgentStatus;
  client_ip.fromString(CLIENT_IP);
  agent_ip.fromString(SBC_AGENT_IP);
  set_microros_native_ethernet_udp_transports(mac, client_ip, agent_ip,
                                              AGENT_PORT);
  while (1) {
    uRosPingAgentStatus =
        uRosPingAgent(uROS_PING_TIMEOUT_MS, uROS_PING_ATTEMPTS);
    xQueueOverwrite(rtos::queues::uRosAgentConectionQueue,
                    (void*)&uRosPingAgentStatus);
    switch (uRosPingAgentStatus) {
      case Ok:
        SetGreenLed(On);
        SetRedLed(Off);
        break;
      case Error:
        SetGreenLed(Off);
        SetRedLed(Toggle);
        break;
      case Default:
        SetGreenLed(Toggle);
        SetRedLed(Off);
        break;
      default:
        SetGreenLed(Off);
        SetRedLed(Off);
        break;
    }
    vTaskDelay(FREQ_TO_TIME(uROS_PING_FREQUENCY));
  }
}

static void HardwareLoopTask(void* p) {
  vTaskDelay(1000);
  FanHardwareInit();
  while (1) {
    FanLoopHanlder();
    vTaskDelay(100);
  }
}

static void RuntimeStatsTask(void* p) {
  char buf[2000];
  LOG_DEBUG("runtime stats task started");
  while (1) {
    if (firmware_mode == fw_debug) {
      vTaskGetRunTimeStats(buf);
      Serial.printf("\r\n%s\r\n-------------", buf);
    }
    vTaskDelay(100);
  }
}

/*============== LOOP - IDDLE TASK ===============*/
void loop() { ; }

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
