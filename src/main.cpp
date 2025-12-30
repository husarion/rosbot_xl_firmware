#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <micro_ros_cfg.h>
/*===== HARDEWARE =====*/
#include <battery_types.h>
#include <bsp.h>
#include <hardware_cfg.h>

#include "log.hpp"
// MOTORS
#include <motors.h>
// IMU
#include <ImuLib_cfg.h>
// PIXEL
#if defined(ROSBOT_XL)
#include <PixelLedLib_cfg.h>
#endif
/*===== CONNECTIVITY =====*/
#include <LwIP.h>
#include <STM32Ethernet.h>
#include <UartLib.h>
#include <hal_conf_custom.h>

#include "stm32f407xx.h"
/*===== RTOS =====*/
#include "rtos/queues.hpp"
#include "rtos/tasks.hpp"

/* VARIABLES */
bool uRosInitSuccesfull = false;
// RTOS
QueueHandle_t SetpointQueue;
QueueHandle_t MotorStateQueue;
QueueHandle_t ImuQueue;
QueueHandle_t BatteryStateQueue;
QueueHandle_t uRosPingAgentStatusQueue;

/* EXTERN VARIABLES */
Log_level_t firmware_log_level = LOG_LEVEL_DEBUG;

// IMU
// microROS
extern std_msgs__msg__String msgs;
extern sensor_msgs__msg__Imu imu_msg;
extern sensor_msgs__msg__JointState motors_cmd_msg;
extern sensor_msgs__msg__JointState motors_response_msg;
extern rcl_publisher_t imu_publisher;
extern rcl_publisher_t motor_state_publisher;
// MOTORS
extern TimebaseTimerClass timebase_timer;
extern MotorClass wheel_motors[];

#if defined(ROSBOT_XL)
extern UartProtocolClass PowerBoardSerial;
// LED
extern PixelLedClass pixel_strip;
#endif

// ETHERNET
EthernetClient EthClient;

// REST
FirmwareModeTypeDef firmware_mode = (FirmwareModeTypeDef)DEFAULT_FIRMWARE_MODE;
extern String PowerBoardFirmwareVersion;
extern String PowerBoardVersion;

/* FUNCTIONS */

/*==================== SETUP ========================*/
void setup() {
  // Hardware configuration
  BoardPheripheralsInit();
  uRosTransportInit();

  // RTOS init
  rtos::queues::createAll();
  rtos::tasks::createAll();

  vTaskStartScheduler();
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
