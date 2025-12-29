#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <micro_ros_cfg.h>
/*===== HARDEWARE =====*/
#include <bsp.h>
#include <hardware_cfg.h>
#include <battery_types.h>
#include <ranges.h>
// MOTORS
#include <motors.h>
// IMU
#include "hardware/imu.h"
/*===== CONNECTIVITY =====*/
#include <LwIP.h>
#include <STM32Ethernet.h>
#include <UartLib.h>
#include <hal_conf_custom.h>
#include "stm32f407xx.h"
/*===== RTOS =====*/
#include "rtos/tasks.h"
#include "rtos/queues.h"
#include "log.h"

/* EXTERN VARIABLES */
Log_level_t firmware_log_level = LOG_LEVEL_DEBUG;

// microROS
extern std_msgs__msg__String msgs;
extern sensor_msgs__msg__Imu imu_msg;
extern sensor_msgs__msg__JointState motors_cmd_msg;
extern sensor_msgs__msg__JointState motors_response_msg;
extern rcl_publisher_t imu_publisher;
extern rcl_publisher_t motor_state_publisher;
// MOTORS
extern TimebaseTimerClass timebase_timer;

// REST
FirmwareModeTypeDef firmware_mode = (FirmwareModeTypeDef)DEFAULT_FIRMWARE_MODE;
extern String PowerBoardFirmwareVersion;
extern String PowerBoardVersion;

/*==================== SETUP ========================*/
void setup() {
  Serial.setRx(FTDI_SERIAL_RX);
  Serial.setTx(FTDI_SERIAL_TX);
  Serial.setTimeout(FTDI_SERIAL_TIMEOUT);
  Serial.begin(FTDI_SERIAL_BAUDRATE);

  // Hardware init
  BoardPheripheralsInit();
  uRosTransportInit();
  if(!imuDriver.Init()) {
    LOG_ERROR("imuDriver.Init() failed!");
  }
  SetGreenLed(On);
  delay(150);
  SetGreenLed(Off);

  rtos::queues::createAll();
  rtos::tasks::createAll();

  vTaskStartScheduler();
}

/*============== LOOP - IDDLE TASK ===============*/
void loop() { ; }

/*=========== Runtime stats ====================*/
HardwareTimer RuntimeStatsTimer(TIM5);  // TIM5 - 32 bit

void vConfigureTimerForRunTimeStats(void)
{
  RuntimeStatsTimer.setPrescaleFactor(
    1680);  // Set prescaler to 2564 => timer frequency = 168MHz/1680 = 100000
            // Hz (from prediv'd by 1 clocksource of 168 MHz)
  RuntimeStatsTimer.setOverflow(0xffffffff);  // Set overflow to 32761 => timer
                                              // frequency = 65522 Hz / 32761 = 2 Hz
  RuntimeStatsTimer.refresh();                // Make register changes take effect
  RuntimeStatsTimer.resume();                 // Start
}

uint32_t vGetTimerValueForRunTimeStats(void) { return RuntimeStatsTimer.getCount(); }
