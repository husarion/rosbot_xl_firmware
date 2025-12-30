#include <Arduino.h>
#include <STM32FreeRTOS.h>

#include "micro_ros_cfg.hpp"
/*===== HARDEWARE =====*/
#include "battery_types.hpp"
#include "bsp.hpp"
#include "hardware_cfg.hpp"
#include "ranges.hpp"
// MOTORS
#include "motors.hpp"
// IMU
#include "hardware/imu.hpp"
/*===== CONNECTIVITY =====*/
#include <LwIP.h>
#include <STM32Ethernet.h>
#include <hal_conf_custom.h>

#include "uart.hpp"
/*===== RTOS =====*/
#include "log.hpp"
#include "rtos/queues.hpp"
#include "rtos/tasks.hpp"

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
  // Hardware init
  BoardPheripheralsInit();

  uRosTransportInit();
  if (!imuDriver.init()) {
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
