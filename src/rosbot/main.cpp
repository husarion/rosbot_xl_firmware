#include <Arduino.h>

/*===== HARDEWARE =====*/
#include "battery_types.hpp"
#include "bsp.hpp"
#include "hardware/imu.hpp"
#include "hardware_cfg.hpp"
#include "micro_ros_cfg.hpp"
/*===== RTOS =====*/
#include "rtos/queues.hpp"
#include "rtos/tasks.hpp"

/* EXTERN VARIABLES */
Log_level_t firmware_log_level = LOG_LEVEL_DEBUG;
FirmwareModeTypeDef firmware_mode = (FirmwareModeTypeDef)DEFAULT_FIRMWARE_MODE;

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
