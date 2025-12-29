/**
 * @file hardware_cfg.h
 * @author Jakub Klein
 * @brief
 * @version 0.1
 * @date 2025-09-25
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef HARDWARE_CFG
#define HARDWARE_CFG

#include <stdint.h>

/* COMMON DEFINITIONS */

#define DEFAULT_FIRMWARE_MODE 2  // 0 - normal; 1 - error; 2 - debug
typedef enum { fw_normal = 0, fw_error = 1, fw_debug = 2 } FirmwareModeTypeDef;

#define RTOS_FREQUENCY 1000      // hz
#define FREQ_TO_TIME(freq) (TickType_t)(RTOS_FREQUENCY / freq * portTICK_PERIOD_MS)
#define FREQ_TO_TICKS(freq) (TickType_t)(configTICK_RATE_HZ / freq)

/* CHOOSE HARDWARE CONFIG */

#if defined(ROSBOT_XL)
  #include "rosbot_xl/hardware_cfg.h"
#elif defined(ROSBOT)
  #include "rosbot/hardware_cfg.h"
#else
  #error "No board version defined! Did you set correct flag in platformio.ini?"
#endif
#include "battery_types.h"

#endif /* HARDWARE_CFG */
