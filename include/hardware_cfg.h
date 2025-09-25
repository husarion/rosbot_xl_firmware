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

typedef enum { fw_normal = 0, fw_error = 1, fw_debug = 2 } FirmwareModeTypeDef;


/* CHOOSE HARDWARE CONFIG */

#ifdef BOARD_ROSBOT_XL
  #warning "INFO: Selected config: BOARD_ROSBOT_XL"
  #include "hardware_cfg_rosbot_xl.h"
#elif defined(BOARD_ROSBOT_2)
  #warning "INFO: Selected config: BOARD_ROSBOT_2"
  #include "hardware_cfg_rosbot_2.h"
#else
  #error "No board version defined! Did you set correct flag in platformio.ini?"
#endif

#endif /* HARDWARE_CFG */
