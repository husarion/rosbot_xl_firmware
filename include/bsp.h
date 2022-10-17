/**
 * @file bsp.h
 * @author Maciej Kurcius
 * @brief Board support package
 * @version 0.1
 * @date 2022-01-20
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef BSP_H
#define BSP_H

#include "hardware_cfg.h"
#include <Arduino.h>
#include "UartLib.h"

typedef enum{
    Off     = 0,
    On      = 1,
    Toggle  = 2
}SwitchStateTypeDef;

void BoardGpioInit(void);
void SetLocalPower(SwitchStateTypeDef State_);
void SetGreenLed(SwitchStateTypeDef State_);
void SetRedLed(SwitchStateTypeDef State_);
void BoardPheripheralsInit(void);




#endif /* BSP_H */