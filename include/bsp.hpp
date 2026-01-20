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

#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "robot_config.hpp"
#include "motors.hpp"
#include "uart.hpp"

typedef enum { Off = 0, On = 1, Toggle = 2 } SwitchStateTypeDef;
typedef enum { Idle = 0, Shutdown = 1 } PowerOffSignalTypeDef;

extern TwoWire imu_i2c;
extern TwoWire range_i2c;

void BoardGpioInit(void);
void SetLocalPower(SwitchStateTypeDef State_);
void SetGreenLed(SwitchStateTypeDef State_);
void SetGreenLed2(SwitchStateTypeDef State_);
void SetRedLed(SwitchStateTypeDef State_);
void BoardPheripheralsInit(void);
PowerOffSignalTypeDef PowerOffSignalLoopHandler(void);
String GetBoardVersion(void);
void I2cBusInit(void);

// POWER BOARD FUNCTIONS

void TestFunction(uint8_t);
void RoboticArmInvReset(void);
void PbInfoRequest(void);
void BatteryInfoRequest(void);
void FanHardwareInit(void);
void FanLoopHanlder(void);
int8_t GetInsideTemperature(void);

// EEPROM FUNCTIONS

uint8_t EepromWriteByte(uint8_t BlockAddr, uint8_t ByteAddr, uint8_t value);
uint8_t EepromReadByte(uint8_t BlockAddr, uint8_t ByteAddr, uint8_t* value);
bool EepromWritePage(uint8_t BlockAddr, uint8_t ByteAddr, const uint8_t* Value,
                     uint8_t Size);
bool EepromReadPage(uint8_t BlockAddr, uint8_t ByteAddr, uint8_t* Value,
                    uint8_t Size);
