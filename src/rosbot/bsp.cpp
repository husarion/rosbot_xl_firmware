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

#include "bsp.hpp"

#include "hardware/imu.hpp"
#include "log.hpp"

#if EXT_SERIAL_EN_FLAG == 1
HardwareSerial EXT_SERIAL(EXT_SERIAL_RX, EXT_SERIAL_TX);
#endif

HardwareSerial SBC_SERIAL(SBC_SERIAL_TX, SBC_SERIAL_RX);
String PowerBoardFirmwareVersion = "";
String PowerBoardVersion = "";
extern FirmwareModeTypeDef firmware_mode;
TwoWire I2cBus(IMU_SDA, IMU_SCL);

void BoardGpioInit(void) {
  digitalWrite(RD_LED, LOW);
  pinMode(RD_LED, OUTPUT);
  digitalWrite(GRN_LED, LOW);
  pinMode(GRN_LED, OUTPUT);
  digitalWrite(GRN_LED2, LOW);
  pinMode(GRN_LED2, OUTPUT);
}

void SetGreenLed(SwitchStateTypeDef State_) {
  if (State_ == Off) digitalWrite(GRN_LED, LOW);
  if (State_ == On) digitalWrite(GRN_LED, HIGH);
  if (State_ == Toggle) digitalToggle(GRN_LED);
}

void SetGreenLed2(SwitchStateTypeDef State_) {
  if (State_ == Off) digitalWrite(GRN_LED2, LOW);
  if (State_ == On) digitalWrite(GRN_LED2, HIGH);
  if (State_ == Toggle) digitalToggle(GRN_LED2);
}

void SetRedLed(SwitchStateTypeDef State_) {
  if (State_ == Off) digitalWrite(RD_LED, LOW);
  if (State_ == On) digitalWrite(RD_LED, HIGH);
  if (State_ == Toggle) digitalToggle(RD_LED);
}

void BoardPheripheralsInit(void) {
  BoardGpioInit();
  if (firmware_mode == fw_debug) {
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM6_STOP;  // set debug options
  }

  // Enable power for IMU sensor
  pinMode(IMU_POWER_ON, OUTPUT);
  digitalWrite(IMU_POWER_ON, HIGH);

  // FTDI UART-USB init
  FTDI_SERIAL.setRx(FTDI_SERIAL_RX);
  FTDI_SERIAL.setTx(FTDI_SERIAL_TX);
  FTDI_SERIAL.setTimeout(FTDI_SERIAL_TIMEOUT);
  FTDI_SERIAL.begin(FTDI_SERIAL_BAUDRATE);

  I2cBusInit();
  delay(250);

  if (!imuDriver.init()) {
    LOG_ERROR("imuDriver.Init() failed!");
  }
}

PowerOffSignalTypeDef PowerOffSignalLoopHandler(void) { return Idle; }

String GetBoardVersion(void) {
  static String BoardVersion = (String) "core2";
  return BoardVersion;
}

void I2cBusInit(void) { I2cBus.begin(); }

int8_t GetInsideTemperature(void) {
  float InsideTemp, logR2, SensorResistance, AdcValue;
  AdcValue = (float)analogRead(NTC_SENS_PIN);
  SensorResistance = (AdcValue * NTC_PULLUP_RES) / (1023 - AdcValue);
  logR2 = log(SensorResistance);
  InsideTemp = (1.0 / (NTC_SENS_C1 + NTC_SENS_C2 * logR2 +
                       NTC_SENS_C3 * logR2 * logR2 * logR2)) -
               NTC_OFFSET_VAL;
  return (int8_t)InsideTemp;
}
