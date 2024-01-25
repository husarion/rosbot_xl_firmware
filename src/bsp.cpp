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

#include "bsp.h"

#if EXT_SERIAL_EN_FLAG == 1
HardwareSerial EXT_SERIAL(EXT_SERIAL_RX, EXT_SERIAL_TX);
#endif
UartProtocolClass PowerBoardSerial(
  PWR_BRD_SERIAL_RX, PWR_BRD_SERIAL_TX, PWR_BRD_SERIAL_BAUDRATE, PWR_BRD_SERIAL_CONFIG);
String PowerBoardFirmwareVersion = "";
String PowerBoardVersion = "";
extern FirmwareModeTypeDef firmware_mode;
TwoWire I2cBus(IMU_SDA, IMU_SCL);
HardwareTimer FanTimer(FAN_PWM_TIMER);

void BoardGpioInit(void)
{
  digitalWrite(GRN_LED, LOW);
  pinMode(GRN_LED, OUTPUT);
  digitalWrite(EN_LOC_5V, LOW);
  pinMode(EN_LOC_5V, OUTPUT);
  digitalWrite(RD_LED, LOW);
  pinMode(RD_LED, OUTPUT);
  digitalWrite(PWR_BRD_GPIO_OUTPUT, LOW);
  pinMode(PWR_BRD_GPIO_OUTPUT, OUTPUT);
  pinMode(PWR_BRD_GPIO_INPUT, INPUT_PULLUP);
  digitalWrite(AUDIO_SHDN, HIGH);
  pinMode(AUDIO_SHDN, OUTPUT);
}

void SetLocalPower(SwitchStateTypeDef State_)
{
  if (State_ == Off) digitalWrite(EN_LOC_5V, LOW);
  if (State_ == On) digitalWrite(EN_LOC_5V, HIGH);
  if (State_ == Toggle) digitalToggle(EN_LOC_5V);
}

void SetGreenLed(SwitchStateTypeDef State_)
{
  if (State_ == Off) digitalWrite(GRN_LED, LOW);
  if (State_ == On) digitalWrite(GRN_LED, HIGH);
  if (State_ == Toggle) digitalToggle(GRN_LED);
}

void SetRedLed(SwitchStateTypeDef State_)
{
  if (State_ == Off) digitalWrite(RD_LED, LOW);
  if (State_ == On) digitalWrite(RD_LED, HIGH);
  if (State_ == Toggle) digitalToggle(RD_LED);
}

void BoardPheripheralsInit(void)
{
  BoardGpioInit();
  if (firmware_mode == fw_debug) {
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM6_STOP;  // set debug options
  }
  // SBC Serial port init
  SBC_SERIAL.setRx(SBC_SERIAL_RX);
  SBC_SERIAL.setTx(SBC_SERIAL_TX);
  SBC_SERIAL.begin(SBC_SERIAL_BAUDRATE);
  SBC_SERIAL.println("Hello SBC");
  // Power Board Serial port init
  PowerBoardSerial.setTimeout(PWR_BRD_SERIAL_TIMEOUT);
  PowerBoardSerial.begin(PWR_BRD_SERIAL_BAUDRATE);
// External Serial port init
#if EXT_SERIAL_EN_FLAG == 1
  EXT_SERIAL.begin(EXT_SERIAL_BAUDRATE);
  EXT_SERIAL.println("Hello external device");
#endif
  SetLocalPower(On);
  I2cBusInit();
  delay(250);
  SetMaxMotorsCurrent(ILIM1, ILIM2, ILIM3, ILIM4);
}

PowerOffSignalTypeDef PowerOffSignalLoopHandler(void)
{
  if (digitalRead(PWR_BRD_GPIO_INPUT))
    return Shutdown;
  else
    return Idle;
}

String GetBoardVersion(void)
{
  // uint8_t BoardVer2Write[] = {'v', '1', '.', '2'};
  // EepromWritePage(BOARD_VER_MEM_BLOCK, BOARD_VER_MEM_ADDR,
  // (uint8_t*)BoardVer2Write, BOARD_VER_MEM_SIZE);
  static String BoardVersion = (String) "unknown";
  if (BoardVersion == (String) "unknown") {
    char RetVal[BOARD_VER_MEM_SIZE + 1];
    RetVal[BOARD_VER_MEM_SIZE] = '\0';
    for (uint8_t i = 0; i < BOARD_VER_READ_ATTEMPTS; i++) {
      if (
        EepromReadPage(
          BOARD_VER_MEM_BLOCK, BOARD_VER_MEM_ADDR, (uint8_t *)RetVal, BOARD_VER_MEM_SIZE) != -1) {
        BoardVersion = String(RetVal);
        return BoardVersion;
      }
      break;
    }
    BoardVersion = (String) "v1.1";
  }
  return BoardVersion;
}

void I2cBusInit(void) { I2cBus.begin(); }

void TestFunction(uint8_t state)
{
  if (state == 1) SetGreenLed(On);
  if (state == 0) SetGreenLed(Off);
  UartProtocolFrame TestFrame;
  TestFrame.arg_size = 12;
  TestFrame.cmd = 25;
  TestFrame.args[0] = 1;
  TestFrame.args[1] = 1;
  TestFrame.args[2] = 8;
  TestFrame.args[11] = state;
  PowerBoardSerial.SendFrame(TestFrame);
}

void PbInfoRequest(void)
{
  UartProtocolFrame PbInfoReqFrame;
  PbInfoReqFrame.arg_size = 1;
  PbInfoReqFrame.cmd = 1;
  PbInfoReqFrame.args[0] = 0;
  PowerBoardSerial.SendFrame(PbInfoReqFrame);
}

void BatteryInfoRequest(void)
{
  UartProtocolFrame PbInfoReqFrame;
  PbInfoReqFrame.arg_size = 18;
  PbInfoReqFrame.cmd = 2;
  PbInfoReqFrame.args[0] = 0;
  PowerBoardSerial.SendFrame(PbInfoReqFrame);
}

void FanHardwareInit(void)
{
  if (GetBoardVersion() == (String) "v1.1") {
    digitalWrite(FAN_PP_PIN, LOW);
    pinMode(FAN_PP_PIN, OUTPUT);
  }
  if (GetBoardVersion() == (String) "v1.2") {
    analogReadResolution(ADC_RESOLUTION);
    digitalWrite(FAN_PWM_PIN, LOW);
    pinMode(FAN_PWM_PIN, OUTPUT);
  }
}

void FanLoopHanlder(void)
{
  if (GetBoardVersion() == (String) "v1.1") {
    digitalWrite(FAN_PP_PIN, HIGH);
  } else if (GetBoardVersion() == (String) "v1.2") {
    if (GetInsideTemperature() < FAN_TEMP_THRSH_DOWN) {
      digitalWrite(FAN_PWM_PIN, LOW);
    } else if (GetInsideTemperature() > FAN_TEMP_THRSH_UP) {
      digitalWrite(FAN_PWM_PIN, HIGH);
    }
  }
}

int8_t GetInsideTemperature(void)
{
  float InsideTemp, logR2, SensorResistance, AdcValue;
  AdcValue = (float)analogRead(NTC_SENS_PIN);
  SensorResistance = (AdcValue * NTC_PULLUP_RES) / (1023 - AdcValue);
  logR2 = log(SensorResistance);
  InsideTemp = (1.0 / (NTC_SENS_C1 + NTC_SENS_C2 * logR2 + NTC_SENS_C3 * logR2 * logR2 * logR2)) -
               NTC_OFFSET_VAL;
  return (int8_t)InsideTemp;
}

uint8_t EepromWriteByte(uint8_t BlockAddr, uint8_t ByteAddr, uint8_t Value)
{
  uint8_t DataToSend[] = {ByteAddr, Value};
  I2cBus.beginTransmission(EEPROM_CONTROL_BYTE(EEPROM_DEV_ID, BlockAddr));
  if (I2cBus.write(DataToSend, 2) == 2) {
    I2cBus.endTransmission();
    return 0;
  }
  I2cBus.endTransmission();
  return -1;
}

uint8_t EepromReadByte(uint8_t BlockAddr, uint8_t ByteAddr, uint8_t * Value)
{
  I2cBus.beginTransmission(EEPROM_CONTROL_BYTE(EEPROM_DEV_ID, BlockAddr));
  I2cBus.write(ByteAddr);
  I2cBus.endTransmission();
  I2cBus.requestFrom(EEPROM_CONTROL_BYTE(EEPROM_DEV_ID, BlockAddr), 1);
  if (I2cBus.available()) {
    int RetVal = I2cBus.read();
    if (RetVal == -1) return -1;
    *Value = (uint8_t)RetVal;
    return 0;
  }
  return -1;
}

uint8_t EepromWritePage(uint8_t BlockAddr, uint8_t ByteAddr, uint8_t * Value, uint8_t Size)
{
  uint8_t DataToSend[Size + 1];
  DataToSend[0] = ByteAddr;
  memcpy(DataToSend + 1, Value, Size);
  I2cBus.beginTransmission(EEPROM_CONTROL_BYTE(EEPROM_DEV_ID, BlockAddr));
  if (I2cBus.write(DataToSend, Size + 1) == Size + 1) {
    I2cBus.endTransmission();
    return 0;
  }
  I2cBus.endTransmission();
  return -1;
}

uint8_t EepromReadPage(uint8_t BlockAddr, uint8_t ByteAddr, uint8_t * Value, uint8_t Size)
{
  I2cBus.beginTransmission(EEPROM_CONTROL_BYTE(EEPROM_DEV_ID, BlockAddr));
  I2cBus.write(ByteAddr);
  I2cBus.endTransmission();
  I2cBus.requestFrom(EEPROM_CONTROL_BYTE(EEPROM_DEV_ID, BlockAddr), (int)Size);
  if (I2cBus.available()) {
    if (I2cBus.readBytes(Value, Size) == Size)
      ;
    return 0;
  }
  return -1;
}
