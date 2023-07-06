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

// HardwareSerial PWR_BRD_SERIAL(PWR_BRD_SERIAL_RX, PWR_BRD_SERIAL_TX);
#if EXT_SERIAL_EN_FLAG == 1
    HardwareSerial EXT_SERIAL(EXT_SERIAL_RX, EXT_SERIAL_TX);
#endif
UartProtocolClass PowerBoardSerial(PWR_BRD_SERIAL_RX, PWR_BRD_SERIAL_TX, PWR_BRD_SERIAL_BAUDRATE, PWR_BRD_SERIAL_CONFIG);
String PowerBoardFirmwareVersion = "";
String PowerBoardVersion = "";
extern FirmwareModeTypeDef firmware_mode;



void BoardGpioInit(void){
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
    digitalWrite(FAN, HIGH);
    pinMode(FAN, OUTPUT);
}

// void MotorsPinInit(void){
//     digitalWrite(GRN_LED, LOW);
//     pinMode(ILIM1, INPUT);
// }

void SetLocalPower(SwitchStateTypeDef State_){
    if(State_ == Off)    digitalWrite(EN_LOC_5V, LOW);
    if(State_ == On)     digitalWrite(EN_LOC_5V, HIGH);
    if(State_ == Toggle) digitalToggle(EN_LOC_5V);
}

void SetGreenLed(SwitchStateTypeDef State_){
    if(State_ == Off)    digitalWrite(GRN_LED, LOW);
    if(State_ == On)     digitalWrite(GRN_LED, HIGH);
    if(State_ == Toggle) digitalToggle(GRN_LED);
}

void SetRedLed(SwitchStateTypeDef State_){
    if(State_ == Off)    digitalWrite(RD_LED, LOW);
    if(State_ == On)     digitalWrite(RD_LED, HIGH);
    if(State_ == Toggle) digitalToggle(RD_LED);
}

void BoardPheripheralsInit(void){
    BoardGpioInit();
    if(firmware_mode == fw_debug){
        DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM6_STOP; // set debug options
    }
    //SBC Serial port init
    SBC_SERIAL.setRx(SBC_SERIAL_RX);
    SBC_SERIAL.setTx(SBC_SERIAL_TX);
    SBC_SERIAL.begin(SBC_SERIAL_BAUDRATE);
    SBC_SERIAL.println("Hello SBC");
    //Power Board Serial port init
    PowerBoardSerial.setTimeout(PWR_BRD_SERIAL_TIMEOUT);
    PowerBoardSerial.begin(PWR_BRD_SERIAL_BAUDRATE);
    //External Serial port init
    #if EXT_SERIAL_EN_FLAG == 1
        EXT_SERIAL.begin(EXT_SERIAL_BAUDRATE);
        EXT_SERIAL.println("Hello external device");
    #endif
    // IWatchdog.begin(WATCHDOG_TIMEOUT);
    SetLocalPower(On);
    delay(250);
}

PowerOffSignalTypeDef PowerOffSignalLoopHandler(void){
    if(digitalRead(PWR_BRD_GPIO_INPUT))
        return Shutdown;
    else
        return Idle;
}

void TestFunction(uint8_t state){
    if(state == 1) SetGreenLed(On);
    if(state == 0) SetGreenLed(Off);
    UartProtocolFrame TestFrame;
    TestFrame.arg_size = 12;
    TestFrame.cmd = 25;
    TestFrame.args[0] = 1;
    TestFrame.args[1] = 1;
    TestFrame.args[2] = 8;
    TestFrame.args[11] = state;
    PowerBoardSerial.SendFrame(TestFrame);
}

void PbInfoRequest(void){
    UartProtocolFrame PbInfoReqFrame;
    PbInfoReqFrame.arg_size = 1;
    PbInfoReqFrame.cmd = 1;
    PbInfoReqFrame.args[0] = 0;
    PowerBoardSerial.SendFrame(PbInfoReqFrame);
}

void BatteryInfoRequest(void){
    UartProtocolFrame PbInfoReqFrame;
    PbInfoReqFrame.arg_size = 18;
    PbInfoReqFrame.cmd = 2;
    PbInfoReqFrame.args[0] = 0;
    PowerBoardSerial.SendFrame(PbInfoReqFrame);
}
