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

HardwareSerial SBC_SERIAL(SBC_SERIAL_RX, SBC_SERIAL_TX);
HardwareSerial PWR_BRD_SERIAL(PWR_BRD_SERIAL_RX, PWR_BRD_SERIAL_TX);
#if EXT_SERIAL_EN_FLAG == 1
    HardwareSerial EXT_SERIAL(EXT_SERIAL_RX, EXT_SERIAL_TX);
#endif


void BoardGpioInit(void){
    pinMode(GRN_LED, OUTPUT);
    digitalWrite(GRN_LED, LOW);
    pinMode(EN_LOC_5V, OUTPUT);
    digitalWrite(EN_LOC_5V, LOW);
    pinMode(RD_LED, OUTPUT);
    digitalWrite(RD_LED, LOW);
    pinMode(PWR_BRD_GPIO_OUTPUT, OUTPUT);
    digitalWrite(PWR_BRD_GPIO_OUTPUT, LOW);
    pinMode(PWR_BRD_GPIO_INPUT, INPUT_PULLUP);
}

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
    //SBC Serial port init
    SBC_SERIAL.begin(SBC_SERIAL_BAUDRATE);
    //Power Board Serial port init
    PWR_BRD_SERIAL.begin(PWR_BRD_SERIAL_BAUDRATE);
    //External Serial port init
    #if EXT_SERIAL_EN_FLAG == 1
        EXT_SERIAL.begin(EXT_SERIAL_BAUDRATE);
    #endif
    while(1) {
        SBC_SERIAL.println("Hello SBC");
        PWR_BRD_SERIAL.println("Hello Power Board");
        #if EXT_SERIAL_EN_FLAG == 1 
            EXT_SERIAL.println("Hello external device");
        #endif
        SetGreenLed(Toggle);
        delay(1000);
    }
}


