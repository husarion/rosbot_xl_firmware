/**
 * @file UartLib.cpp
 * @author Maciej Kurcius
 * @brief 
 * @version 0.1
 * @date 2022-02-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <UartLib.h>
#include <STM32FreeRTOS.h>

UartProtocolClass:: UartProtocolClass(uint32_t _rx, uint32_t _tx, uint32_t baudrate_, uint8_t config_)
    :
    HardwareSerial(_rx, _tx){
        this->begin(baudrate_, config_);
        this->setTimeout(DEFAULT_TIMEOUT);
    }


UartProtocolClass:: ~UartProtocolClass(){
    ;
}


void UartProtocolClass:: UartProtocolLoopHandler(){
    RxBufferSize = this->readBytes(RxBuffer, RX_BUFF_CAPACITY);
    if(RxBufferSize != 0){
        //Frame proceed start
        SetRedLed(On);  //for debug
        vTaskDelay(50);
        SetRedLed(Off);
    }
}

