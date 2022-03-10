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

UartProtocolClass:: UartProtocolClass(uint32_t _rx, uint32_t _tx, uint32_t baudrate_, uint8_t config_)
    :
    HardwareSerial(_rx, _tx){
        this->begin(baudrate_, config_);
    }


UartProtocolClass:: ~UartProtocolClass(){
    ;
}