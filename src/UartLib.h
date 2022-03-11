/**
 * @file UartLib.h
 * @author Maciej Kurcius
 * @brief 
 * @version 0.1
 * @date 2022-02-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef UartLib_H
#define UartLib_H

#include <Arduino.h>
#include <bsp.h>

#define MAX_ARGS_SIZE       32
#define RX_BUFF_CAPACITY    500
#define DEFAULT_TIMEOUT     1


struct UartProtocolFrame{
    uint8_t cmd;
    uint8_t arg_size;
    uint8_t args[MAX_ARGS_SIZE];
};

class UartProtocolClass: public HardwareSerial{
    public:
    UartProtocolClass(uint32_t _rx, uint32_t _tx, uint32_t baudrate_, uint8_t config_);
    ~UartProtocolClass();
    void UartProtocolLoopHandler();

    private:
    UartProtocolFrame ProcessedFrame;
    uint8_t RxBuffer[RX_BUFF_CAPACITY];
    volatile uint16_t RxBufferSize = 0;
};

#endif /* UartLib_H */