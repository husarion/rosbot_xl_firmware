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
#define UART_H2B_ERR        0xFFFF


struct UartProtocolFrame{
    uint8_t Cmd;
    uint8_t ArgSize;
    uint8_t Arg[MAX_ARGS_SIZE];
    uint8_t CheckSum;
};

class UartProtocolClass: public HardwareSerial{
    public:
    UartProtocolClass(uint32_t _rx, uint32_t _tx, uint32_t baudrate_, uint8_t config_);
    ~UartProtocolClass();
    void UartProtocolLoopHandler();
    int8_t StreamParse();
    uint16_t HexToByte(uint8_t* byte_);
    uint16_t ByteToHex(uint8_t* byte_);
    uint8_t DecodeHex(uint8_t byte_);
    uint8_t EncodeHex(uint8_t byte_);
    void ExecuteFrame();
    void SendFrame(uint8_t cmd_, uint8_t arg_size_, uint8_t* arg);

    private:
    UartProtocolFrame ProcessedFrame;
    uint8_t RxBuffer[RX_BUFF_CAPACITY];
    volatile uint16_t RxBufferSize = 0;
};

#endif /* UartLib_H */