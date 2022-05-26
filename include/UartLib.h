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
#include <STM32FreeRTOS.h>

#define FRAME_START_BIT                 '<'
#define FRAME_STOP_BIT                  '>'
#define MAX_ARGS_SIZE                   32
#define RX_BUFF_CAPACITY                500
#define DEFAULT_TIMEOUT                 1
#define UART_H2B_ERR                    0xFFFF
#define UART_FRAME_DATA_LENGTH(arg)     (arg + 3) * 2 
#define UART_FRAME_LENGTH(arg)          UART_FRAME_DATA_LENGTH(arg) + 2               
#define UART_MAX_FRAME_DATA_LENGTH      UART_FRAME_DATA_LENGTH(MAX_ARGS_SIZE)

//extern variables
extern QueueHandle_t BatteryStateQueue;

typedef enum{
    ConversionOk    = 0,
    ConversionError = 1
}UartConvStatusTypeDef;


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
    UartConvStatusTypeDef HexToByte(uint8_t* byte_, uint8_t* result);
    uint8_t* ByteToHex(uint8_t byte_, uint8_t* buffer_);
    uint8_t DecodeHex(uint8_t byte_);
    uint8_t EncodeHex(uint8_t byte_);
    void ExecuteFrame();
    void SendFrame(UartProtocolFrame frame_);
    void SendBuffer(uint8_t size_, uint8_t* buffer_);
    void SendBuffer(uint8_t size_, String buffer_);
    private:
    UartProtocolFrame ProcessedFrame;
    uint8_t RxBuffer[RX_BUFF_CAPACITY];
    volatile uint16_t RxBufferSize = 0;
};

#endif /* UartLib_H */