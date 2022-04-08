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
    this->RxBufferSize = this->readBytes(RxBuffer, RX_BUFF_CAPACITY);
    static uint8_t marker = 0;
    marker ++;
    if(this->RxBufferSize != 0){
        this->StreamParse();
    }
    UartProtocolFrame Frame;
    Frame.Cmd = 1;
    Frame.ArgSize = 2;
    Frame.Arg[0] = 12;
    Frame.Arg[1] = 56;
    this->SendFrame(Frame);
    // if(marker > 5 && RxBufferSize != 0){
    //     if(this->StreamParse() == -1)
    //         return;
    //     SetRedLed(On);  //for debug
    //     vTaskDelay(50);
    //     SetRedLed(Off);
    // }
}

int8_t UartProtocolClass:: StreamParse(){
    uint16_t FrameDataPtr;
    uint8_t CountedCheckSum;
    if(RxBufferSize >= RX_BUFF_CAPACITY)    
        return -1;
    for(uint16_t i = 0; i < RxBufferSize || i < RX_BUFF_CAPACITY; i++){
        if(RxBuffer[i] == '<'){
            CountedCheckSum = 0;
            if(HexToByte(&RxBuffer[i+1], &this->ProcessedFrame.Cmd) == ConversionError) 
                return -1;
            CountedCheckSum ^= this->ProcessedFrame.Cmd;
            if(HexToByte(&RxBuffer[i+3], &this->ProcessedFrame.ArgSize) == ConversionError)
                return -1;
            if(this->ProcessedFrame.ArgSize > MAX_ARGS_SIZE)
                return -1;
            CountedCheckSum ^= this->ProcessedFrame.ArgSize;
            FrameDataPtr = i+5; 
            if(HexToByte(&RxBuffer[i + this->ProcessedFrame.ArgSize*2 + 5], &ProcessedFrame.CheckSum) == ConversionError) //1 byte of '<' + 2 bytes (CMD) + 2 bytes (ARG_SIZE)
                return -1;
            for(uint8_t j = 0; j < this->ProcessedFrame.ArgSize; j++){
                if(HexToByte(&RxBuffer[FrameDataPtr + j*2], &this->ProcessedFrame.Arg[j]) == ConversionError)
                    return -1;
                CountedCheckSum ^= this->ProcessedFrame.Arg[j];
            }
            if(CountedCheckSum == this->ProcessedFrame.CheckSum){
                this->ExecuteFrame();
                i = i + this->ProcessedFrame.ArgSize*2 + 7;//1 byte of '<' + 2 bytes (CMD) + 2 bytes (ARG_SIZE) 2 bytes of CRC + 1 byte of '>' -1 (i++ in for loop)
            }
        }
    }
    return 0;
}

UartConvStatusTypeDef UartProtocolClass:: HexToByte(uint8_t* byte_, uint8_t* result_){
    uint8_t value, result;
    value = DecodeHex(byte_[0]);
    if(value < 16){
        *result_ = value << 4;
        value = DecodeHex(byte_[1]);
        if(value < 16){
            *result_ = *result_ + value;
            return ConversionOk;
        }
        return ConversionError;
    }
    return ConversionError;
}

uint8_t* UartProtocolClass:: ByteToHex(uint8_t byte_, uint8_t* buffer_){
    buffer_[0] = EncodeHex((byte_ & 0xF0) >> 4);
    buffer_[1] = EncodeHex(byte_ & 0x0F);
    return buffer_ +2;
}

uint8_t UartProtocolClass:: DecodeHex(uint8_t byte_){
    if(byte_ >= '0' && byte_ <= '9')
        return byte_ - '0';
    else if(byte_ >= 'a' && byte_ <= 'f')
        return byte_ - 'a' + 10;
    else if(byte_ >= 'A' && byte_ <= 'F')
        return byte_ -'A' + 10;
    else
        return 0xFF;
}

uint8_t UartProtocolClass:: EncodeHex(uint8_t byte_){
    if(byte_ < 10)
        return '0' + byte_;
    else if(byte_ < 16)
        return 'a' + byte_ - 10;
    else
        return 0xFF;
}

void UartProtocolClass:: ExecuteFrame(){
    if(this->ProcessedFrame.Cmd > 1)
        ;//send empty frame with the same cmd to confirm receiving
    uint8_t temp = 0;
    switch(this->ProcessedFrame.Cmd){
    case 0:
        ;
        break;
    case 1: // Battery State
        if(this->ProcessedFrame.ArgSize < BATTERY_STATE_MSG_LENGTH)
            break;
        battery_state_queue_t BatteryState;
        for(uint8_t i = 0; i < 17; i++){
            temp = this->ProcessedFrame.Arg[i];
        }
        BatteryState.Voltage = this->ProcessedFrame.Arg[0]  | this->ProcessedFrame.Arg[1] << 8;
        BatteryState.Temperature = this->ProcessedFrame.Arg[2] << 8 | this->ProcessedFrame.Arg[3];
        BatteryState.Current = this->ProcessedFrame.Arg[4] << 8 | this->ProcessedFrame.Arg[5];
        BatteryState.ChargeCurrent = this->ProcessedFrame.Arg[6] << 8 | this->ProcessedFrame.Arg[7];
        BatteryState.Capacity = this->ProcessedFrame.Arg[8] << 8 | this->ProcessedFrame.Arg[9];
        BatteryState.DesignCapacity = this->ProcessedFrame.Arg[10] << 8 | this->ProcessedFrame.Arg[11];
        BatteryState.Percentage = this->ProcessedFrame.Arg[12];
        BatteryState.Status = (BatteryStatusTypeDef)this->ProcessedFrame.Arg[13];
        BatteryState.Health = (BatteryHealthTypeDef)this->ProcessedFrame.Arg[14];
        BatteryState.Technology = (BatteryTechnologyTypeDef)this->ProcessedFrame.Arg[15];
        BatteryState.Present = this->ProcessedFrame.Arg[16];
        xQueueSendToFront(BatteryStateQueue, (void*) &BatteryState, (TickType_t) 0);
        break;
    case 2:
        ;// SetGreenLed(Toggle);
        break;
    }
}

void UartProtocolClass:: SendFrame(UartProtocolFrame frame_){
    static uint8_t TxBuff[UART_FRAME_LENGTH(MAX_ARGS_SIZE)];
    uint8_t* TxBufPtr = TxBuff;
    frame_.CheckSum = frame_.Cmd ^ frame_.ArgSize;
    *TxBufPtr++ = FRAME_START_BIT;
    TxBufPtr = ByteToHex(frame_.Cmd, TxBufPtr);
    TxBufPtr = ByteToHex(frame_.ArgSize, TxBufPtr);
    for(uint8_t i = 0; i < frame_.ArgSize; i++){
        TxBufPtr = ByteToHex(frame_.Arg[i], TxBufPtr);
        frame_.CheckSum ^= frame_.Arg[i];
    }
    TxBufPtr = ByteToHex(frame_.CheckSum, TxBufPtr);
    *TxBufPtr++ = FRAME_STOP_BIT;
    this->SendBuffer(UART_FRAME_LENGTH(frame_.ArgSize), (char*)TxBuff);
}

void UartProtocolClass:: SendBuffer(uint8_t size_, uint8_t* buffer_){
    String Input = (char*) buffer_;
    String TxData = Input.substring(0, size_);
    this->print (TxData);
}

void UartProtocolClass:: SendBuffer(uint8_t size_, String buffer_){
    this->print (buffer_.substring(0, size_));
}
