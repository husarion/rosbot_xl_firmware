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

UartProtocolClass:: UartProtocolClass(uint32_t arg_rx, uint32_t arg_tx, uint32_t arg_baudrate, uint8_t arg_config)
    :
    HardwareSerial(arg_rx, arg_tx){
        this->begin(arg_baudrate, arg_config);
        this->setTimeout(DEFAULT_TIMEOUT);
    }

UartProtocolClass:: ~UartProtocolClass(){
    ;
}

void UartProtocolClass:: UartProtocolLoopHandler(){
    this->rx_buffer_size_ = this->readBytes(rx_buffer_, RX_BUFF_CAPACITY);
    static uint8_t marker = 0;
    marker ++;
    if(this->rx_buffer_size_ != 0){
        this->StreamParse();
    }
    UartProtocolFrame frame;
    frame.cmd = 1;
    frame.arg_size = 2;
    frame.args[0] = 12;
    frame.args[1] = 56;
    this->SendFrame(frame);
}

int8_t UartProtocolClass:: StreamParse(){
    uint16_t frame_data_ptr;
    uint8_t counted_check_sum;
    if(rx_buffer_size_ >= RX_BUFF_CAPACITY)    
        return -1;
    for(uint16_t i = 0; i < rx_buffer_size_ || i < RX_BUFF_CAPACITY; i++){
        if(rx_buffer_[i] == '<'){
            counted_check_sum = 0;
            if(HexToByte(&rx_buffer_[i+1], &this->processed_frame_.cmd) == ConversionError) 
                return -1;
            counted_check_sum ^= this->processed_frame_.cmd;
            if(HexToByte(&rx_buffer_[i+3], &this->processed_frame_.arg_size) == ConversionError)
                return -1;
            if(this->processed_frame_.arg_size > MAX_ARGS_SIZE)
                return -1;
            counted_check_sum ^= this->processed_frame_.arg_size;
            frame_data_ptr = i+5; 
            if(HexToByte(&rx_buffer_[i + this->processed_frame_.arg_size * 2 + 5], &processed_frame_.check_sum) == ConversionError) //1 byte of '<' + 2 bytes (CMD) + 2 bytes (ARG_SIZE)
                return -1;
            for(uint8_t j = 0; j < this->processed_frame_.arg_size; j++){
                if(HexToByte(&rx_buffer_[frame_data_ptr + j * 2], &this->processed_frame_.args[j]) == ConversionError)
                    return -1;
                counted_check_sum ^= this->processed_frame_.args[j];
            }
            if(counted_check_sum == this->processed_frame_.check_sum){
                this->ExecuteFrame();
                i = i + this->processed_frame_.arg_size * 2 + 7;//1 byte of '<' + 2 bytes (CMD) + 2 bytes (ARG_SIZE) 2 bytes of CRC + 1 byte of '>' -1 (i++ in for loop)
            }
        }
    }
    return 0;
}

UartConvStatusTypeDef UartProtocolClass:: HexToByte(uint8_t* arg_byte, uint8_t* arg_result){
    uint8_t value;
    value = DecodeHex(arg_byte[0]);
    if(value < 16){
        *arg_result = value << 4;
        value = DecodeHex(arg_byte[1]);
        if(value < 16){
            *arg_result = *arg_result + value;
            return ConversionOk;
        }
        return ConversionError;
    }
    return ConversionError;
}

uint8_t* UartProtocolClass:: ByteToHex(uint8_t arg_byte, uint8_t* arg_buffer){
    arg_buffer[0] = EncodeHex((arg_byte & 0xF0) >> 4);
    arg_buffer[1] = EncodeHex(arg_byte & 0x0F);
    return arg_buffer + 2;
}

uint8_t UartProtocolClass:: DecodeHex(uint8_t arg_byte){
    if(arg_byte >= '0' && arg_byte <= '9')
        return arg_byte - '0';
    else if(arg_byte >= 'a' && arg_byte <= 'f')
        return arg_byte - 'a' + 10;
    else if(arg_byte >= 'A' && arg_byte <= 'F')
        return arg_byte -'A' + 10;
    else
        return 0xFF;
}

uint8_t UartProtocolClass:: EncodeHex(uint8_t arg_byte){
    if(arg_byte < 10)
        return '0' + arg_byte;
    else if(arg_byte < 16)
        return 'a' + arg_byte - 10;
    else
        return 0xFF;
}

void UartProtocolClass:: ExecuteFrame(){
    uint8_t frame_arg = 0;
    if(this->processed_frame_.cmd > 1)
        ;//send empty frame with the same cmd to confirm receiving
    switch(this->processed_frame_.cmd){
    case 0:
        break;
    case 1: // Battery State
        if(this->processed_frame_.arg_size< BATTERY_STATE_MSG_LENGTH){
            break;
        }
        battery_state_queue_t battery_state;
        for(uint8_t i = 0; i < 17; i++){
            frame_arg = this->processed_frame_.args[i];
        }
        battery_state.voltage = this->processed_frame_.args[0] << 8 | this->processed_frame_.args[1];
        battery_state.temperature = this->processed_frame_.args[2] << 8 | this->processed_frame_.args[3];
        battery_state.current = this->processed_frame_.args[4] << 8 | this->processed_frame_.args[5];
        battery_state.charge_current = this->processed_frame_.args[6] << 8 | this->processed_frame_.args[7];
        battery_state.capacity = this->processed_frame_.args[8] << 8 | this->processed_frame_.args[9];
        battery_state.design_capacity = this->processed_frame_.args[10] << 8 | this->processed_frame_.args[11];
        battery_state.percentage = this->processed_frame_.args[12];
        battery_state.status = (BatteryStatusTypeDef)this->processed_frame_.args[13];
        battery_state.health = (BatteryHealthTypeDef)this->processed_frame_.args[14];
        battery_state.technology = (BatteryTechnologyTypeDef)this->processed_frame_.args[15];
        battery_state.present = this->processed_frame_.args[16];
        xQueueSendToFront(BatteryStateQueue, (void*) &battery_state, (TickType_t) 0);
        break;
    case 2:
        break;
    default:
        break;
    }
}

void UartProtocolClass:: SendFrame(UartProtocolFrame arg_frame){
    static uint8_t tx_buff[UART_FRAME_LENGTH(MAX_ARGS_SIZE)];
    uint8_t* tx_buff_ptr = tx_buff;
    arg_frame.check_sum = arg_frame.cmd ^ arg_frame.arg_size;
    *tx_buff_ptr++ = FRAME_START_BIT;
    tx_buff_ptr = ByteToHex(arg_frame.cmd, tx_buff_ptr);
    tx_buff_ptr = ByteToHex(arg_frame.arg_size, tx_buff_ptr);
    for(uint8_t i = 0; i < arg_frame.arg_size; i++){
        tx_buff_ptr = ByteToHex(arg_frame.args[i], tx_buff_ptr);
        arg_frame.check_sum ^= arg_frame.args[i];
    }
    tx_buff_ptr = ByteToHex(arg_frame.check_sum, tx_buff_ptr);
    *tx_buff_ptr++ = FRAME_STOP_BIT;
    this->SendBuffer(UART_FRAME_LENGTH(arg_frame.arg_size), (char*)tx_buff_ptr);
}

void UartProtocolClass:: SendBuffer(uint8_t arg_size, uint8_t* arg_buffer){
    String input = (char*) arg_buffer;
    String tx_data = input.substring(0, arg_size);
    this->print (tx_data);
}

void UartProtocolClass:: SendBuffer(uint8_t arg_size, String arg_buffer){
    this->print (arg_buffer.substring(0, arg_size));
}
