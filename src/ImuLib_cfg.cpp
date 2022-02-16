/**
 * @file ImuLib_cfg.cpp
 * @author Maciej Kurcius
 * @brief 
 * @version 0.1
 * @date 2022-02-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "ImuLib_cfg.h"

TwoWire ImuWire(IMU_SDA, IMU_SCL);
ImuDriver ImuBno(IMU_ID, IMU_ADDR_B, &ImuWire);

ImuDriver::ImuDriver(uint8_t ImuId_, uint8_t ImuAddr_, TwoWire* ImuWire_){
    this->ImuBno = new Adafruit_BNO055(ImuId_, ImuAddr_, ImuWire_);
}

ImuDriver::~ImuDriver(){
    ;
}

bool ImuDriver::Init(){
    return this->ImuBno->begin();
}

imu_queue_t ImuDriver::LoopHandler(){
    imu_queue_t ImuQueue;
    double* buffer = &(this->ImuBno->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER))[0];
    ImuQueue.LinearAcceleration[0] = buffer[0];
    ImuQueue.LinearAcceleration[1] = buffer[1];
    ImuQueue.LinearAcceleration[2] = buffer[2];
    buffer = &(this->ImuBno->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE))[0];
    ImuQueue.AngularVelocity[0] = buffer[0];
    ImuQueue.AngularVelocity[1] = buffer[1];
    ImuQueue.AngularVelocity[2] = buffer[2];
    
    return ImuQueue;
}



