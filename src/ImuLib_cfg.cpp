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
    if(this->ImuBno->begin()){
        this->ImuBno->setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P1);
        this->ImuBno->setAxisSign(Adafruit_BNO055::REMAP_SIGN_P4);
        this->ImuBno->setExtCrystalUse(true);
        return false;
    }
    return true;
}

imu_queue_t ImuDriver::LoopHandler(){
    imu_queue_t ImuQueue;
    imu::Quaternion Quaternion;
    double* buffer = &(this->ImuBno->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER))[0];
    ImuQueue.LinearAcceleration[0] = (float)buffer[0];
    ImuQueue.LinearAcceleration[1] = (float)buffer[1];
    ImuQueue.LinearAcceleration[2] = (float)buffer[2];
    buffer = &(this->ImuBno->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE))[0];
    ImuQueue.AngularVelocity[0] = (float)buffer[0];
    ImuQueue.AngularVelocity[1] = (float)buffer[1];
    ImuQueue.AngularVelocity[2] = (float)buffer[2];
    Quaternion = this->ImuBno->getQuat();
    ImuQueue.Orientation[0] = Quaternion.x();
    ImuQueue.Orientation[1] = Quaternion.y();
    ImuQueue.Orientation[2] = Quaternion.z();
    ImuQueue.Orientation[3] = Quaternion.w();
    return ImuQueue;
}



