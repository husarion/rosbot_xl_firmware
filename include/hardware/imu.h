/**
 * @file ImuLib_cfg.h
 * @author Maciej Kurcius
 * @brief
 * @version 0.1
 * @date 2022-02-15
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <hardware_cfg.h>

typedef struct
{
  float Orientation[4];        // quaternion: x y z w
  float AngularVelocity[3];    // rad/s
  float LinearAcceleration[3]; // m/s^2
} imu_data_t;

class ImuDriver
{
public:
  ImuDriver(uint8_t ImuId_, uint8_t ImuAddr_, TwoWire * ImuWire_);
  ~ImuDriver();
  bool Init();
  imu_data_t LoopHandler();

private:
  Adafruit_BNO055 * ImuBno;
};

extern ImuDriver imuDriver;
