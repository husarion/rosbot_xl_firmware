/**
 * @file imu.hpp
 * @author Maciej Kurcius
 * @brief
 * @version 0.1
 * @date 2022-02-15
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef ImuLibCfg_H
#define ImuLibCfg_H

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <hardware_cfg.h>

typedef struct {
  float Orientation[4];
  float AngularVelocity[3];
  float LinearAcceleration[3];
} imu_data_t;

class ImuDriver {
 public:
  ImuDriver(uint8_t ImuId_, uint8_t ImuAddr_, TwoWire* ImuWire_);
  ~ImuDriver();
  bool Init();
  imu_data_t loopHandler();

 private:
  Adafruit_BNO055* ImuBno;
};

extern ImuDriver imuDriver;

#endif /* ImuLibCfg_H */
