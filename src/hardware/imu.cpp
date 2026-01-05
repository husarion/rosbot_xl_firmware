// Copyright 2022 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hardware/imu.hpp"

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define DEGREESPERSEC_TO_RADPERSEC 0.017453293

extern TwoWire I2cBus;

ImuDriver imuDriver(BNO055_ID, BNO055_ADDRESS_B, &I2cBus);

ImuDriver::ImuDriver(uint8_t ImuId, uint8_t ImuAddr, TwoWire* ImuWire) {
  this->imuBno = new Adafruit_BNO055(ImuId, ImuAddr, ImuWire);
}

ImuDriver::~ImuDriver() {}

bool ImuDriver::init() {
  if (!this->imuBno->begin(OPERATION_MODE_NDOF)) {
    return false;
  }

  imuBno->setAxisRemap(ROBOT_IMU_AXIS_CONFIG);
  imuBno->setAxisSign(Adafruit_BNO055::REMAP_SIGN_P4);
  imuBno->setExtCrystalUse(true);

  delay(10);
  return true;
}

imu_data_t ImuDriver::loopHandler() {
  this->imuBno->getEvent(&this->event);
  imu_data_t data;

  // Acceleration
  imu::Vector<3> accel =
      imuBno->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  data.acceleration[0] = accel.x();
  data.acceleration[1] = accel.y();
  data.acceleration[2] = accel.z();

  // Gyroscope
  imu::Vector<3> gyro = imuBno->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  data.angular_velocity[0] = gyro.x() * DEGREESPERSEC_TO_RADPERSEC;
  data.angular_velocity[1] = gyro.y() * DEGREESPERSEC_TO_RADPERSEC;
  data.angular_velocity[2] = gyro.z() * DEGREESPERSEC_TO_RADPERSEC;

  // Orientation (quaternion)
  imu::Quaternion q = imuBno->getQuat();
  data.orientation[0] = q.x();
  data.orientation[1] = q.y();
  data.orientation[2] = q.z();
  data.orientation[3] = q.w();

  return data;
}
