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

#include "sensors/imu.hpp"

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#if defined(ROSBOT)
#define IMU_AXIS_CONFIG Adafruit_BNO055::REMAP_CONFIG_P0
#elif defined(ROSBOT_XL)
#define IMU_AXIS_CONFIG Adafruit_BNO055::REMAP_CONFIG_P1
#endif
#define DEGREESPERSEC_TO_RADPERSEC 0.017453293

bool ImuDriver::init(uint8_t id, uint8_t addr) {
  bus_->begin();
  bus_->setClock(400000);  // 400 kHz fast_mode / 100 kHz robust_mode

  this->imuBno_ = new Adafruit_BNO055(id, addr, bus_);
  if (!this->imuBno_->begin(OPERATION_MODE_NDOF)) {
    return false;
  }

  imuBno_->setAxisRemap(IMU_AXIS_CONFIG);
  imuBno_->setAxisSign(Adafruit_BNO055::REMAP_SIGN_P4);
  imuBno_->setExtCrystalUse(true);

  return true;
}

void ImuDriver::update() {
  // Acceleration
  imu::Vector<3> accel =
      imuBno_->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  data_.acceleration[0] = accel.x();
  data_.acceleration[1] = accel.y();
  data_.acceleration[2] = accel.z();

  // Gyroscope
  imu::Vector<3> gyro = imuBno_->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  data_.angular_velocity[0] = gyro.x() * DEGREESPERSEC_TO_RADPERSEC;
  data_.angular_velocity[1] = gyro.y() * DEGREESPERSEC_TO_RADPERSEC;
  data_.angular_velocity[2] = gyro.z() * DEGREESPERSEC_TO_RADPERSEC;

  // Orientation (quaternion)
  imu::Quaternion q = imuBno_->getQuat();
  data_.orientation[0] = q.x();
  data_.orientation[1] = q.y();
  data_.orientation[2] = q.z();
  data_.orientation[3] = q.w();
}
