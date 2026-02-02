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

#include "bsp.hpp"

#if defined(ROSBOT)
#define IMU_AXIS_CONFIG Adafruit_BNO055::REMAP_CONFIG_P0
#elif defined(ROSBOT_XL)
#define IMU_AXIS_CONFIG Adafruit_BNO055::REMAP_CONFIG_P1
#endif
#define DEGREESPERSEC_TO_RADPERSEC 0.017453293

bool ImuDriver::init(uint8_t id, uint8_t addr, TwoWire* wire) {
  this->imuBno_ = new Adafruit_BNO055(id, addr, wire);
  if (!this->imuBno_->begin(OPERATION_MODE_NDOF)) {
    return false;
  }

  imuBno_->setAxisRemap(IMU_AXIS_CONFIG);
  imuBno_->setAxisSign(Adafruit_BNO055::REMAP_SIGN_P4);
  imuBno_->setExtCrystalUse(true);

  delay(10);
  return true;
}

imu_data_t ImuDriver::loopHandler() {
  this->imuBno_->getEvent(&event_);
  imu_data_t data;

  // Acceleration
  imu::Vector<3> accel =
      imuBno_->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  data.acceleration[0] = accel.x();
  data.acceleration[1] = accel.y();
  data.acceleration[2] = accel.z();

  // Gyroscope
  imu::Vector<3> gyro = imuBno_->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  data.angular_velocity[0] = gyro.x() * DEGREESPERSEC_TO_RADPERSEC;
  data.angular_velocity[1] = gyro.y() * DEGREESPERSEC_TO_RADPERSEC;
  data.angular_velocity[2] = gyro.z() * DEGREESPERSEC_TO_RADPERSEC;

  // Orientation (quaternion)
  imu::Quaternion q = imuBno_->getQuat();
  data.orientation[0] = q.x();
  data.orientation[1] = q.y();
  data.orientation[2] = q.z();
  data.orientation[3] = q.w();

  return data;
}
