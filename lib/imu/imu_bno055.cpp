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

#include "imu_bno055.hpp"

#include <wiring_constants.h>

ImuBno055::ImuBno055(const ImuBno055Config& cfg)
    : cfg_(cfg), bno_(cfg.sensor_id, cfg.i2c_addr, cfg.bus) {}

bool ImuBno055::init() {
  if (!bno_.begin(OPERATION_MODE_NDOF)) {
    return false;
  }

  bno_.setAxisRemap(cfg_.axis_config);
  bno_.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P4);
  bno_.setExtCrystalUse(true);

  return true;
}

void ImuBno055::update() {
  imu::Vector<3> accel = bno_.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  data_.acceleration[0] = accel.x();
  data_.acceleration[1] = accel.y();
  data_.acceleration[2] = accel.z();

  imu::Vector<3> gyro = bno_.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  data_.angular_velocity[0] = gyro.x() * DEG_TO_RAD;
  data_.angular_velocity[1] = gyro.y() * DEG_TO_RAD;
  data_.angular_velocity[2] = gyro.z() * DEG_TO_RAD;

  imu::Quaternion q = bno_.getQuat();
  data_.orientation[0] = q.x();
  data_.orientation[1] = q.y();
  data_.orientation[2] = q.z();
  data_.orientation[3] = q.w();
}
