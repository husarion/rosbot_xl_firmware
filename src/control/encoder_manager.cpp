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

#include "control/encoders_manager.hpp"
#include "robot_config.hpp"

using namespace control;

void EncoderManager::init() {
  MotorID m;
  uint8_t m_idx;

  m = MotorID::FR;
  m_idx = static_cast<uint8_t>(m);
  encoders_[m_idx].init(getEncAPin(m), getEncBPin(m), getEncoderTimer(m),
                        getDirection(m), RobotParams::RAD_PER_TICK);

  m = MotorID::FL;
  m_idx = static_cast<uint8_t>(m);
  encoders_[m_idx].init(getEncAPin(m), getEncBPin(m), getEncoderTimer(m),
                        getDirection(m), RobotParams::RAD_PER_TICK);

  m = MotorID::RR;
  m_idx = static_cast<uint8_t>(m);
  encoders_[m_idx].init(getEncAPin(m), getEncBPin(m), getEncoderTimer(m),
                        getDirection(m), RobotParams::RAD_PER_TICK);

  m = MotorID::RL;
  m_idx = static_cast<uint8_t>(m);
  encoders_[m_idx].init(getEncAPin(m), getEncBPin(m), getEncoderTimer(m),
                        getDirection(m), RobotParams::RAD_PER_TICK);
}

void EncoderManager::update() {
  for (uint8_t i = 0; i < NUM_ENCODERS; ++i) {
    encoders_[i].update();
      data_.position[i] = encoders_[i].getPosition();
      data_.velocity[i] = encoders_[i].getVelocity();
      // data_.effort[i] = 0.0f;  // Not implemented
  }
}
