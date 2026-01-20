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

#pragma once

typedef enum { fw_normal = 0, fw_error = 1, fw_debug = 2 } FirmwareModeTypeDef;

/* CHOOSE HARDWARE CONFIG */
#if defined(ROSBOT_XL)
#include "rosbot_xl/hardware_cfg.hpp"
#elif defined(ROSBOT)
#include "rosbot/config.hpp"
#include "rosbot/hardware_cfg.hpp"
#else
#error "No board version defined! Did you set correct flag in platformio.ini?"
#endif

static_assert(motors::CONFIG.size() == static_cast<size_t>(MotorID::COUNT),
              "Motor config does not match MotorID count");
