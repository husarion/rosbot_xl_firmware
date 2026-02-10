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
#include <Arduino.h>

#include "serial_manager.hpp"

typedef enum {
  LOG_LEVEL_DEBUG,
  LOG_LEVEL_INFO,
  LOG_LEVEL_WARN,
  LOG_LEVEL_ERROR
} Log_level_t;

extern Log_level_t firmware_log_level;

#define LOG_DEBUG(...)                                                     \
  do {                                                                     \
    if (firmware_log_level <= LOG_LEVEL_DEBUG) {                           \
      serialManager.debug().printf("[DEBUG][%s:%d] ", __FILE__, __LINE__); \
      serialManager.debug().printf(__VA_ARGS__);                           \
      serialManager.debug().printf("\r\n");                                \
    }                                                                      \
  } while (0);

#define LOG_INFO(...)                                                     \
  do {                                                                    \
    if (firmware_log_level <= LOG_LEVEL_INFO) {                           \
      serialManager.debug().printf("[INFO][%s:%d] ", __FILE__, __LINE__); \
      serialManager.debug().printf(__VA_ARGS__);                          \
      serialManager.debug().printf("\r\n");                               \
    }                                                                     \
  } while (0)

#define LOG_WARN(...)                                                     \
  do {                                                                    \
    if (firmware_log_level <= LOG_LEVEL_WARN) {                           \
      serialManager.debug().printf("[WARN][%s:%d] ", __FILE__, __LINE__); \
      serialManager.debug().printf(__VA_ARGS__);                          \
      serialManager.debug().printf("\r\n");                               \
    }                                                                     \
  } while (0)

#define LOG_ERROR(...)                                                     \
  do {                                                                     \
    if (firmware_log_level <= LOG_LEVEL_ERROR) {                           \
      serialManager.debug().printf("[ERROR][%s:%d] ", __FILE__, __LINE__); \
      serialManager.debug().printf(__VA_ARGS__);                           \
      serialManager.debug().printf("\r\n");                                \
    }                                                                      \
  } while (0)
