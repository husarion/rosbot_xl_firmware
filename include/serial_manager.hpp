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

#include <HardwareSerial.h>

#include "robot_config.hpp"

class SerialManager {
 public:
  static constexpr uint16_t CHECK_INTERVAL = 50;
  static constexpr uint16_t RESEND_READY_INTERVAL = 250;
  static constexpr size_t NS_MAX_LENGTH = 32;
  static inline constexpr char NS_DEFAULT[] = "";

  void init() {
    initSerial(SBC_SERIAL_CONFIG);
    initSerial(FTDI_SERIAL_CONFIG);
  }

  const SerialConfig& selectActive(uint32_t timeout_ms = 2000) {
    uint32_t startTime = millis();

    while ((millis() - startTime) < timeout_ms) {
      if (digitalRead(PUSH_BUTTON1) == LOW ||
          digitalRead(PUSH_BUTTON2) == LOW) {
        digitalWrite(GRN_LED, HIGH);
        digitalWrite(GRN_LED2, HIGH);
        active_ = &FTDI_SERIAL_CONFIG;
        return *active_;
      }
      delay(CHECK_INTERVAL);
    }

    active_ = &SBC_SERIAL_CONFIG;
    return *active_;
  }

  bool configureNamespace(uint16_t timeout_ms = 2000) {
    if (!active_) return false;

    // Try to get from host
    if (waitForHostConfig(timeout_ms)) {
      return true;
    }

    // Default namespace
    strncpy(namespace_, NS_DEFAULT, NS_MAX_LENGTH);
    namespace_[NS_MAX_LENGTH - 1] = '\0';
    return true;
  }

  // ============== Accessors ==============

  HardwareSerial& active() { return *active_->serial; }
  const SerialConfig& activeConfig() const { return *active_; }
  const char* getNamespace() const { return namespace_; }

  HardwareSerial& sbc() { return *SBC_SERIAL_CONFIG.serial; }
  HardwareSerial& ftdi() { return *FTDI_SERIAL_CONFIG.serial; }

  HardwareSerial& debug() {
    return (active_->serial == &Serial1) ? Serial3 : Serial1;
  }

 private:
  const SerialConfig* active_ = nullptr;
  char namespace_[NS_MAX_LENGTH] = {};

  // Flash storage
  static constexpr uint32_t FLASH_SECTOR = FLASH_SECTOR_11;
  static constexpr uint32_t FLASH_ADDR = 0x080E0000;
  static constexpr uint16_t FLASH_MAGIC = 0xCAFE;

  struct FlashStorage {
    uint16_t magic;
    uint16_t length;
    char ns[NS_MAX_LENGTH];
    uint32_t crc;
  } __attribute__((packed, aligned(4)));

  // ============== Private Methods ==============

  void initSerial(const SerialConfig& cfg) {
    cfg.serial->setRx(cfg.rxPin);
    cfg.serial->setTx(cfg.txPin);
    cfg.serial->begin(cfg.baudrate);
    cfg.serial->setTimeout(cfg.timeout_ms);
  }

  bool waitForHostConfig(uint32_t timeout_ms) {
    HardwareSerial& serial = active();
    uint32_t start_time = millis();

    char buffer[NS_MAX_LENGTH] = {0};
    size_t idx = 0;
    bool got_line = false;
    uint32_t last_ready = 0;

    serial.println("READY");
    serial.flush();
    last_ready = millis();

    while (millis() - start_time < timeout_ms && !got_line) {
      while (serial.available()) {
        char c = serial.read();
        if (c == '\n') {
          got_line = true;
          break;
        }
        if (idx < NS_MAX_LENGTH - 1) buffer[idx++] = c;
      }

      if (millis() - last_ready >= RESEND_READY_INTERVAL) {
        serial.println("READY");
        serial.flush();
        last_ready = millis();
      }
    }

    if (got_line && idx > 0 && strncmp(buffer, "NS:", 3) == 0) {
      strncpy(namespace_, buffer + 3, NS_MAX_LENGTH);
      serial.println("ACK");
      serial.flush();
      return true;
    }

    return false;
  }

};

extern SerialManager serialManager;
