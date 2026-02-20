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

#include <functional>

#include "serial.hpp"

class SerialManager {
 public:
  static constexpr uint16_t CHECK_INTERVAL = 50;
  static constexpr uint16_t RESEND_READY_INTERVAL = 250;
  static constexpr size_t NS_MAX_LENGTH = 32;
  static inline constexpr char NS_DEFAULT[] = "";

  SerialManager(SerialConfig main, const SerialConfig* alt = nullptr,
                std::function<bool()> useAltCondition = nullptr,
                std::function<void()> confirmAlt = nullptr)
      : main_(main),
        alt_(alt),
        useAltCondition_(useAltCondition),
        confirmAlt_(confirmAlt) {}

  void init() {
    initSerial(main_);
    if (alt_) initSerial(*alt_);
  }

  const SerialConfig& selectActive(uint32_t timeout_ms = 2000) {
    uint32_t startTime = millis();

    while ((millis() - startTime) < timeout_ms) {
      if (alt_ && useAltCondition_ && useAltCondition_()) {
        active_ = alt_;
        if (confirmAlt_) confirmAlt_();
        return *active_;
      }
      delay(CHECK_INTERVAL);
    }

    active_ = &main_;
    return *active_;
  }

  bool configureNamespace(uint16_t timeout_ms = 2000) {
    if (!active_) return false;

    if (waitForHostConfig(timeout_ms)) {
      return true;
    }

    strncpy(namespace_, NS_DEFAULT, NS_MAX_LENGTH);
    namespace_[NS_MAX_LENGTH - 1] = '\0';
    return true;
  }

  // ============== Accessors ==============

  HardwareSerial& active() { return *active_->serial; }
  const SerialConfig& activeConfig() const { return *active_; }
  const char* getNamespace() const { return namespace_; }

  HardwareSerial& main() { return *main_.serial; }
  HardwareSerial& alt() { return *alt_->serial; }

  HardwareSerial& debug() {
    return (active_->serial == main_.serial) ? *alt_->serial : *main_.serial;
  }

 private:
  const SerialConfig main_;
  const SerialConfig* alt_ = nullptr;
  const SerialConfig* active_ = nullptr;
  char namespace_[NS_MAX_LENGTH] = {};

  std::function<bool()> useAltCondition_ = nullptr;
  std::function<void()> confirmAlt_ = nullptr;

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
