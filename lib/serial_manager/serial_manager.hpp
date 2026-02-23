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

struct SerialConfig {
  HardwareSerial* serial;
  uint32_t baudrate;
  uint8_t rxPin;
  uint8_t txPin;
  uint32_t timeout_ms;
  const char* name;
};

struct SerialManagerConfig {
  SerialConfig main;
  const SerialConfig* alt = nullptr;
  std::function<bool()> useAltCondition = nullptr;
  std::function<void()> confirmAlt = nullptr;
  uint16_t check_interval = 50;
  uint16_t resend_ready_interval = 250;
  const char* ns_default = "";
};

class SerialManager {
 public:
  static constexpr size_t NS_MAX_LENGTH = 32;

  SerialManager(SerialManagerConfig cfg) : cfg_(cfg) {}

  void init() {
    initSerial(cfg_.main);
    if (cfg_.alt) initSerial(*cfg_.alt);
  }

  const SerialConfig& selectCommunicationSerial(uint32_t timeout_ms = 2000) {
    uint32_t startTime = millis();

    while ((millis() - startTime) < timeout_ms) {
      if (cfg_.alt && cfg_.useAltCondition && cfg_.useAltCondition()) {
        active_ = cfg_.alt;
        if (cfg_.confirmAlt) cfg_.confirmAlt();
        return *active_;
      }
      delay(cfg_.check_interval);
    }

    active_ = &cfg_.main;
    return *active_;
  }

  bool configureNamespace(uint16_t timeout_ms = 2000) {
    if (!active_) return false;

    if (waitForHostConfig(timeout_ms)) {
      return true;
    }

    strncpy(namespace_, cfg_.ns_default, NS_MAX_LENGTH);
    namespace_[NS_MAX_LENGTH - 1] = '\0';
    return true;
  }

  // ============== Accessors ==============

  HardwareSerial& active() { return *active_->serial; }
  const SerialConfig& activeConfig() const { return *active_; }
  const char* getNamespace() const { return namespace_; }

  HardwareSerial& main() { return *cfg_.main.serial; }
  HardwareSerial& alt() { return *cfg_.alt->serial; }

  HardwareSerial& debug() {
    return (active_->serial == cfg_.main.serial) ? *cfg_.alt->serial
                                                 : *cfg_.main.serial;
  }

 private:
  SerialManagerConfig cfg_;
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

    const char* fw_version = "0.0.0";
#if defined(FW_VERSION)
    fw_version = FW_VERSION;
#endif
    serial.printf("FW: %s\r\n", fw_version);
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

      if (millis() - last_ready >= cfg_.resend_ready_interval) {
        serial.printf("FW: %s\r\n", fw_version);
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

extern SerialManager g_serialManager;
