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

#include "robot_config.hpp"

// ============== Configuration ==============

#define NS_MAX_LENGTH 32
#define NS_DEFAULT ""
#define NS_PRECOMM_TIMEOUT 1500  // ms
#define NS_PRECOMM_PREFIX "NS:"  // Protocol: "NS:robot1\n"

// Flash storage (last 16KB sector of STM32F407)
#define NS_FLASH_SECTOR FLASH_SECTOR_11
#define NS_FLASH_ADDR 0x080E0000
#define NS_FLASH_MAGIC 0xCAFE

// ============== Flash Structure ==============

struct NamespaceStorage {
  uint16_t magic;
  uint16_t length;
  char ns[NS_MAX_LENGTH];
  uint32_t crc;
} __attribute__((packed, aligned(4)));

namespace ns_config {

// ============== CRC Calculation ==============

inline uint32_t calculateCRC(const NamespaceStorage& storage) {
  uint32_t crc = 0xFFFFFFFF;
  const uint8_t* data = (const uint8_t*)&storage;
  // CRC over magic + length + ns (exclude crc field itself)
  size_t len = offsetof(NamespaceStorage, crc);
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320 : 0);
    }
  }
  return ~crc;
}

// ============== Flash Operations ==============

inline bool loadFromFlash(char* outNs, size_t maxLen) {
  const NamespaceStorage* stored = (const NamespaceStorage*)NS_FLASH_ADDR;

  // Validate magic
  if (stored->magic != NS_FLASH_MAGIC) {
    return false;
  }

  // Validate length
  if (stored->length == 0 || stored->length >= NS_MAX_LENGTH) {
    return false;
  }

  // Validate CRC
  if (calculateCRC(*stored) != stored->crc) {
    return false;
  }

  // Copy namespace
  size_t copyLen = min((size_t)stored->length, maxLen - 1);
  memcpy(outNs, stored->ns, copyLen);
  outNs[copyLen] = '\0';

  return true;
}

inline bool saveToFlash(const char* ns) {
  NamespaceStorage storage;
  storage.magic = NS_FLASH_MAGIC;
  storage.length = strlen(ns);
  strncpy(storage.ns, ns, NS_MAX_LENGTH - 1);
  storage.ns[NS_MAX_LENGTH - 1] = '\0';
  storage.crc = calculateCRC(storage);

  // Unlock Flash
  HAL_FLASH_Unlock();

  // Erase sector
  FLASH_EraseInitTypeDef eraseInit;
  eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
  eraseInit.Sector = NS_FLASH_SECTOR;
  eraseInit.NbSectors = 1;
  eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  uint32_t sectorError;
  if (HAL_FLASHEx_Erase(&eraseInit, &sectorError) != HAL_OK) {
    HAL_FLASH_Lock();
    return false;
  }

  // Write data (word by word)
  uint32_t* src = (uint32_t*)&storage;
  uint32_t addr = NS_FLASH_ADDR;
  size_t words = (sizeof(NamespaceStorage) + 3) / 4;

  for (size_t i = 0; i < words; i++) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, src[i]) != HAL_OK) {
      HAL_FLASH_Lock();
      return false;
    }
    addr += 4;
  }

  HAL_FLASH_Lock();
  return true;
}

// ============== Pre-communication ==============

inline bool waitForHostConfig(const SerialConfig& serialCfg, char* outNs,
                              size_t maxLen) {
  HardwareSerial* serial = serialCfg.serial;

  // Initialize serial for pre-comm
  serial->setRx(serialCfg.rxPin);
  serial->setTx(serialCfg.txPin);
  serial->begin(serialCfg.baudrate);
  serial->setTimeout(NS_PRECOMM_TIMEOUT);

  // Signal ready
  serial->println("READY");
  serial->flush();

  // Wait for namespace command
  String line = serial->readStringUntil('\n');

  if (line.startsWith(NS_PRECOMM_PREFIX)) {
    String ns = line.substring(strlen(NS_PRECOMM_PREFIX));
    ns.trim();

    if (ns.length() > 0 && ns.length() < maxLen) {
      ns.toCharArray(outNs, maxLen);
      serial->println("ACK");
      serial->flush();
      serial->end();
      return true;
    }
  }

  serial->end();
  return false;
}

// ============== Main Configuration Function ==============

inline void configure(const SerialConfig& serialCfg, char* nsBuffer,
                      size_t bufferLen) {
  // 1. Try to get namespace from host via pre-communication
  if (waitForHostConfig(serialCfg, nsBuffer, bufferLen)) {
    // Got new namespace - save to Flash for future boots
    saveToFlash(nsBuffer);
    return;
  }

  // 2. No host response - try loading from Flash
  if (loadFromFlash(nsBuffer, bufferLen)) {
    return;
  }

  // 3. Flash empty/invalid - use default
  strncpy(nsBuffer, NS_DEFAULT, bufferLen - 1);
  nsBuffer[bufferLen - 1] = '\0';
}

}  // namespace ns_config

extern char g_namespace[NS_MAX_LENGTH];
