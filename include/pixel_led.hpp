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

/*  INCLUDES */

#include <Arduino.h>
#include <PixelLedLib.h>
#include <SPI.h>
#include <STM32FreeRTOS.h>

#include "hardware_cfg.hpp"

/* VARIABLES */

SPIClass PixelSpi(PIXEL_MOSI, PB14, PIXEL_SCK);
uint8_t VirtualLeds[VIRTUAL_LED_LENGTH] = {5, 6, 7, 8, 9, 18, 19, 20, 21, 22};
PixelLedClass PixelStrip(PIXEL_LENGTH, VIRTUAL_LED_LENGTH, 0);

/* PIXEL LED LIBRARY PORT */

extern uint8_t PixelInitActions(PixelLedClass* PixelStrip_) {
  if (PixelStrip_->GetInstance() == 0) {
    PixelStrip_->SetStripColour(0x0F, 0x00, 0x00, 0x0F);
    if (PixelStrip_->PixelStripMapSwap(23, 27)) return 1;
    if (PixelStrip_->PixelStripMapSwap(24, 26)) return 1;
    if (PixelStrip_->SetLedsAsVirtual(VirtualLeds, VIRTUAL_LED_LENGTH))
      ;
    return 1;
    return 0;
  } else
    return 1;
}

extern void PixelDelay(uint32_t time) {
  vTaskDelay(TickType_t(time / portTICK_PERIOD_MS));
}

extern uint8_t PixelSpiInit(PixelLedClass* PixelStrip_) {
  if (PixelStrip_->GetInstance() == 0) {
    SPISettings spi_settings(PIXEL_SPI_SPEED, LSBFIRST, SPI_MODE3,
                             SPI_TRANSMITONLY);
    PixelSpi.beginTransaction(CS_PIN_CONTROLLED_BY_USER, spi_settings);
    return 0;
  } else
    return 1;
}

extern void PixelSpiTransferData(PixelLedClass* PixelStrip_,
                                 uint8_t DataToSend_) {
  if (PixelStrip_->GetInstance() == 0)
    PixelSpi.transfer(CS_PIN_CONTROLLED_BY_USER, DataToSend_);
}

/* FUNCTIONS */

void PixelIddleAnimation(PixelLedClass* PixelStrip_, uint8_t Red_,
                         uint8_t Green_, uint8_t Blue_, uint8_t Brightness_,
                         uint32_t Interval_) {
  uint8_t StripLength = PixelStrip_->GetStripLength();
  for (int i = 0; i < StripLength / 2; i++) {
    PixelStrip_->SetNthLedBuffer(((StripLength / 2) - i - 1), Red_, Green_,
                                 Blue_, Brightness_);
    PixelStrip_->SetNthLedBuffer(((StripLength / 2) + i), Red_, Green_, Blue_,
                                 Brightness_);
    PixelStrip_->SendBuffersData();
    PixelDelay(Interval_);
  }
}

#endif /* PixelLedLibCfg_H */
