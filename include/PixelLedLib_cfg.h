/**
 * @file PixelLedLib_cfg.cpp
 * @author Maciej Kurcius
 * @brief
 * @version 0.1
 * @date 2022-02-09
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef PixelLedLibCfg_H
#define PixelLedLibCfg_H

/*  INCLUDES */

#include <Arduino.h>
#include <PixelLedLib.h>
#include <SPI.h>
#include <STM32FreeRTOS.h>
#include <hardware_cfg.h>

/* VARIABLES */

#define BRIGHTNESS_0 0b00010 // 0xx10
#define BRIGHTNESS_1 0b10010 // 1xx10
#define BRIGHTNESS_2 0b00100 // xx100
#define BRIGHTNESS_3 0b01000 // x1000
#define BRIGHTNESS_4 0b10000 // 10000
#define BRIGHTNESS_5 0b00000 // 00000
#define BRIGHTNESS_6 0b00001 // 00xx1
#define BRIGHTNESS_7 0b10001 // 10xx1
#define BRIGHTNESS_8 0b01001 // 01xx1
#define BRIGHTNESS_9 0b11001 // 11xx1

SPIClass PixelSpi(PIXEL_MOSI, PB14, PIXEL_SCK);
// uint8_t VirtualLeds[VIRTUAL_LED_LENGTH] = {5, 6, 7, 8, 9, 18, 19, 20, 21, 22};
PixelLedClass PixelStrip(PIXEL_LENGTH, VIRTUAL_LED_LENGTH, 0);

/* PIXEL LED LIBRARY PORT */

extern uint8_t PixelInitActions(PixelLedClass * PixelStrip_)
{
  if (PixelStrip_->GetInstance() == 0) {
    PixelStrip_->SetStripColour(0x0F, 0x00, 0x00, 0x0F);
    PixelStrip_->PixelStripMapSwap(13, 17);
    PixelStrip_->PixelStripMapSwap(14, 16);
    // if (PixelStrip_->PixelStripMapSwap(23, 27)) return 1;
    // if (PixelStrip_->PixelStripMapSwap(24, 26)) return 1;
    // if (PixelStrip_->SetLedsAsVirtual(VirtualLeds, VIRTUAL_LED_LENGTH))
    //   ;
    // return 1;
    // return 0;
  } else
    return 1;
}

extern void PixelDelay(uint32_t time) { vTaskDelay(TickType_t(time / portTICK_PERIOD_MS)); }

extern uint8_t PixelSpiInit(PixelLedClass * PixelStrip_)
{
  if (PixelStrip_->GetInstance() == 0) {
    SPISettings spi_settings(PIXEL_SPI_SPEED, LSBFIRST, SPI_MODE3, SPI_TRANSMITONLY);
    PixelSpi.beginTransaction(CS_PIN_CONTROLLED_BY_USER, spi_settings);
    return 0;
  } else
    return 1;
}

extern void PixelSpiTransferData(PixelLedClass * PixelStrip_, uint8_t DataToSend_)
{
  if (PixelStrip_->GetInstance() == 0) PixelSpi.transfer(CS_PIN_CONTROLLED_BY_USER, DataToSend_);
}

/* FUNCTIONS */

void PixelIddleAnimation(
  PixelLedClass * PixelStrip_, uint8_t Red_, uint8_t Green_, uint8_t Blue_, uint8_t Brightness_,
  uint32_t Interval_)
{
  uint8_t StripLength = PixelStrip_->GetStripLength();
  for (int i = 0; i < StripLength / 2; i++) {
    PixelStrip_->SetNthLedBuffer(((StripLength / 2) - i - 1), Red_, Green_, Blue_, Brightness_);
    PixelStrip_->SetNthLedBuffer(((StripLength / 2) + i), Red_, Green_, Blue_, Brightness_);
    PixelStrip_->SendBuffersData();
    PixelDelay(Interval_);
  }
}

#endif /* PixelLedLibCfg_H */
