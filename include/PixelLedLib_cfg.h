#ifndef PIXELLEDLIB_CFG_H
#define PIXELLEDLIB_CFG_H

#include <Arduino.h>
#include <PixelLedLib.h>
#include <SPI.h>
#include <STM32FreeRTOS.h>
#include <hardware_cfg.h>

#define BRIGHTNESS_0 0b00010
#define BRIGHTNESS_1 0b10010
#define BRIGHTNESS_2 0b00100
#define BRIGHTNESS_3 0b01000
#define BRIGHTNESS_4 0b10000
#define BRIGHTNESS_5 0b00000
#define BRIGHTNESS_6 0b00001
#define BRIGHTNESS_7 0b10001
#define BRIGHTNESS_8 0b01001
#define BRIGHTNESS_9 0b11001

extern SPIClass PixelSpi;
extern PixelLedClass PixelStrip;

extern uint8_t PixelInitActions(PixelLedClass *PixelStrip_);
extern void PixelDelay(uint32_t time);
extern uint8_t PixelSpiInit(PixelLedClass *PixelStrip_);
extern void PixelSpiTransferData(PixelLedClass *PixelStrip_, uint8_t DataToSend_);
extern void PixelIddleAnimation(PixelLedClass *PixelStrip_, uint8_t Red_, uint8_t Green_, uint8_t Blue_, uint8_t Brightness_, uint32_t Interval_);

#endif // PIXELLEDLIB_CFG_H
