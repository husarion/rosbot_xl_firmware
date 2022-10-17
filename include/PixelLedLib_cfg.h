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

#include <PixelLedLib.h>
#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <hardware_cfg.h>
#include <SPI.h>

/* VARIABLES */

SPIClass pixel_spi(PIXEL_MOSI, PB14, PIXEL_SCK);
uint8_t virtual_leds[VIRTUAL_LED_LENGTH] = {5, 6, 7, 8, 9, 18, 19, 20, 21, 22};
PixelLedClass pixel_strip(PIXEL_LENGTH, VIRTUAL_LED_LENGTH, 0);

/* PIXEL LED LIBRARY PORT */

extern uint8_t PixelInitActions(PixelLedClass* pixel_strip){
    if(pixel_strip->GetInstance() == 0){
    pixel_strip->SetStripColour(0x0F, 0x00, 0x00, 0x0F);
        if(pixel_strip->PixelStripMapSwap(23,27))
            return 1;
        if(pixel_strip->PixelStripMapSwap(24,26))
            return 1;
        if(pixel_strip->SetLedsAsVirtual(virtual_leds, VIRTUAL_LED_LENGTH));
            return 1;
        return 0;
    }
    else
        return 1;
}

extern void PixelDelay(uint32_t time){
    vTaskDelay(TickType_t(time / portTICK_PERIOD_MS));
}

extern uint8_t PixelSpiInit(PixelLedClass* pixel_strip){
    if(pixel_strip->GetInstance() == 0){
        SPISettings spi_settings(PIXEL_SPI_SPEED, LSBFIRST, SPI_MODE3, SPI_TRANSMITONLY);
        pixel_spi.beginTransaction(CS_PIN_CONTROLLED_BY_USER, spi_settings);
        return 0;
    }
    else 
        return 1;
}

extern void PixelSpiTransferData(PixelLedClass* pixel_strip, uint8_t data_to_send){
    if(pixel_strip->GetInstance() == 0)
        pixel_spi.transfer(CS_PIN_CONTROLLED_BY_USER, data_to_send);
}

/* FUNCTIONS */

void PixelIddleAnimation(PixelLedClass* PixelStrip_, uint8_t Red_, uint8_t Green_, uint8_t Blue_, uint8_t Brightness_, uint32_t Interval_){
    uint8_t StripLength = PixelStrip_->GetStripLength();
    for(int i = 0; i < StripLength/2; i++){
        PixelStrip_->SetNthLedBuffer(((StripLength/2)-i-1), Red_, Green_, Blue_, Brightness_);
        PixelStrip_->SetNthLedBuffer(((StripLength/2)+i), Red_, Green_, Blue_, Brightness_);
        PixelStrip_->SendBuffersData();
        PixelDelay(Interval_);
    }
}

#endif /* PixelLedLibCfg_H */