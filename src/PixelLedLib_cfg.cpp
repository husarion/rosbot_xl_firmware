#include "PixelLedLib_cfg.h"

SPIClass PixelSpi(PIXEL_MOSI, PB14, PIXEL_SCK);
PixelLedClass PixelStrip(PIXEL_LENGTH, VIRTUAL_LED_LENGTH, 0);

uint8_t PixelInitActions(PixelLedClass *PixelStrip_) {
    if (PixelStrip_->GetInstance() == 0) {
        PixelStrip_->SetStripColour(0x0F, 0x00, 0x00, 0x0F);
        PixelStrip_->PixelStripMapSwap(13, 17);
        PixelStrip_->PixelStripMapSwap(14, 16);
        return 0;
    } else {
        return 1;
    }
}

void PixelDelay(uint32_t time) {
    vTaskDelay(TickType_t(time / portTICK_PERIOD_MS));
}

uint8_t PixelSpiInit(PixelLedClass *PixelStrip_) {
    if (PixelStrip_->GetInstance() == 0) {
        SPISettings spi_settings(PIXEL_SPI_SPEED, LSBFIRST, SPI_MODE3, SPI_TRANSMITONLY);
        PixelSpi.beginTransaction(CS_PIN_CONTROLLED_BY_USER, spi_settings);
        return 0;
    } else {
        return 1;
    }
}

void PixelSpiTransferData(PixelLedClass *PixelStrip_, uint8_t DataToSend_) {
    if (PixelStrip_->GetInstance() == 0) {
        PixelSpi.transfer(CS_PIN_CONTROLLED_BY_USER, DataToSend_);
    }
}

void PixelIddleAnimation(PixelLedClass *PixelStrip_, uint8_t Red_, uint8_t Green_, uint8_t Blue_, uint8_t Brightness_, uint32_t Interval_) {
    uint8_t StripLength = PixelStrip_->GetStripLength();
    for (int i = 0; i < StripLength / 2; i++) {
        PixelStrip_->SetNthLedBuffer(((StripLength / 2) - i - 1), Red_, Green_, Blue_, Brightness_);
        PixelStrip_->SetNthLedBuffer(((StripLength / 2) + i), Red_, Green_, Blue_, Brightness_);
        PixelStrip_->SendBuffersData();
        PixelDelay(Interval_);
    }
}
