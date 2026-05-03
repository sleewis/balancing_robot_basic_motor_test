#include <Arduino.h>
#include "mt6835.h"
#include "hc42.h"

uint8_t mt6835_readReg(uint8_t dev, uint16_t addr) {
    uint8_t tx[3] = { (uint8_t)(0x30 | (addr >> 8)), (uint8_t)(addr & 0xFF), 0x00 };
    uint8_t rx[3] = {};
    sensorSPI.beginTransaction(SPISettings(4'000'000, MSBFIRST, SPI_MODE3)); // max 16'000'000
    hc42_select(dev);
    sensorSPI.transferBytes(tx, rx, 3);
    hc42_deselect();
    sensorSPI.endTransaction();
    return rx[2];
}

float mt6835_readAngle(uint8_t dev, uint8_t* statusOut) {
    uint8_t tx[6] = { 0x30, 0x03, 0x00, 0x00, 0x00, 0x00 };
    uint8_t rx[6] = {};
    sensorSPI.beginTransaction(SPISettings(4'000'000, MSBFIRST, SPI_MODE3)); // max 16'000'000
    hc42_select(dev);
    sensorSPI.transferBytes(tx, rx, 6);
    hc42_deselect();
    sensorSPI.endTransaction();
    if (statusOut) *statusOut = rx[4] & 0x03;
    uint32_t raw = ((uint32_t)rx[2] << 13) | ((uint32_t)rx[3] << 5) | ((rx[4] >> 3) & 0x1F);
    return raw * 360.0f / (float)(1UL << 21);
}
