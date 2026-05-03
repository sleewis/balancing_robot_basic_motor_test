#include <Arduino.h>
#include "bmi088.h"
#include "hc42.h"

static void bmi088_writeReg(uint8_t dev, uint8_t addr, uint8_t val) {
    sensorSPI.beginTransaction(SPISettings(10'000'000, MSBFIRST, SPI_MODE0));
    hc42_select(dev);
    sensorSPI.transfer(addr & 0x7F);
    sensorSPI.transfer(val);
    hc42_deselect();
    sensorSPI.endTransaction();
}

void bmi088_init(uint8_t dev_acc, uint8_t dev_gyr) {
    (void)dev_gyr;
    bmi088_writeReg(dev_acc, 0x7C, 0x00);  // PWR_CONF: active
    delay(5);
    bmi088_writeReg(dev_acc, 0x7D, 0x04);  // PWR_CTRL: accel on
    delay(5);
    bmi088_writeReg(dev_acc, 0x40, 0xA8);  // ACC_CONF: ODR=1600Hz, BW=normal
    bmi088_writeReg(dev_acc, 0x41, 0x00);  // ACC_RANGE: ±3g
}

void bmi088_readAccel(uint8_t dev, float& ax, float& ay, float& az) {
    uint8_t tx[8] = { 0x92, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t rx[8] = {};
    sensorSPI.beginTransaction(SPISettings(10'000'000, MSBFIRST, SPI_MODE0));
    hc42_select(dev);
    sensorSPI.transferBytes(tx, rx, 8);
    hc42_deselect();
    sensorSPI.endTransaction();
    int16_t rx_raw = (int16_t)((rx[3] << 8) | rx[2]);
    int16_t ry_raw = (int16_t)((rx[5] << 8) | rx[4]);
    int16_t rz_raw = (int16_t)((rx[7] << 8) | rx[6]);
    const float scale = 9.80665f / 10920.0f;
    ax = rx_raw * scale;
    ay = ry_raw * scale;
    az = rz_raw * scale;
}

void bmi088_readGyro(uint8_t dev, float& gx, float& gy, float& gz) {
    uint8_t tx[7] = { 0x82, 0, 0, 0, 0, 0, 0 };
    uint8_t rx[7] = {};
    sensorSPI.beginTransaction(SPISettings(10'000'000, MSBFIRST, SPI_MODE0));
    hc42_select(dev);
    sensorSPI.transferBytes(tx, rx, 7);
    hc42_deselect();
    sensorSPI.endTransaction();
    int16_t rx_raw = (int16_t)((rx[2] << 8) | rx[1]);
    int16_t ry_raw = (int16_t)((rx[4] << 8) | rx[3]);
    int16_t rz_raw = (int16_t)((rx[6] << 8) | rx[5]);
    const float scale = 1.0f / 16.384f;
    gx = rx_raw * scale;
    gy = ry_raw * scale;
    gz = rz_raw * scale;
}
