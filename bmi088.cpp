#include <Arduino.h>
#include "bmi088.h"

static void bmi088_writeReg(uint8_t cs, uint8_t addr, uint8_t val) {
    sensorSPI.beginTransaction(SPISettings(10'000'000, MSBFIRST, SPI_MODE0));
    digitalWrite(cs, LOW);
    sensorSPI.transfer(addr & 0x7F);
    sensorSPI.transfer(val);
    digitalWrite(cs, HIGH);
    sensorSPI.endTransaction();
}

void bmi088_init(uint8_t cs_acc, uint8_t cs_gyr) {
    (void)cs_gyr;  // gyro heeft geen init nodig
    bmi088_writeReg(cs_acc, 0x7C, 0x00);  // PWR_CONF: active
    delay(5);
    bmi088_writeReg(cs_acc, 0x7D, 0x04);  // PWR_CTRL: accel on
    delay(5);
    bmi088_writeReg(cs_acc, 0x40, 0xA8);  // ACC_CONF: ODR=1600Hz, BW=normal
    bmi088_writeReg(cs_acc, 0x41, 0x00);  // ACC_RANGE: ±3g
}

void bmi088_readAccel(uint8_t cs, float& ax, float& ay, float& az) {
    uint8_t tx[8] = { 0x92, 0, 0, 0, 0, 0, 0, 0 };  // 0x80|0x12 + dummy + 6 bytes
    uint8_t rx[8] = {};
    sensorSPI.beginTransaction(SPISettings(10'000'000, MSBFIRST, SPI_MODE0));
    digitalWrite(cs, LOW);
    sensorSPI.transferBytes(tx, rx, 8);
    digitalWrite(cs, HIGH);
    sensorSPI.endTransaction();
    int16_t rx_raw = (int16_t)((rx[3] << 8) | rx[2]);
    int16_t ry_raw = (int16_t)((rx[5] << 8) | rx[4]);
    int16_t rz_raw = (int16_t)((rx[7] << 8) | rx[6]);
    const float scale = 9.80665f / 10920.0f;  // ±3g default
    ax = rx_raw * scale;
    ay = ry_raw * scale;
    az = rz_raw * scale;
}

void bmi088_readGyro(uint8_t cs, float& gx, float& gy, float& gz) {
    uint8_t tx[7] = { 0x82, 0, 0, 0, 0, 0, 0 };  // 0x80|0x02, geen dummy, 6 bytes
    uint8_t rx[7] = {};
    sensorSPI.beginTransaction(SPISettings(10'000'000, MSBFIRST, SPI_MODE0));
    digitalWrite(cs, LOW);
    sensorSPI.transferBytes(tx, rx, 7);
    digitalWrite(cs, HIGH);
    sensorSPI.endTransaction();
    int16_t rx_raw = (int16_t)((rx[2] << 8) | rx[1]);
    int16_t ry_raw = (int16_t)((rx[4] << 8) | rx[3]);
    int16_t rz_raw = (int16_t)((rx[6] << 8) | rx[5]);
    const float scale = 1.0f / 16.384f;  // ±2000°/s default
    gx = rx_raw * scale;
    gy = ry_raw * scale;
    gz = rz_raw * scale;
}
