#pragma once
#include <Arduino.h>
#include <SPI.h>

// Gedeelde SPI-bus, gedefinieerd in het hoofd-.ino bestand.
extern SPIClass sensorSPI;

// Initialiseer de BMI088 accel (gyro heeft geen init nodig).
void bmi088_init(uint8_t cs_acc, uint8_t cs_gyr);

// Lees versnelling [m/s²], ±3g default (SPI_MODE0, 10 MHz).
void bmi088_readAccel(uint8_t cs, float& ax, float& ay, float& az);

// Lees hoeksnelheid [°/s], ±2000°/s default (SPI_MODE0, 10 MHz).
void bmi088_readGyro(uint8_t cs, float& gx, float& gy, float& gz);
