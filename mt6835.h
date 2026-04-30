#pragma once
#include <Arduino.h>
#include <SPI.h>

// Gedeelde SPI-bus, gedefinieerd in het hoofd-.ino bestand.
extern SPIClass sensorSPI;

// Lees één register (24-bit frame, SPI_MODE3, max 16 MHz).
uint8_t mt6835_readReg(uint8_t cs, uint16_t addr);

// Lees de absolute hoek via burst read (registers 0x003–0x006 in één transactie).
// statusOut (optioneel): bit1=MAGH (magneet te sterk), bit0=MAGL (magneet te zwak).
float mt6835_readAngle(uint8_t cs, uint8_t* statusOut = nullptr);
