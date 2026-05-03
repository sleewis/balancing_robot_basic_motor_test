#pragma once
#include <Arduino.h>

// ── 74HC42 BCD-naar-decimaal decoder ─────────────────────────────────────────
// 3A is hardwired naar GND → gebruikt als 1-of-8 decoder (uitvoer 0Y–7Y).
//
// Adrespinnen (ESP32 → 74HC42):
//   HC42_A0  GPIO23  → 0A (LSB)
//   HC42_A1  GPIO5   → 1A
//   HC42_A2  GPIO21  → 2A
//
// Apparaat-codes (BCD-ingang → welke uitvoer LOW gaat):
//   DEV_ENC1    0  → 0Y  (encoder motor 1)
//   DEV_ENC2    1  → 1Y  (encoder motor 2)
//   DEV_BMI_ACC 2  → 2Y  (BMI088 accel)
//   DEV_BMI_GYR 3  → 3Y  (BMI088 gyro)
//   DEV_NONE    4  → 4Y  (niet aangesloten – alle CS hoog = deselect)
// ─────────────────────────────────────────────────────────────────────────────

#define HC42_A0  23
#define HC42_A1   5
#define HC42_A2  21

#define DEV_ENC1     0
#define DEV_ENC2     1
#define DEV_BMI_ACC  2
#define DEV_BMI_GYR  3
#define DEV_NONE     4

inline void hc42_select(uint8_t code) {
    digitalWrite(HC42_A0, (code >> 0) & 1);
    digitalWrite(HC42_A1, (code >> 1) & 1);
    digitalWrite(HC42_A2, (code >> 2) & 1);
}

inline void hc42_deselect() {
    hc42_select(DEV_NONE);
}

inline void hc42_init() {
    pinMode(HC42_A0, OUTPUT);
    pinMode(HC42_A1, OUTPUT);
    pinMode(HC42_A2, OUTPUT);
    hc42_deselect();
}
