# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project doel
Testproject voor open-loop sinusoïdale FOC-aansturing van twee BLDC-motoren op de MKS-ESP32FOC V2.0. Bouwsteen voor het grotere `balancing_robot`-project.

## Build & upload
Compileren en uploaden gebeurt via **Arduino IDE 2.x** (geen CLI beschikbaar in deze omgeving). Board: `ESP32S3 Dev Module` of het MKS-ESP32FOC-boardpakket. Sjoerd doet compileren/uploaden zelf.

Seriële debug-output: 115200 baud via USB.

## Architectuur

### `.ino` (main)
- Maakt twee `TwoWire`-instanties (`I2C_1` op pins 19/18, `I2C_2` op pins 23/5, 400 kHz).
- Maakt `Motor`-objecten aan met pinout: U, V, W (LEDC PWM), EN (enable), IA, IB (current sense ADC), wire-bus, polePairs.
- Hardware-timer (1 MHz base, alarm elke 1000 ticks = **1 kHz regelloop**) zet `motorFlag` in ISR (`IRAM_ATTR`).
- `loop()` controleert de flag en roept `motor.loop(voltage)` aan.

### `Motor.h / Motor.cpp`
Één klasse die alles afhandelt voor één BLDC-motor:

| Methode | Functie |
|---|---|
| `begin()` | LEDC PWM initialiseren (20 kHz, 10-bit), EN HIGH, `alignRotor()` |
| `alignRotor()` | Stator op 0 rad zetten, 3 s wachten, mechanische hoek lezen → offset berekenen |
| `loop(voltage)` | Mechanische hoek → elektrische hoek → `setPhaseVoltage()` |
| `setPhaseVoltage(angle, V)` | Sinusoïdaal 3-fase PWM (center-aligned: `(sin·0.5+0.5)·PWM_MAX`) |
| `readSensor()` | AS5600 via I²C (register 0x0C, 12-bit ruwe hoek) |
| `getMechanicalAngle()` | rawAngle → radialen (÷4096 × 2π) |
| `readCurrent(pin)` | ADC → ampère: `(raw − adcMid) × 3.3/4095 / (shunt × gain)` |

### Constanten / hardware-parameters
- `PWM_FREQ` 20000 Hz, `PWM_RES` 10-bit, `PWM_MAX` 1023
- `AS5600_ADDR` 0x36
- `polePairs` = 11 (GM4108H-120T)
- Shunt 0.01 Ω, versterking 50×, ADC-middenpunt ~1845 (3.3 V, 12-bit ADC)
- Motor 2 staat uitgecommentarieerd; zelfde pinout-patroon, andere pins en negatieve voltage voor tegengestelde richting.
