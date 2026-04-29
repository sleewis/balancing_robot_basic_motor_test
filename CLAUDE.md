# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project doel
Testproject voor open-loop sinusoïdale FOC-aansturing van twee BLDC-motoren op de MKS-ESP32FOC V2.0, met directe SPI-uitlezing van MT6835 encoders en BMI088 IMU. Bouwsteen voor het grotere `balancing_robot`-project.

## Build & upload
Compileren en uploaden via **Arduino IDE 2.x**. Board: MKS-ESP32FOC-boardpakket (ESP32). Sjoerd doet compileren/uploaden zelf.

Seriële debug-output: 115200 baud via USB.

## Enable-flags (bovenaan .ino)
```cpp
#define USE_ENC1    true/false   // MT6835 encoder motor 1
#define USE_ENC2    true/false   // MT6835 encoder motor 2
#define USE_IMU     true/false   // BMI088 accel + gyro
#define USE_MOTOR1  true/false   // motor 1 aansturen
#define USE_MOTOR2  true/false   // motor 2 aansturen
```
Alleen aangevinkte onderdelen worden geïnitialiseerd en gelezen. Geen simulatiedata bij `false`.

## Architectuur

### `.ino` (main)
- SPI-bus via `SPIClass sensorSPI(VSPI)` op J5 (SCK=IO18, MISO=IO19, MOSI=IO22).
- CS-lijnen op J6: IO23 (ENC1), IO5 (ENC2), IO21 (BMI_ACC), IO4 (BMI_GYR – tijdelijk, zie TODO).
- Hardware-timer (1 MHz base, alarm 1000 ticks = **1 kHz regelloop**) zet `motorFlag` in ISR.
- `readSensors()` leest actieve sensoren in de regelloop; berekent snelheid via finite-difference.
- `loop()` leest sensoren, drijft actieve motoren aan met externe hoek, print diagnostiek 1×/s.

### `Motor.h / Motor.cpp`
Motor-klasse zonder I²C-afhankelijkheid; hoek wordt extern aangeleverd.

| Methode | Functie |
|---|---|
| `begin()` | LEDC PWM initialiseren (20 kHz, 10-bit), EN HIGH – géén interne uitlijning |
| `setOffset(float)` | Elektrische offset instellen na externe uitlijning |
| `loop(voltage, mechAngle)` | Externe mechAngle [rad] → elektrische hoek → `setPhaseVoltage()` |
| `setPhaseVoltage(angle, V)` | Sinusoïdaal 3-fase PWM (center-aligned: `(sin·0.5+0.5)·PWM_MAX`) |
| `readCurrent(pin)` | ADC → ampère: `(raw − adcMid) × 3.3/4095 / (shunt × gain)` |

Uitlijning gebeurt in `setup()` van de `.ino`: stator op 0 rad, 3 s wachten, MT6835 lezen, offset berekenen, `setOffset()` aanroepen.

### MT6835 encoder driver
- SPI_MODE3, 16 MHz, 24-bit frames.
- **Burst read** van registers 0x003–0x006 in één transactie (6 bytes).
- Snelheidsberekening: `Δhoek / dt` met wraparound-correctie (±180°).
- TODO: moving-average filter (8–16 samples) toevoegen bij zichtbare ruis in de regelaar.

### BMI088 IMU driver
- Accel: SPI_MODE0, 10 MHz; dummy-byte na adres; ±3g default (schaal: 9.80665/10920).
- Gyro:  SPI_MODE0, 10 MHz; geen dummy-byte; ±2000°/s default (schaal: 1/16.384).

### Hardware-parameters
- `PWM_FREQ` 20000 Hz, `PWM_RES` 10-bit, `PWM_MAX` 1023
- `POLE_PAIRS` 11 (GM4108H-120T)
- `SENSOR_HZ` 1000 (= regelloop-frequentie)
- Shunt 0.01 Ω, versterking 50×, ADC-middenpunt ~1845

### Pinout motoren
| | U | V | W | EN | IA | IB |
|---|---|---|---|---|---|---|
| Motor 1 | 33 | 32 | 25 | 12 | 39 | 36 |
| Motor 2 | 26 | 27 | 14 | 12 | 34 | 35 |

## Toekomstige uitbreidingen
- **74HC42 decoder**: CS-lijnen voor ENC1/ENC2/BMI_ACC/BMI_GYR samenvatten naar 2–3 GPIO-pins. CS_BMI_GYR (IO4) is tijdelijk en wordt dan opnieuw ingedeeld.
- Snelheidsregelaar toevoegen (PID op `enc_rads`).
- Stroombeveiliging via `updateCurrent()` activeren.
