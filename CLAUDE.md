# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project doel
Testproject voor sinusoïdale FOC-aansturing van twee BLDC-motoren op de MKS-ESP32FOC V2.0, met directe SPI-uitlezing van MT6835 encoders en BMI088 IMU. Bouwsteen voor het grotere `balancing_robot`-project.

## Build & upload
Compileren en uploaden via **Arduino IDE 2.x**. Board: MKS-ESP32FOC-boardpakket (ESP32). Vereiste library: **arduino-esp32 v3.3.8** (ESP-IDF v5 MCPWM API).
Sjoerd doet compileren/uploaden zelf.

Seriële debug-output: 115200 baud via USB.

## Enable-flags (bovenaan .ino)
```cpp
#define USE_IMU  false   // BMI088 accel + gyro (nog niet ontvangen)
```
Alleen actief subsysteem wordt geïnitialiseerd en gelezen.

## Architectuur

### `.ino` (main)
- SPI-bus via `SPIClass sensorSPI(VSPI)` op J5 (SCK=IO18, MISO=IO19, MOSI=IO22).
- CS-lijnen via 74HC42 decoder op J6: GPIO23=A0, GPIO5=A1, GPIO21=A2.
- FreeRTOS taken:
  - **motorTask** (Core 1, 1 kHz, prioriteit 5): leest encoders, berekent snelheid via moving-average (MA_SIZE=8), voert SpeedPID uit, past cogging-compensatie toe, stuurt motoren aan.
  - **slowTask** (Core 0, 1 Hz, prioriteit 1): print diagnostiek via Serial.
- `sharedSensors` struct wordt via `portMUX` gedeeld tussen de twee taken.

### `Motor.h / Motor.cpp`
Motor-klasse gebaseerd op **MCPWM** (ESP-IDF v5, `mcpwm_prelude.h`); hoek wordt extern aangeleverd.

| Methode | Functie |
|---|---|
| `begin()` | MCPWM initialiseren (center-aligned UP_DOWN, 20 kHz), EN HIGH, ADC kalibratie |
| `setOffset(float)` | Elektrische offset instellen na externe uitlijning |
| `loop(voltage, mechAngle)` | Externe mechAngle [rad] → elektrische hoek → `setPhaseVoltage()` |
| `setPhaseVoltage(angle, V)` | Sinusoïdaal 3-fase center-aligned PWM |
| `updateCurrent()` | EMA-gefilterde stroomberekening (niet TEZ-gesynchroniseerd) |
| `readInstantCurrents(ia, ib)` | ADC-uitlezing gesynchroniseerd op timer-zero via semaphore |
| `computeDQ(mechAngle, ia, ib, id, iq)` | Clarke + Park transformatie |
| `calibrateStart/Step/Finish()` | Cogging-kalibratie (1°/bin, 360 bins) |
| `coggingCorrection(deg)` | Opzoeken voltagekorrekties uit gecalibreerde tabel |
| `coggingAmplitude()` | Peak-to-peak amplitude voor diagnostiek |

### MCPWM-architectuur (kritiek, ESP-IDF v5)
- **3 operators per motor** (één per fase U/V/W), elk met 1 comparator + 1 generator.
- Motor1: group 0, Motor2: group 1.
- `timer_cfg.period_ticks = PWM_RES_TICKS * 2` (= 4000) **verplicht** in UP_DOWN mode:
  - ESP-IDF v5 slaat intern `peak_ticks = period_ticks / 2` op → met 4000 krijg je peak=2000.
  - Geldig compare-bereik: `[0, PWM_RES_TICKS-1]` = `[0, 1999]`.
  - Frequentie: 80 MHz / (2 × 2000) = **20 kHz**.
- `toCmp()` clamps: `t ≤ 0 → 0`, `t ≥ PWM_RES_TICKS → PWM_RES_TICKS-1`.

### ADC-synchronisatie
- MCPWM timer-zero event (TEZ, elke 50 µs) geeft een FreeRTOS binary semaphore vanuit ISR.
- `readInstantCurrents()` wacht op die semaphore → ADC-meting valt precies op het rustpunt van de center-aligned PWM (laagste rimpel).
- `updateCurrent()` (voor overcurrentdetectie) leest asynchroon met EMA-filter.

### Cogging-compensatie
- Kalibratie in `setup()` vóór taakstart: motor draait 8s op 1 r/s via mini-PID.
- Per positiebin (1° resolutie, 360 bins): gemiddelde rijspanning opslaan.
- DC-component (gemiddeld over alle bins) afgetrokken → netto positieafhankelijke correctie.
- Lege bins worden lineair geïnterpoleerd.
- Correctie opgeteld bij SpeedPID output, geconstrained op ±SPEED_OUT_MAX.

### SpeedPID
- Output: genormaliseerde rijspanning `[−1..1]`, direct naar `motor.loop()`.
- Geen IqPI: de stroomregelaar is uitgeschakeld (IqPI saboteerde koppel bij stilstand).
- Clarke/Park (`computeDQ`) draait wel — voor Iq-monitoring en overcurrentdetectie.

### MT6835 encoder driver
- SPI_MODE3, 16 MHz, 24-bit frames, burst read registers 0x003–0x006.
- Snelheid via moving-average over hoekdeltas (MA_SIZE=8 samples, wraparound-correctie ±180°).

### BMI088 IMU driver
- Accel: SPI_MODE0, 10 MHz; dummy-byte na adres; ±3g (schaal: 9.80665/10920).
- Gyro:  SPI_MODE0, 10 MHz; geen dummy-byte; ±2000°/s (schaal: 1/16.384).

### Hardware-parameters
- `PWM_RES_TICKS` 2000, `N_COGGING` 360
- `POLE_PAIRS` 11 (GM4108H-120T)
- `SENSOR_HZ` 1000 (= regelloop-frequentie)
- `SUPPLY_V` 12.0, `MAX_CURRENT_A` 1.0
- `SPEED_KP` 0.10, `SPEED_KI` 0.10, `SPEED_OUT_MAX` 1.0
- Shunt 0.01 Ω, versterking 50×, ADC-middenpunt gekalibreerd bij `begin()` (256 metingen)

### Pinout
| | U | V | W | EN | IA | IB |
|---|---|---|---|---|---|---|
| Motor 1 | 33 | 32 | 25 | 12 | 39 | 36 |
| Motor 2 | 26 | 27 | 14 | 12 | 35 | 34 |

### CS (74HC42 decoder)
| Apparaat | HC42-output | A2 (IO21) | A1 (IO5) | A0 (IO23) |
|---|---|---|---|---|
| ENC1 (MT6835) | Y0 | 0 | 0 | 0 |
| ENC2 (MT6835) | Y1 | 0 | 0 | 1 |
| BMI_ACC       | Y2 | 0 | 1 | 0 |
| BMI_GYR       | Y3 | 0 | 1 | 1 |

## Volgende stappen
- Cogging-kalibratie uploaden en testen; amplitude loggen.
- **TiltPID**: zodra BNO055 (of BMI088) aanwezig, SpeedPID vervangen door tilt-regelaar.
- Stroombeveiliging `MAX_CURRENT_A` fine-tunen op basis van gemeten motorstroom.
