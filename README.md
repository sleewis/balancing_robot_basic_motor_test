# Balancing Robot – Basic Motor Test

Testschets voor **Field Oriented Control (FOC)** in voltage-mode van twee brushless DC-motoren op een ESP32-gebaseerd motorstuurbordje. Encoders en IMU worden direct via SPI uitgelezen. Dit project is de bouwsteen voor een zelfbalancerende robot (Segway-stijl) die later ook via Bluetooth met een XBOX-controller aangestuurd wordt.

---

## Hardware

| Onderdeel | Specificatie |
|---|---|
| Microcontroller | MKS-ESP32FOC V2.0 (ESP32) |
| Motoren | 2× GM4108H-120T – 11 poolparen, 11.1 Ω, ~20 V |
| Encoders | 2× MT6835 – magnetisch, absoluut, 21-bit, SPI |
| IMU | BMI088 – accel + gyro, SPI |
| Spanningsmonitor | Adafruit INA228 – I²C, 85 V / 20-bit *(nog niet geïntegreerd)* |

---

## Pinout

### SPI sensor-bus → J5

| J5-pin | GPIO | Functie | Draadkleur |
|---|---|---|---|
| SDA | IO19 | MISO | Blauw |
| SCL | IO18 | SCK  | Wit |
| I_0 | IO22 | MOSI | Geel |

### CS-lijnen → J6

| J6-pin | GPIO | Sensor |
|---|---|---|
| SDA | IO23 | CS encoder 1 |
| SCL | IO5  | CS encoder 2 |
| I_1 | IO21 | CS BMI088 accel |
| –   | IO4  | CS BMI088 gyro *(tijdelijk, zie 74HC42)* |

> **Let op:** IO4 (BMI088 gyro CS) is een tijdelijke toewijzing. Zodra de **74HC42 BCD-decoder** beschikbaar is, worden alle vier CS-lijnen samengevat naar 2–3 GPIO-pins.

### Motoren

| Signaal | Motor 1 | Motor 2 |
|---|---|---|
| Fase U (PWM) | GPIO 33 | GPIO 26 |
| Fase V (PWM) | GPIO 32 | GPIO 27 |
| Fase W (PWM) | GPIO 25 | GPIO 14 |
| Enable gate-driver | GPIO 12 | GPIO 12 |
| Stroom fase A (ADC) | GPIO 39 | GPIO 34 |
| Stroom fase B (ADC) | GPIO 36 | GPIO 35 |

---

## Enable-flags

Bovenaan het `.ino`-bestand staan vijf flags. Zet alleen aan wat fysiek is aangesloten:

```cpp
#define USE_ENC1    true    // MT6835 encoder motor 1
#define USE_ENC2    false   // MT6835 encoder motor 2
#define USE_IMU     false   // BMI088 accel + gyro
#define USE_MOTOR1  true    // motor 1 aansturen
#define USE_MOTOR2  false   // motor 2 aansturen
```

Wat op `false` staat wordt volledig overgeslagen – geen init, geen sensorlezing, geen simulatiedata.

---

## Werkingsprincipe

De schets implementeert **voltage-mode sinusoïdale FOC**:

1. **Rotoruitlijning** – bij opstart wordt een kleine spanning aangelegd op een bekende elektrische hoek (0 rad). De rotor trekt naar de bijbehorende positie. Na 3 seconden wordt de mechanische hoek via MT6835 (SPI) gelezen en de **elektrische offset** berekend en opgeslagen via `setOffset()`.

2. **Sensor uitlezing (1 kHz)** – `readSensors()` leest actieve encoders en/of IMU rechtstreeks via SPI. Per encoder wordt ook de **hoeksnelheid** berekend (finite-difference met wraparound-correctie):
   ```
   ω [rad/s] = Δhoek [deg] × (π/180) × SENSOR_HZ
   ```

3. **Regelloop (1 kHz)** – hardware-timer interrupt zet elke milliseconde een vlag. `loop()` voert één regelstap uit:
   - Lees mechanische hoek uit `sensors.enc1_deg`
   - Bereken elektrische hoek: `θ_elec = θ_mech × polePairs + offset + π/2`
   - Genereer sinusoïdaal 3-fase PWM op basis van `θ_elec`

4. **PWM-generatie** – drie fases 120° uit fase. Bipolair naar unipolair:
   `pwm = (sin(θ) × 0.5 + 0.5) × 1023`

### PWM-instellingen

| Parameter | Waarde |
|---|---|
| Frequentie | 20 kHz |
| Resolutie | 10-bit (0–1023) |

---

## Bestandsstructuur

```
balancing_robot_basic_motor_test/
├── balancing_robot_basic_motor_test.ino   # Setup, loop, timer-ISR, sensor drivers
├── Motor.h                                # Klasse-declaratie
├── Motor.cpp                              # FOC-implementatie
├── CLAUDE.md                              # Instructies voor Claude Code
└── README.md                              # Dit bestand
```

### Klasse-overzicht `Motor`

| Methode | Beschrijving |
|---|---|
| `Motor(u,v,w,en,ia,ib,poles)` | Constructor, stelt pinout en stroomparameters in |
| `begin()` | PWM initialiseren, enable gate-driver – géén interne uitlijning |
| `setOffset(float)` | Elektrische offset instellen na externe uitlijning |
| `loop(voltage, mechAngle)` | Één regelstap: externe hoek [rad] → elektrische hoek → PWM |
| `setPhaseVoltage(angle, V)` | Schrijf sinusoïdaal 3-fase PWM |
| `readCurrent(pin)` | ADC → ampère via shunt (0.01 Ω) + versterking (50×) |

---

## Installatie & gebruik

### Vereisten
- **Arduino IDE 2.x** met ESP32-boardpakket
- Geen extra libraries nodig (alleen ingebouwde `SPI.h`)

### Board-instelling
- **Board**: MKS-ESP32FOC boardpakket of `ESP32 Dev Module`
- **Upload Speed**: 921600

### Eerste gebruik
1. Sluit MKS-ESP32FOC aan via USB.
2. Stel de enable-flags in bovenaan het `.ino`-bestand.
3. Selecteer het juiste board en COM-poort, upload de schets.
4. Open de seriële monitor op **115200 baud**.
5. Bij `USE_MOTOR1=true` en `USE_ENC1=true`: na ~3 seconden uitlijning verschijnt de offset-waarde en begint de motor te draaien.

### Spanning aanpassen
`DRIVE_VOLTAGE_V` (standaard 0.6 V) bepaalt het koppel. Verhoog voorzichtig:
- Te laag → motor beweegt niet of schokkerig
- Te hoog → overmatige stroom (geen stroombeveiliging in open-loop!)

---

## Toekomstige uitbreidingen

- [ ] 74HC42 decoder integreren voor CS-lijn demultiplexing
- [ ] BMI088 IMU testen (`USE_IMU=true`)
- [ ] Moving-average filter op hoeksnelheid (ruis reduceren)
- [ ] Gesloten-lus stroomregeling (FOC met PI-regelaar)
- [ ] Snelheidsregelaar op basis van `enc_rads`
- [ ] PID-balansregelaar op basis van IMU
- [ ] Bluetooth-besturing via XBOX-controller
- [ ] Accuspanning bewaking via INA228

---

## Licentie

MIT – vrij te gebruiken en aan te passen.
