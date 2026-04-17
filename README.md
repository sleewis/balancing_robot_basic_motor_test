# Balancing Robot – Basic Motor Test

Testschets voor **Field Oriented Control (FOC)** in voltage-mode van twee brushless DC-motoren op een ESP32-gebaseerd motorstuurbordje. Dit project is de bouwsteen voor een zelfbalancerende robot (Segway-stijl) die later ook via Bluetooth met een XBOX-controller aangestuurd wordt.

---

## Hardware

| Onderdeel | Specificatie |
|---|---|
| Microcontroller | MKS-ESP32FOC V2.0 (ESP32) |
| Motoren | 2× GM4108H-120T – 11 poolparen, 11.1 Ω, ~20 V |
| Encoders | 2× AS5600 – magnetisch, absoluut, 12-bit, I²C (adres 0x36) |
| IMU | Adafruit BNO055 (STEMMA QT) |
| Spanningsmonitor | Adafruit INA228 – I²C, 85 V / 20-bit |

### Pinout MKS-ESP32FOC V2.0

| Signaal | Motor 1 | Motor 2 |
|---|---|---|
| Fase U (PWM) | GPIO 33 | GPIO 26 |
| Fase V (PWM) | GPIO 32 | GPIO 27 |
| Fase W (PWM) | GPIO 25 | GPIO 14 |
| Enable gate-driver | GPIO 12 | GPIO 12 |
| Stroom fase A (ADC) | GPIO 39 | GPIO 34 |
| Stroom fase B (ADC) | GPIO 36 | GPIO 35 |
| I²C SDA | GPIO 19 | GPIO 23 |
| I²C SCL | GPIO 18 | GPIO 5 |

---

## Werkingsprincipe

De schets implementeert **voltage-mode sinusoïdale FOC**:

1. **Rotoruitlijning** – bij opstart wordt een kleine spanning aangelegd op een bekende elektrische hoek (0 rad). De rotor trekt naar de bijbehorende positie. Na 3 seconden wordt de mechanische hoek gemeten en de **elektrische offset** berekend.

2. **Regelloop (1 kHz)** – een hardware-timer interrupt zet elke milliseconde een vlag. De `loop()` functie controleert de vlag en voert één regelstap uit:
   - Lees de mechanische rotorhoek via de AS5600 encoder (I²C).
   - Bereken de elektrische hoek:  
     `θ_elec = θ_mech × polePairs + offset + π/2`  
     De π/2 offset zorgt dat het magnetisch veld loodrecht op de rotor staat voor maximaal koppel.
   - Genereer sinusoïdaal drie-fase PWM op basis van `θ_elec`.

3. **PWM-generatie** – de drie fases zijn 120° uit fase verschoven. De bipolaire sinuswaarden worden verschoven naar het unipolaire PWM-bereik `[0, PWM_MAX]`:  
   `pwm = (sin(θ) × 0.5 + 0.5) × 1023`

### PWM-instellingen

| Parameter | Waarde |
|---|---|
| Frequentie | 20 kHz (boven hoorbaar bereik) |
| Resolutie | 10-bit (0–1023) |

---

## Bestandsstructuur

```
balancing_robot_basic_motor_test/
├── balancing_robot_basic_motor_test.ino   # Setup, loop, timer-ISR, motor-instanties
├── Motor.h                                # Klasse-declaratie + constanten
└── Motor.cpp                              # Implementatie FOC-logica
```

### Klasse-overzicht `Motor`

| Methode | Beschrijving |
|---|---|
| `Motor(u,v,w,en,ia,ib,wire,poles)` | Constructor, stelt pinout en sensor-parameters in |
| `begin()` | Initialiseer PWM, enable gate-driver, voer rotoruitlijning uit |
| `loop(voltage)` | Één regelstap: lees encoder → bereken hoek → zet PWM |
| `alignRotor()` | Bepaal elektrische offset bij opstart (~3 s) |
| `setPhaseVoltage(angle, V)` | Schrijf sinusoïdaal 3-fase PWM |
| `getMechanicalAngle()` | Lees AS5600, converteer naar radialen |
| `readCurrent(pin)` | ADC-waarde → ampère via shunt + versterking |
| `updateCurrent()` | Filter stromen van alle drie fases (EMA, α=0.1) |

---

## Installatie & gebruik

### Vereisten
- **Arduino IDE 2.x** met ESP32-boardpakket geïnstalleerd  
  *(Beheer via: Bestand → Voorkeuren → Aanvullende boardmanager-URL's → `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`)*
- Geen extra libraries nodig (alleen ingebouwde `Wire.h`)

### Board-instelling
- **Board**: `ESP32 Dev Module` (of het MKS-ESP32FOC-specifieke boardpakket indien beschikbaar)
- **Upload Speed**: 921600
- **Flash Mode**: QIO

### Eerste gebruik
1. Sluit de MKS-ESP32FOC aan via USB.
2. Open `balancing_robot_basic_motor_test.ino` in de Arduino IDE.
3. Selecteer het juiste board en de COM-poort.
4. Upload de schets.
5. Open de seriële monitor op **115200 baud**.
6. Na ~3 seconden verschijnt de offset-waarde, waarna de motor begint te draaien.

### Motor 2 activeren
Verwijder de commentaar-markeringen (`//`) bij de regels voor `motor2` in het `.ino`-bestand:
```cpp
Motor motor2(26, 27, 14, 12, 34, 35, &I2C_2, 11);
// ↓ in setup():
motor2.begin();
// ↓ in loop():
motor2.loop(-0.2f);  // negatief = tegengestelde rijrichting
```

### Spanning aanpassen
De parameter van `motor1.loop(0.6f)` is de fasespanning in volt. Verhoog voorzichtig:
- Te laag → motor beweegt niet of schokkerig
- Te hoog → overmatige stroom (geen stroombeveiliging in open-loop!)

---

## Toekomstige uitbreidingen

- [ ] Gesloten-lus stroomregeling (FOC met PI-regelaar)
- [ ] Snelheidsregelaar op basis van encoder-differentiaal
- [ ] PID-balansregelaar op basis van BNO055 IMU
- [ ] Bluetooth-besturing via XBOX-controller
- [ ] Accuspanning bewaking via INA228

---

## Licentie

MIT – vrij te gebruiken en aan te passen.
