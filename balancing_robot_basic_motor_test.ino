/*
 * balancing_robot_basic_motor_test.ino
 *
 * Testschets voor open-loop sinusoïdale FOC-aansturing van BLDC-motoren.
 * Doel: valideren dat de motoren correct draaien vóór integratie in
 * het volledige zelfbalancerende-robot project.
 *
 * Board  : MKS-ESP32FOC V2.0 (ESP32)
 * Motoren: 2× GM4108H-120T  (11 poolparen, ~20 V)
 * Encoder: AS5600 magnetische absolute encoder per motor (I²C)
 *
 * Regelloop loopt op 1 kHz via een hardware-timer interrupt.
 */

#include "Motor.h"

// ─── Opmerking voor ESP32-S3-VROOM ────────────────────────────────────────────
// Op de ESP32-S3-VROOM is de USB-CDC seriële poort Serial0 i.p.v. Serial.
// Verwijder de commentaar-markering hieronder als je die chip gebruikt.
//
//#define Serial Serial0
//
// ──────────────────────────────────────────────────────────────────────────────

// Hardware-timer handle; wordt ingesteld in setup()
hw_timer_t *motorTimer = NULL;

// Mutex voor veilige toegang tot gedeelde variabelen tussen ISR en hoofdloop.
// Niet strikt nodig voor één bool, maar goed patroon voor uitbreiding.
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// ─── I²C-bussen ───────────────────────────────────────────────────────────────
// De ESP32 heeft twee hardware-I²C-controllers (0 en 1).
// Elke AS5600-encoder heeft hetzelfde adres (0x36), dus we gebruiken
// aparte bussen om adresconflicten te vermijden.
TwoWire I2C_1 = TwoWire(0);   // Bus 0: SDA=19, SCL=18 → motor 1
TwoWire I2C_2 = TwoWire(1);   // Bus 1: SDA=23, SCL=5  → motor 2

// ─── Motor-objecten ───────────────────────────────────────────────────────────
// Motor(U, V, W, EN, IA, IB, &wire, polePairs)
//   U/V/W      : PWM-pins voor de drie fases
//   EN         : enable-pin van de gate-driver (HIGH = actief)
//   IA/IB      : ADC-pins voor stroommeters (fase A en B; fase C = -A-B)
//   &wire      : I²C-bus voor de AS5600-encoder
//   polePairs  : aantal poolparen van de motor (GM4108H-120T = 11)
Motor motor1(33, 32, 25, 12, 39, 36, &I2C_1, 11);
//Motor motor2(26, 27, 14, 12, 34, 35, &I2C_2, 11);  // uitgecommentarieerd tijdens eenmotor-test

// ─── Timer-interrupt vlag ─────────────────────────────────────────────────────
// De ISR schrijft alleen deze vlag; de zware berekeningen gebeuren in loop().
// 'volatile' zorgt dat de compiler de waarde niet wegoptimaliseert.
volatile bool motorFlag = false;

// Interrupt Service Routine – wordt elke 1 ms aangeroepen (1 kHz).
// IRAM_ATTR plaatst de ISR in interne RAM zodat hij ook werkt tijdens
// flash-cache-misses.
void IRAM_ATTR onMotorTimer() {
  motorFlag = true;
}


void setup() {
  // Seriële poort voor debug-output (offset-waarde bij opstart)
  Serial.begin(115200);
  while (!Serial) { delay(10); }   // wacht tot USB-verbinding gereed is

  // ─── Hardware-timer configureren ────────────────────────────────────────────
  // timerBegin(1000000) → tijdbasis van 1 MHz (één tick = 1 µs)
  // timerAlarm(timer, 1000, true, 0) → alarm elke 1000 µs = 1 ms = 1 kHz
  motorTimer = timerBegin(1000000);
  timerAttachInterrupt(motorTimer, &onMotorTimer);
  timerAlarm(motorTimer, 1000, true, 0);  // 1000 ticks × 1 µs = 1 ms interval

  // ─── I²C-bussen starten ─────────────────────────────────────────────────────
  // begin(SDA, SCL, frequentie)
  // 400 kHz (Fast Mode) is vereist voor vlotte AS5600-uitlezing op 1 kHz
  I2C_1.begin(19, 18, 400000);
  I2C_2.begin(23,  5, 400000);

  // Motor initialiseren: PWM instellen + rotor uitlijnen (duurt ~3 s)
  motor1.begin();
  //motor2.begin();
}


void loop() {
  // Wacht op de 1 kHz timer-vlag; alle andere code buiten de if blijft
  // niet-blokkend zodat toekomstige taken (BT, IMU, PID) hier ingevoegd kunnen worden
  if (motorFlag) {
    motorFlag = false;   // vlag direct resetten zodat we de volgende cyclus niet missen

    // Stuur motor 1 met 0.6 V fasespanning (open-loop, positieve richting).
    // Verhoog dit getal voorzichtig; te hoge waarden zonder stroombeveiliging
    // kunnen de motor of driver beschadigen.
    motor1.loop(0.6f);

    // Motor 2: negatieve waarde = tegengestelde rijrichting (voor een rijdende robot)
    //motor2.loop(-0.2f);
  }
}
