/*
 * Motor.h
 *
 * Klasse-declaratie voor sinusoïdale FOC-aansturing (voltage-mode) van één
 * drie-fase BLDC-motor met AS5600 magnetische encoder.
 *
 * Werkingsprincipe:
 *   1. Lees de mechanische rotorhoek via de AS5600 (I²C).         ← gesloten lus
 *   2. Bereken de elektrische hoek: θ_elec = θ_mech × polePairs + offset + π/2
 *      (de π/2 offset zorgt dat het magnetisch veld loodrecht op de rotor staat
 *       → maximaal koppel bij gegeven spanning).
 *   3. Genereer sinusoïdaal drie-fase PWM op basis van θ_elec.
 *
 *   De spanningsamplitude is een vaste waarde (voltage-mode / open-loop spanning):
 *   er is geen stroom- of snelheidsregelaar. updateCurrent() is al aanwezig
 *   als bouwsteen voor een toekomstige gesloten-lus stroomregelaar.
 *   Fase C wordt afgeleid als -Ia - Ib (Kirchhoff), niet direct gemeten.
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Wire.h>

// ─── PWM-instellingen ─────────────────────────────────────────────────────────
// 20 kHz ligt boven het hoorbare bereik; vermindert motorgeluid en
// verbetert de stroomripple t.o.v. lagere frequenties.
#define PWM_FREQ   20000   // Hz – PWM-draaggolffrequentie
#define PWM_RES       10   // bit – resolutie (2^10 = 1024 stappen)
#define PWM_MAX     1023   // maximale duty-cycle waarde (2^10 - 1)

// I²C-adres van de AS5600 magnetische encoder (fabrieksinstelling, niet aanpasbaar)
#define AS5600_ADDR 0x36


class Motor {
public:
  /*
   * Constructor
   * u, v, w   : GPIO-pinnummers voor PWM-uitgang naar gate-driver (fase U/V/W)
   * en        : GPIO-pin voor de enable-ingang van de gate-driver (HIGH = aan)
   * ia, ib    : ADC-pinnummers voor stroomsensoren fase A en B
   * wbus      : pointer naar de TwoWire I²C-bus waarop de AS5600 is aangesloten
   * poles     : aantal poolparen van de motor (GM4108H-120T = 11)
   */
  Motor(int u, int v, int w, int en,
        int ia, int ib,
        TwoWire *wbus,
        int poles);

  // Initialiseer GPIO, LEDC-PWM en voer rotoruitlijning uit.
  // Blokkeer ~3 seconden vanwege de uitlijnvertraging.
  void begin();

  // Voer één regelstap uit: lees encoder → bereken elektrische hoek → zet PWM.
  // Roep aan vanuit de 1 kHz timer-interrupt handler (via vlag in loop()).
  // voltage: gewenste fasespanning in volt (positief = vooruit, negatief = achteruit)
  void loop(float voltage);

private:
  // ─── Pin-nummers ────────────────────────────────────────────────────────────
  int U, V, W;        // PWM-uitgangen voor de drie motorfases
  int EN;             // enable-pin gate-driver
  int IA_PIN, IB_PIN; // ADC-ingangen stroomsensoren

  // ─── I²C & encoder ──────────────────────────────────────────────────────────
  TwoWire *wire;      // I²C-bus voor AS5600
  int polePairs;      // aantal poolparen (bepaalt verhouding mech. → elec. hoek)

  float    offset;    // elektrische offsethoek bepaald tijdens rotoruitlijning (rad)
  uint16_t rawAngle;  // laatste ruwe AS5600-waarde (0–4095 = 0–360°)

  // ─── Stroomfilters (exponentieel voortschrijdend gemiddelde) ─────────────────
  // α = 0.1 → tijdconstante ≈ 10 regelstappen = 10 ms bij 1 kHz
  float avgIa, avgIb, avgIc;  // gemiddelde absolute fasestroom (A)
  float avgI;                  // som van alle gemiddelde fasestromen

  // ─── Stroomsensor-kalibratiewaarden ─────────────────────────────────────────
  float shunt;   // shuntweerstand in Ω (0.01 Ω op MKS-ESP32FOC)
  float gain;    // versterkingsfactor van de stroomopversterker (50×)
  float adcMid;  // ADC-waarde bij 0 A (middenpunt van de opversterker, ~1845/4095)

  // ─── Private methoden ────────────────────────────────────────────────────────

  // Lees de ruwe 12-bit hoekwaarde uit de AS5600 via I²C.
  // Geeft true terug als de lezing geslaagd is, false bij I²C-fout.
  bool readSensor();

  // Blokkerende wrapper: roept readSensor() aan tot een geldige waarde beschikbaar is.
  // Converteert de 12-bit ruwe waarde naar radialen (0–2π).
  float getMechanicalAngle();

  // Lees de fase-stroom via ADC.
  // Formule: I = (ADC - adcMid) × (3.3 / 4095) / (shunt × gain)
  float readCurrent(int pin);

  // Bereken en filter de stromen van alle drie fases.
  // Fase C wordt afgeleid: Ic = -Ia - Ib  (somregel Kirchhoff)
  void updateCurrent();

  // Genereer sinusoïdaal drie-fase PWM voor de opgegeven elektrische hoek en spanning.
  // De sinus-waarden worden verschoven van [-1,+1] naar [0,1] vóór PWM-schrijven:
  //   pwm = (sin(angle) × 0.5 + 0.5) × PWM_MAX
  void setPhaseVoltage(float angle, float voltage);

  // Lijn de rotor uit met een bekend magnetisch veld en bepaal de elektrische offset.
  // Procedure: zet spanning op hoek 0 → wacht 3 s → lees mechanische hoek →
  //            offset = 0 - (mechanischeHoek × polePairs)
  void alignRotor();
};

#endif
