/*
 * Motor.cpp
 *
 * Implementatie van de Motor-klasse voor open-loop sinusoïdale FOC.
 * Zie Motor.h voor uitleg over het werkingsprincipe en de parameters.
 */

#include "Motor.h"


// ─── Constructor ──────────────────────────────────────────────────────────────
Motor::Motor(int u, int v, int w, int en,
             int ia, int ib,
             TwoWire *wbus,
             int poles)
{
  // Sla pin-nummers op
  U = u; V = v; W = w;
  EN = en;
  IA_PIN = ia;
  IB_PIN = ib;

  // I²C-bus en motor-eigenschappen
  wire      = wbus;
  polePairs = poles;

  // Beginwaarden; offset wordt correct ingesteld door alignRotor()
  offset = 0;
  avgIa = avgIb = avgIc = avgI = 0;

  // Stroomsensor-parameters voor de MKS-ESP32FOC V2.0:
  //   shunt = 0.01 Ω (precisie shuntweerstand op de PCB)
  //   gain  = 50×  (interne versterker van de stroomopversterker IC)
  //   adcMid = 1845 ≈ 1.5 V op 12-bit ADC bij 3.3 V referentie
  //            (middenpunt opversterker bij 0 A stroom)
  shunt  = 0.01f;
  gain   = 50.0f;
  adcMid = 1845.0f;
}


// ─── begin() ─────────────────────────────────────────────────────────────────
void Motor::begin() {
  // Zet de gate-driver enable-pin hoog zodat de MOSFET-brug actief wordt
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);

  // Configureer LEDC (LED Control) PWM voor de drie fase-pins.
  // ledcAttach(pin, frequentie, resolutie) is de ESP32-Arduino API v3+ stijl.
  // Alle drie fases draaien op dezelfde frequentie en resolutie voor
  // synchrone three-phase generatie.
  ledcAttach(U, PWM_FREQ, PWM_RES);
  ledcAttach(V, PWM_FREQ, PWM_RES);
  ledcAttach(W, PWM_FREQ, PWM_RES);

  // Voer rotoruitlijning uit om de elektrische offset te bepalen.
  // Dit blokkeert ~3 seconden en print de offset naar de seriële monitor.
  alignRotor();
}


// ─── readSensor() ────────────────────────────────────────────────────────────
bool Motor::readSensor() {
  // Start I²C-transmissie naar de AS5600 en vraag register 0x0C aan.
  // Register 0x0C = ANGLE_HIGH (bits 11..8), gevolgd door 0x0D = ANGLE_LOW (bits 7..0).
  // endTransmission(false) stuurt een herhaalde START in plaats van een STOP,
  // zodat we direct kunnen lezen zonder de bus vrij te geven.
  wire->beginTransmission(AS5600_ADDR);
  wire->write(0x0C);
  wire->endTransmission(false);

  // Vraag 2 bytes op (hoog- en laagbyte van de 12-bit hoek)
  wire->requestFrom(AS5600_ADDR, 2);
  if (wire->available() == 2) {
    uint16_t high = wire->read();   // bits 11–8 (hogere nibble)
    uint16_t low  = wire->read();   // bits  7–0 (laagste byte)
    // Combineer tot 12-bit waarde; maskeer de ongebruikte hogere bits
    rawAngle = ((high << 8) | low) & 0x0FFF;
    return true;
  }
  return false;  // I²C-fout: sensor niet gevonden of communicatiestoring
}


// ─── getMechanicalAngle() ────────────────────────────────────────────────────
float Motor::getMechanicalAngle() {
  // Blijf proberen tot de sensor een geldige waarde geeft.
  // In normale werking slaagt dit vrijwel altijd op de eerste poging.
  while (!readSensor());

  // Converteer de 12-bit waarde (0–4095) naar radialen (0–2π):
  //   hoek [rad] = rawAngle × (2π / 4096)
  return rawAngle * (TWO_PI / 4096.0f);
}


// ─── readCurrent() ────────────────────────────────────────────────────────────
float Motor::readCurrent(int pin) {
  // Lees de ruwe 12-bit ADC-waarde (0–4095 bij 3.3 V referentie)
  int raw = analogRead(pin);

  // Bereken de spanning over de shunt-versterker-uitgang:
  //   V_out = (raw - adcMid) × (3.3 V / 4095)
  // adcMid is de ADC-waarde bij 0 A (middenpunt van de opversterker)
  float voltage = (raw - adcMid) * (3.3f / 4095.0f);

  // Bereken de stroom via Ohm's wet met de versterking:
  //   I = V_shunt / R_shunt  en  V_shunt = V_out / gain
  //   → I = V_out / (gain × shunt)
  return voltage / (shunt * gain);
}


// ─── updateCurrent() ─────────────────────────────────────────────────────────
void Motor::updateCurrent() {
  float Ia = readCurrent(IA_PIN);
  float Ib = readCurrent(IB_PIN);

  // Fase C wordt afgeleid via de somregel van Kirchhoff:
  // In een ster-verbinding geldt altijd: Ia + Ib + Ic = 0
  float Ic = -Ia - Ib;

  // Exponentieel voortschrijdend gemiddelde (EMA) op de absolute stroomwaarden.
  // Formule: avg = α_oud × 0.9 + α_nieuw × 0.1
  // Bij 1 kHz regelloop: tijdconstante ≈ 10 ms
  avgIa = 0.9f * avgIa + 0.1f * fabsf(Ia);
  avgIb = 0.9f * avgIb + 0.1f * fabsf(Ib);
  avgIc = 0.9f * avgIc + 0.1f * fabsf(Ic);

  // Totale gemiddelde stroom (som van alle fases, bruikbaar voor overstroomdetectie)
  avgI = avgIa + avgIb + avgIc;
}


// ─── setPhaseVoltage() ───────────────────────────────────────────────────────
void Motor::setPhaseVoltage(float angle, float voltage) {
  // Bereken de sinusoïdale spanning per fase.
  // De drie fases zijn 120° (= 2π/3 ≈ 2.0944 rad) uit fase verschoven.
  float a = sinf(angle)              * voltage;  // Fase U (0°)
  float b = sinf(angle - 2.0943951f) * voltage;  // Fase V (120°)
  float c = sinf(angle - 4.1887902f) * voltage;  // Fase W (240°)

  // Verschuif de bipolaire sinus [-1, +1] naar het unipolaire PWM-bereik [0, PWM_MAX].
  // Formule: pwm = (sin × 0.5 + 0.5) × PWM_MAX
  // Bij voltage = 0 → alle PWM op 50% duty-cycle (geen netto spanning over motor)
  int pwmA = (int)((a * 0.5f + 0.5f) * PWM_MAX);
  int pwmB = (int)((b * 0.5f + 0.5f) * PWM_MAX);
  int pwmC = (int)((c * 0.5f + 0.5f) * PWM_MAX);

  // Begrens de waarden voor het geval drijvende-komma afrondingsfouten
  // buiten het geldige bereik vallen
  pwmA = constrain(pwmA, 0, PWM_MAX);
  pwmB = constrain(pwmB, 0, PWM_MAX);
  pwmC = constrain(pwmC, 0, PWM_MAX);

  // Schrijf de PWM-duty-cycle naar de drie fase-pins via LEDC
  ledcWrite(U, pwmA);
  ledcWrite(V, pwmB);
  ledcWrite(W, pwmC);
}


// ─── loop() ──────────────────────────────────────────────────────────────────
void Motor::loop(float voltage) {
  // Stap 1: lees de huidige mechanische rotorhoek van de AS5600 (0–2π rad)
  float mech = getMechanicalAngle();

  // Stap 2: bereken de elektrische hoek
  //   θ_elec = θ_mech × polePairs + offset + π/2
  //
  //   × polePairs : één mechanische omwenteling = polePairs elektrische perioden
  //   + offset    : corrector voor de nulmeting tijdens alignRotor()
  //   + π/2       : 90° vooruitlopen zodat het statorveld loodrecht op de
  //                 rotormagneten staat → maximaal koppel per volt
  float elec = mech * polePairs + offset + PI / 2.0f;

  // Stap 3: zet de berekende spanning om in sinusoïdaal PWM
  setPhaseVoltage(elec, voltage);
}


// ─── alignRotor() ────────────────────────────────────────────────────────────
void Motor::alignRotor() {
  // Uitlijningsprocedure om de elektrische nulhoek te bepalen:
  //
  // Probleem: de AS5600 meet de absolute mechanische hoek, maar we weten niet
  // welke mechanische positie overeenkomt met elektrische hoek 0.
  // Die informatie is nodig om het statorveld correct op de rotor uit te lijnen.
  //
  // Oplossing:
  //   1. Stuur een kleine spanning op een bekende elektrische hoek (0 rad).
  //      De rotor trekt naar de bijbehorende magnetische positie.
  //   2. Wacht 3 seconden tot de rotor stil staat.
  //   3. Lees de mechanische hoek die de rotor heeft aangenomen.
  //   4. Bereken de offset: offset = elektrischeHoek - mechanischeHoek × polePairs

  float alignAngle = 0.0f;  // bekende elektrische hoek voor uitlijning (0 rad)

  // Zet een lage uitrichtingsspanning (0.3 V) om de rotor zacht aan te trekken
  // zonder te veel stroom te trekken tijdens de stilstand
  setPhaseVoltage(alignAngle, 0.3f);
  delay(3000);  // wacht tot rotor volledig tot stilstand is gekomen

  // Lees de mechanische hoek die de rotor heeft aangenomen
  float mech = getMechanicalAngle();

  // Sla de offset op; deze wordt gebruikt in elke loop()-aanroep
  offset = alignAngle - mech * polePairs;

  // Rapporteer de offset zodat je kunt controleren of de sensor correct werkt.
  // Typische waarden liggen tussen -2π en +2π.
  Serial.print("Offset: ");
  Serial.println(offset);
}
