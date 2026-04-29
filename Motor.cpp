
#include "Motor.h"

Motor::Motor(int u, int v, int w, int en, int ia, int ib, int poles)
{
    U = u; V = v; W = w;
    EN = en;
    IA_PIN = ia;
    IB_PIN = ib;
    polePairs = poles;
    offset = 0.0f;
    avgIa = avgIb = avgIc = avgI = 0.0f;
    shunt  = 0.01f;
    gain   = 50.0f;
    adcMid = 1845.0f;
}

void Motor::begin() {
    pinMode(EN, OUTPUT);
    digitalWrite(EN, HIGH);
    ledcAttach(U, PWM_FREQ, PWM_RES);
    ledcAttach(V, PWM_FREQ, PWM_RES);
    ledcAttach(W, PWM_FREQ, PWM_RES);
}

void Motor::setOffset(float o) {
    offset = o;
}

void Motor::setPhaseVoltage(float angle, float voltage) {
    float a = sinf(angle)             * voltage;
    float b = sinf(angle - 2.0943951f) * voltage;
    float c = sinf(angle - 4.1887902f) * voltage;

    int pwmA = constrain((int)((a * 0.5f + 0.5f) * PWM_MAX), 0, PWM_MAX);
    int pwmB = constrain((int)((b * 0.5f + 0.5f) * PWM_MAX), 0, PWM_MAX);
    int pwmC = constrain((int)((c * 0.5f + 0.5f) * PWM_MAX), 0, PWM_MAX);

    ledcWrite(U, pwmA);
    ledcWrite(V, pwmB);
    ledcWrite(W, pwmC);
}

void Motor::loop(float voltage, float mechAngle) {
    float elec = mechAngle * polePairs + offset + PI / 2.0f;
    setPhaseVoltage(elec, voltage);
}

float Motor::readCurrent(int pin) {
    int raw = analogRead(pin);
    float voltage = (raw - adcMid) * (3.3f / 4095.0f);
    return voltage / (shunt * gain);
}

void Motor::updateCurrent() {
    float Ia = readCurrent(IA_PIN);
    float Ib = readCurrent(IB_PIN);
    float Ic = -Ia - Ib;
    avgIa = 0.9f * avgIa + 0.1f * fabsf(Ia);
    avgIb = 0.9f * avgIb + 0.1f * fabsf(Ib);
    avgIc = 0.9f * avgIc + 0.1f * fabsf(Ic);
    avgI  = avgIa + avgIb + avgIc;
}
