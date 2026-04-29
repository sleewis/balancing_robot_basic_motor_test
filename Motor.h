
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

#define PWM_FREQ 20000
#define PWM_RES  10
#define PWM_MAX  1023

class Motor {
public:
    Motor(int u, int v, int w, int en, int ia, int ib, int poles);

    void begin();
    void setOffset(float o);
    void setPhaseVoltage(float angle, float voltage);
    void loop(float voltage, float mechAngle);

private:
    int U, V, W, EN, IA_PIN, IB_PIN;
    int polePairs;
    float offset;
    float avgIa, avgIb, avgIc, avgI;
    float shunt, gain, adcMid;

    float readCurrent(int pin);
    void  updateCurrent();
};

#endif
