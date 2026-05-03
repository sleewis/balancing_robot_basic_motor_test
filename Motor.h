
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <driver/mcpwm_prelude.h>
#include <driver/adc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Center-aligned 20 kHz: 80 MHz / (2 × 20 000) = 2000 ticks
// ESP-IDF v5 UP_DOWN: peak = period/2, dus period_ticks = PWM_RES_TICKS*2
#define PWM_RES_TICKS  2000

// Cogging-compensatie tabel: 1 bin per graad
#define N_COGGING      360

class Motor {
public:
    Motor(int u, int v, int w, int en, int ia, int ib, int poles, int mcpwm_group);

    void  begin();
    void  setOffset(float o);
    void  setPhaseVoltage(float angle, float voltage);
    void  loop(float voltage, float mechAngle);
    void  updateCurrent();
    float getAvgCurrent() const { return avgI; }
    void  getAdcMid(float& midA, float& midB) const { midA = adcMidA; midB = adcMidB; }
    void  readInstantCurrents(float& ia, float& ib);
    void  computeDQ(float mechAngle, float ia, float ib, float& id, float& iq);

    // Cogging-compensatie
    void  calibrateStart();
    void  calibrateStep(float mechAngleDeg, float driveVoltage);
    void  calibrateFinish();
    float coggingCorrection(float mechAngleDeg) const;
    float coggingAmplitude()  const;   // peak-to-peak voor diagnostiek
    bool  coggingReady()      const { return coggingReady_; }
    void  printCoggingTable(const char* label) const;
    void  loadCoggingTable(const float* table, int n);

private:
    int U, V, W, EN, IA_PIN, IB_PIN, polePairs, mcpwmGroup;
    float offset;
    float avgIa, avgIb, avgIc, avgI;
    float shunt, gain, adcMidA, adcMidB;

    mcpwm_timer_handle_t  timer_  = nullptr;
    mcpwm_oper_handle_t   operU_  = nullptr;
    mcpwm_oper_handle_t   operV_  = nullptr;
    mcpwm_oper_handle_t   operW_  = nullptr;
    mcpwm_cmpr_handle_t   cmpU_   = nullptr, cmpV_ = nullptr, cmpW_ = nullptr;
    mcpwm_gen_handle_t    genU_   = nullptr, genV_ = nullptr, genW_ = nullptr;

    adc1_channel_t    adcChanA_, adcChanB_;
    SemaphoreHandle_t adcSem_ = nullptr;

    // Cogging interne data
    float coggingSum_[N_COGGING]   = {};
    int   coggingCount_[N_COGGING] = {};
    float coggingTable_[N_COGGING] = {};
    bool  coggingReady_            = false;

    float readCurrentFromRaw(int raw, float mid) const;

    static bool IRAM_ATTR timerOnEmpty(mcpwm_timer_handle_t timer,
                                        const mcpwm_timer_event_data_t* edata,
                                        void* user_data);
};

#endif
