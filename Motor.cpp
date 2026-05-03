
#include "Motor.h"

static adc1_channel_t gpioToAdc1(int gpio) {
    switch (gpio) {
        case 36: return ADC1_CHANNEL_0;
        case 37: return ADC1_CHANNEL_1;
        case 38: return ADC1_CHANNEL_2;
        case 39: return ADC1_CHANNEL_3;
        case 32: return ADC1_CHANNEL_4;
        case 33: return ADC1_CHANNEL_5;
        case 34: return ADC1_CHANNEL_6;
        case 35: return ADC1_CHANNEL_7;
        default: return ADC1_CHANNEL_0;
    }
}

Motor::Motor(int u, int v, int w, int en, int ia, int ib, int poles, int mcpwm_group)
{
    U = u; V = v; W = w;
    EN = en;
    IA_PIN = ia; IB_PIN = ib;
    polePairs  = poles;
    mcpwmGroup = mcpwm_group;
    offset = 0.0f;
    avgIa = avgIb = avgIc = avgI = 0.0f;
    shunt   = 0.01f;
    gain    = 50.0f;
    adcMidA = adcMidB = 2048.0f;
    adcChanA_ = gpioToAdc1(ia);
    adcChanB_ = gpioToAdc1(ib);
}

void Motor::begin() {
    // ── ADC kalibratie vóór driver inschakelen ────────────────
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(adcChanA_, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(adcChanB_, ADC_ATTEN_DB_11);

    long sumA = 0, sumB = 0;
    for (int i = 0; i < 256; i++) {
        sumA += adc1_get_raw(adcChanA_);
        sumB += adc1_get_raw(adcChanB_);
        delayMicroseconds(50);
    }
    adcMidA = sumA / 256.0f;
    adcMidB = sumB / 256.0f;

    // ── Semaphore voor ADC-synchronisatie ─────────────────────
    adcSem_ = xSemaphoreCreateBinary();

    // ── EN pin ────────────────────────────────────────────────
    pinMode(EN, OUTPUT);
    digitalWrite(EN, HIGH);

    // ── MCPWM timer: center-aligned, 20 kHz ──────────────────
    // ESP-IDF v5 UP_DOWN: peak_ticks = period_ticks/2, dus period_ticks = PWM_RES_TICKS*2
    // zodat peak = PWM_RES_TICKS en geldige compare-range [0, PWM_RES_TICKS-1].
    // Frequentie: 80 MHz / (2 × PWM_RES_TICKS) = 20 kHz
    mcpwm_timer_config_t timer_cfg = {};
    timer_cfg.group_id      = mcpwmGroup;
    timer_cfg.clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_cfg.resolution_hz = 80000000;
    timer_cfg.count_mode    = MCPWM_TIMER_COUNT_MODE_UP_DOWN;
    timer_cfg.period_ticks  = PWM_RES_TICKS * 2;
    mcpwm_new_timer(&timer_cfg, &timer_);

    // ── Operators: één per fase (3 per group) ─────────────────
    mcpwm_operator_config_t oper_cfg = {};
    oper_cfg.group_id = mcpwmGroup;
    mcpwm_new_operator(&oper_cfg, &operU_);
    mcpwm_operator_connect_timer(operU_, timer_);
    mcpwm_new_operator(&oper_cfg, &operV_);
    mcpwm_operator_connect_timer(operV_, timer_);
    mcpwm_new_operator(&oper_cfg, &operW_);
    mcpwm_operator_connect_timer(operW_, timer_);

    // ── Comparators: vergelijkwaarde ververst op timer-zero ───
    mcpwm_comparator_config_t cmp_cfg = {};
    cmp_cfg.flags.update_cmp_on_tez = true;
    mcpwm_new_comparator(operU_, &cmp_cfg, &cmpU_);
    mcpwm_new_comparator(operV_, &cmp_cfg, &cmpV_);
    mcpwm_new_comparator(operW_, &cmp_cfg, &cmpW_);
    mcpwm_comparator_set_compare_value(cmpU_, PWM_RES_TICKS / 2);
    mcpwm_comparator_set_compare_value(cmpV_, PWM_RES_TICKS / 2);
    mcpwm_comparator_set_compare_value(cmpW_, PWM_RES_TICKS / 2);

    // ── Generators (GPIO outputs) ─────────────────────────────
    {
        mcpwm_generator_config_t cfg = {};
        cfg.gen_gpio_num = U; mcpwm_new_generator(operU_, &cfg, &genU_);
        cfg.gen_gpio_num = V; mcpwm_new_generator(operV_, &cfg, &genV_);
        cfg.gen_gpio_num = W; mcpwm_new_generator(operW_, &cfg, &genW_);
    }

    // Center-aligned: HIGH op count-up match, LOW op count-down match
    struct { mcpwm_gen_handle_t gen; mcpwm_cmpr_handle_t cmp; } ph[3] = {
        {genU_, cmpU_}, {genV_, cmpV_}, {genW_, cmpW_}
    };
    for (auto& p : ph) {
        mcpwm_generator_set_action_on_compare_event(p.gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,   p.cmp, MCPWM_GEN_ACTION_HIGH));
        mcpwm_generator_set_action_on_compare_event(p.gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, p.cmp, MCPWM_GEN_ACTION_LOW));
    }

    // ── ADC-trigger: semaphore geven op timer-zero (TEZ) ──────
    mcpwm_timer_event_callbacks_t cbs = {};
    cbs.on_empty = timerOnEmpty;
    mcpwm_timer_register_event_callbacks(timer_, &cbs, this);

    // ── Timer starten ─────────────────────────────────────────
    mcpwm_timer_enable(timer_);
    mcpwm_timer_start_stop(timer_, MCPWM_TIMER_START_NO_STOP);
}

// ISR: vuurt elke 50 µs op timer-zero; geeft semaphore aan motorTask
bool IRAM_ATTR Motor::timerOnEmpty(mcpwm_timer_handle_t,
                                    const mcpwm_timer_event_data_t*,
                                    void* user_data)
{
    Motor* m = static_cast<Motor*>(user_data);
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(m->adcSem_, &woken);
    return woken == pdTRUE;
}

void Motor::setOffset(float o) { offset = o; }

void Motor::setPhaseVoltage(float angle, float voltage) {
    float a = sinf(angle)              * voltage;
    float b = sinf(angle - 2.0943951f) * voltage;
    float c = sinf(angle - 4.1887902f) * voltage;

    auto toCmp = [](float v) -> uint32_t {
        int t = (int)((v * 0.5f + 0.5f) * PWM_RES_TICKS);
        if (t <= 0)                   return 0;
        if (t >= (int)PWM_RES_TICKS)  return (uint32_t)(PWM_RES_TICKS - 1);
        return (uint32_t)t;
    };
    mcpwm_comparator_set_compare_value(cmpU_, toCmp(a));
    mcpwm_comparator_set_compare_value(cmpV_, toCmp(b));
    mcpwm_comparator_set_compare_value(cmpW_, toCmp(c));
}

void Motor::loop(float voltage, float mechAngle) {
    float elec = mechAngle * polePairs + offset + PI / 2.0f;
    setPhaseVoltage(elec, voltage);
}

float Motor::readCurrentFromRaw(int raw, float mid) const {
    float v = (raw - mid) * (3.3f / 4095.0f);
    return v / (shunt * gain);
}

void Motor::readInstantCurrents(float& ia, float& ib) {
    // Wacht op TEZ-semaphore (max 5 ms); lees ADC direct erna — binnen µs van het rustpunt
    xSemaphoreTake(adcSem_, pdMS_TO_TICKS(5));
    ia = readCurrentFromRaw(adc1_get_raw(adcChanA_), adcMidA);
    ib = readCurrentFromRaw(adc1_get_raw(adcChanB_), adcMidB);
}

void Motor::computeDQ(float mechAngle, float ia, float ib, float& id, float& iq) {
    float thetaE = mechAngle * polePairs + offset + PI / 2.0f;
    float c = cosf(thetaE);
    float s = sinf(thetaE);
    float ialpha = ia;
    float ibeta  = (ia + 2.0f * ib) * 0.57735027f;  // 1/√3
    id =  ialpha * c + ibeta * s;
    iq =  ialpha * s - ibeta * c;
}

// ── Cogging-compensatie ───────────────────────────────────────

void Motor::calibrateStart() {
    memset(coggingSum_,   0, sizeof(coggingSum_));
    memset(coggingCount_, 0, sizeof(coggingCount_));
    memset(coggingTable_, 0, sizeof(coggingTable_));
    coggingReady_ = false;
}

void Motor::calibrateStep(float mechAngleDeg, float driveVoltage) {
    int bin = (int)mechAngleDeg % N_COGGING;
    if (bin < 0) bin += N_COGGING;
    coggingSum_[bin]   += driveVoltage;
    coggingCount_[bin] += 1;
}

void Motor::calibrateFinish() {
    // Gemiddelde per bin
    float mean  = 0;
    int   filled = 0;
    for (int i = 0; i < N_COGGING; i++) {
        if (coggingCount_[i] > 0) {
            coggingTable_[i] = coggingSum_[i] / coggingCount_[i];
            mean += coggingTable_[i];
            filled++;
        }
    }
    if (filled == 0) return;
    mean /= filled;

    // DC-component verwijderen
    for (int i = 0; i < N_COGGING; i++)
        coggingTable_[i] -= mean;

    // Lege bins vullen via lineaire interpolatie van dichtstbijzijnde buren
    for (int i = 0; i < N_COGGING; i++) {
        if (coggingCount_[i] > 0) continue;
        int ld = -1, rd = -1;
        for (int d = 1; d < N_COGGING; d++) {
            if (coggingCount_[(i - d + N_COGGING) % N_COGGING] > 0) { ld = d; break; }
        }
        for (int d = 1; d < N_COGGING; d++) {
            if (coggingCount_[(i + d) % N_COGGING] > 0) { rd = d; break; }
        }
        float vl = (ld >= 0) ? coggingTable_[(i - ld + N_COGGING) % N_COGGING] : 0;
        float vr = (rd >= 0) ? coggingTable_[(i + rd) % N_COGGING] : 0;
        if (ld >= 0 && rd >= 0)
            coggingTable_[i] = vl + (vr - vl) * ld / (ld + rd);
        else
            coggingTable_[i] = (ld >= 0) ? vl : vr;
    }

    coggingReady_ = true;
}

float Motor::coggingCorrection(float mechAngleDeg) const {
    if (!coggingReady_) return 0.0f;
    int bin = (int)mechAngleDeg % N_COGGING;
    if (bin < 0) bin += N_COGGING;
    return coggingTable_[bin];
}

void Motor::printCoggingTable(const char* label) const {
    if (!coggingReady_) {
        Serial.printf("// %s: geen kalibratie beschikbaar\r\n", label);
        return;
    }
    Serial.printf("// %s cogging tabel  amplitude=%.4f V_norm\r\n", label, coggingAmplitude());
    Serial.printf("const float cogging_%s[%d] = {\r\n", label, N_COGGING);
    for (int i = 0; i < N_COGGING; i++) {
        if (i % 8 == 0) Serial.print("    ");
        Serial.printf("%.6ff", coggingTable_[i]);
        if (i < N_COGGING - 1) Serial.print(", ");
        if (i % 8 == 7 || i == N_COGGING - 1) Serial.print("\r\n");
    }
    Serial.print("};\r\n\r\n");
}

void Motor::loadCoggingTable(const float* table, int n) {
    int sz = (n < N_COGGING) ? n : N_COGGING;
    memcpy(coggingTable_, table, sz * sizeof(float));
    if (sz < N_COGGING)
        memset(coggingTable_ + sz, 0, (N_COGGING - sz) * sizeof(float));
    coggingReady_ = true;
}

float Motor::coggingAmplitude() const {
    if (!coggingReady_) return 0.0f;
    float mn = coggingTable_[0], mx = coggingTable_[0];
    for (int i = 1; i < N_COGGING; i++) {
        if (coggingTable_[i] < mn) mn = coggingTable_[i];
        if (coggingTable_[i] > mx) mx = coggingTable_[i];
    }
    return mx - mn;
}

void Motor::updateCurrent() {
    // Niet TEZ-gesynchroniseerd; EMA-filter middelt rimpel weg
    float Ia = readCurrentFromRaw(adc1_get_raw(adcChanA_), adcMidA);
    float Ib = readCurrentFromRaw(adc1_get_raw(adcChanB_), adcMidB);
    float Ic = -Ia - Ib;
    avgIa = 0.9f * avgIa + 0.1f * fabsf(Ia);
    avgIb = 0.9f * avgIb + 0.1f * fabsf(Ib);
    avgIc = 0.9f * avgIc + 0.1f * fabsf(Ic);
    avgI  = avgIa + avgIb + avgIc;
}
