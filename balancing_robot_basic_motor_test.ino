// ============================================================
//  balancing_robot_basic_motor_test.ino  –  MKS-ESP32FOC V2.0
//
#define USE_IMU           false   // BMI088 accel + gyro (nog niet ontvangen)
#define CALIBRATE_COGGING false   // false = calibrations are already in "cogging_tables.h"
//
//  SPI sensor-bus  →  J5:
//    IO19 = MISO,  IO18 = SCK,  IO22 = MOSI
//  CS via 74HC42  →  J6:
//    GPIO23 = HC42_A0,  GPIO5 = HC42_A1,  GPIO21 = HC42_A2  (3A → GND)
//
//  Motoren (GM4108H-120T, 11 poolparen):
//    Motor 1: U=33 V=32 W=25  EN=12  IA=39 IB=36
//    Motor 2: U=26 V=27 W=14  EN=12  IA=35 IB=34
//
//  Taken:
//    Core 1 – motorTask  (1 kHz, prioriteit 5)
//    Core 0 – slowTask   (1 Hz,  prioriteit 1)
// ============================================================

#include "Motor.h"
#include "mt6835.h"
#include "bmi088.h"
#include "hc42.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#if !CALIBRATE_COGGING
#include "cogging_tables.h"
#endif

#define SENS_SCK   18
#define SENS_MISO  19
#define SENS_MOSI  22

#define SENSOR_HZ        1000
#define MA_SIZE            8 // was 16
#define ALIGN_VOLTAGE_V  0.3f

// Snelheids-PID: output is rijspanning [0..1]
#define SPEED_KP         0.10f  // [V_norm/(rad/s)]
#define SPEED_KI         0.10f
#define SPEED_KD         0.00f
#define SPEED_OUT_MAX    1.0f   // max rijspanning (genormaliseerd)

#define SUPPLY_V         12.0f
#define POLE_PAIRS       11
#define MAX_CURRENT_A    1.0f

SPIClass sensorSPI(VSPI);

Motor motor1(33, 32, 25, 12, 39, 36, POLE_PAIRS, 0);  // MCPWM unit 0
Motor motor2(26, 27, 14, 12, 35, 34, POLE_PAIRS, 1);  // MCPWM unit 1

struct SensorData {
    float enc1_deg,  enc2_deg;
    float enc1_rads, enc2_rads;
    float accel_x, accel_y, accel_z;
    float gyro_x,  gyro_y,  gyro_z;
    float speedSetpoint;
    float driveOut1,  driveOut2;
    float current1,   current2;
    float ia1, ib1, id1, iq1;
    float ia2, ib2, id2, iq2;
};

static portMUX_TYPE  dataMux       = portMUX_INITIALIZER_UNLOCKED;
static SensorData    sharedSensors = {};

// ── Snelheidsberekening met moving-average ────────────────────
static float angleDelta(float cur, float prev) {
    float d = cur - prev;
    if (d >  180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

struct VelocityMA {
    float buf[MA_SIZE] = {};
    float sum           = 0.0f;
    int   idx           = 0;
    int   count         = 0;
    float prevAngle     = -1.0f;

    float update(float angle) {
        float delta = (prevAngle < 0.0f) ? 0.0f : angleDelta(angle, prevAngle);
        prevAngle = angle;
        sum -= buf[idx];
        buf[idx] = delta;
        sum += delta;
        idx = (idx + 1) % MA_SIZE;
        if (count < MA_SIZE) count++;
        return (count < MA_SIZE) ? 0.0f
               : (sum / MA_SIZE) * (float(M_PI) / 180.0f) * SENSOR_HZ;
    }
};

static VelocityMA enc1MA, enc2MA;

// ── Sensors uitlezen (alleen Core 1) ─────────────────────────
static void readSensors(SensorData& d) {
    d.enc1_deg  = mt6835_readAngle(DEV_ENC1);
    d.enc1_rads = enc1MA.update(d.enc1_deg);
    d.enc2_deg  = mt6835_readAngle(DEV_ENC2);
    d.enc2_rads = enc2MA.update(d.enc2_deg);
#if USE_IMU
    bmi088_readAccel(DEV_BMI_ACC, d.accel_x, d.accel_y, d.accel_z);
    bmi088_readGyro (DEV_BMI_GYR, d.gyro_x,  d.gyro_y,  d.gyro_z);
#endif
}

// ── Snelheids-PID ─────────────────────────────────────────────
struct SpeedPID {
    float setpoint = 1.0f; // anti cogging bij setpoint 1.0 is lekker smooth
    float integral = 0.0f;
    float prevMeas = 0.0f;

    float update(float measured) {
        static const float dt = 1.0f / SENSOR_HZ;
        float error  = setpoint - measured;
        integral    += error * dt;
        integral     = constrain(integral,
                                 -SPEED_OUT_MAX / fmaxf(SPEED_KI, 1e-6f),
                                  SPEED_OUT_MAX / fmaxf(SPEED_KI, 1e-6f));
        float deriv  = (measured - prevMeas) / dt;
        prevMeas     = measured;
        return constrain(SPEED_KP * error + SPEED_KI * integral - SPEED_KD * deriv,
                         -SPEED_OUT_MAX, SPEED_OUT_MAX);
    }
};

static SpeedPID speedPID1, speedPID2;
static float    driveVoltage1 = 0.0f, driveVoltage2 = 0.0f;

// ── Core 1: motor-regelloop (1 kHz) ──────────────────────────
void motorTask(void*) {
    SensorData local = {};
    TickType_t lastWake = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1));

        readSensors(local);

        motor1.updateCurrent();
        local.current1 = motor1.getAvgCurrent();
        motor1.readInstantCurrents(local.ia1, local.ib1);
        motor1.computeDQ(local.enc1_deg * DEG_TO_RAD, local.ia1, local.ib1, local.id1, local.iq1);

        motor2.updateCurrent();
        local.current2 = motor2.getAvgCurrent();
        motor2.readInstantCurrents(local.ia2, local.ib2);
        motor2.computeDQ(local.enc2_deg * DEG_TO_RAD, local.ia2, local.ib2, local.id2, local.iq2);

        if (local.current1 > MAX_CURRENT_A) {
            speedPID1.integral = 0.0f;
            driveVoltage1 = 0.0f;
        } else {
            driveVoltage1 = speedPID1.update(local.enc1_rads);
        }
        local.speedSetpoint = speedPID1.setpoint;
        local.driveOut1     = driveVoltage1;

        if (local.current2 > MAX_CURRENT_A) {
            speedPID2.integral = 0.0f;
            driveVoltage2 = 0.0f;
        } else {
            driveVoltage2 = speedPID2.update(local.enc2_rads);
        }
        local.driveOut2 = driveVoltage2;

        float corr1 = motor1.coggingCorrection(local.enc1_deg);
        float corr2 = motor2.coggingCorrection(local.enc2_deg);
        motor1.loop(constrain(driveVoltage1 + corr1, -SPEED_OUT_MAX, SPEED_OUT_MAX),
                    local.enc1_deg * DEG_TO_RAD);
        motor2.loop(constrain(driveVoltage2 + corr2, -SPEED_OUT_MAX, SPEED_OUT_MAX),
                    local.enc2_deg * DEG_TO_RAD);

        portENTER_CRITICAL(&dataMux);
        sharedSensors = local;
        portEXIT_CRITICAL(&dataMux);
    }
}

// ── Core 0: seriële uitvoer (1 Hz) ───────────────────────────
void slowTask(void*) {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        SensorData snap;
        portENTER_CRITICAL(&dataMux);
        snap = sharedSensors;
        portEXIT_CRITICAL(&dataMux);

        char buf[512];
        snprintf(buf, sizeof(buf),
            "m1: %.1f° %.1f r/s  sp=%.1f  Iq=%.3f drv=%.3f  |  "
            "m2: %.1f° %.1f r/s  Iq=%.3f drv=%.3f\r\n",
            snap.enc1_deg, snap.enc1_rads, snap.speedSetpoint,
            snap.iq1, snap.driveOut1,
            snap.enc2_deg, snap.enc2_rads,
            snap.iq2, snap.driveOut2);
        Serial.print(buf);

#if USE_IMU
        snprintf(buf, sizeof(buf),
            "accel=(%.2f,%.2f,%.2f)  gyro=(%.2f,%.2f,%.2f)\r\n",
            snap.accel_x, snap.accel_y, snap.accel_z,
            snap.gyro_x,  snap.gyro_y,  snap.gyro_z);
        Serial.print(buf);
#endif

        static bool calPrinted = false;
        if (!calPrinted) {
            float mA, mB;
            motor1.getAdcMid(mA, mB);
            snprintf(buf, sizeof(buf), "ADC nulpunt m1: IA=%.0f IB=%.0f\r\n", mA, mB);
            Serial.print(buf);
            motor2.getAdcMid(mA, mB);
            snprintf(buf, sizeof(buf), "ADC nulpunt m2: IA=%.0f IB=%.0f\r\n", mA, mB);
            Serial.print(buf);
            calPrinted = true;
        }
    }
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    delay(5'000); // Wacht 5 seconden op een clearscreen te kunnen doen van de uitvoer.

    Serial.print("=== balancing_robot_basic_motor_test ===\r\n");

    sensorSPI.begin(SENS_SCK, SENS_MISO, SENS_MOSI, -1);
    hc42_init();

#if USE_IMU
    bmi088_init(DEV_BMI_ACC, DEV_BMI_GYR);
    Serial.print("BMI088 geinitialiseerd.\r\n");
#endif

    char buf[128];
    float mA, mB;

    motor1.begin();
    motor1.getAdcMid(mA, mB);
    snprintf(buf, sizeof(buf), "Motor1 ADC nulpunt: IA=%.0f IB=%.0f\r\n", mA, mB);
    Serial.print(buf);
    motor1.setPhaseVoltage(0.0f, ALIGN_VOLTAGE_V);
    delay(3000);
    {
        float deg = mt6835_readAngle(DEV_ENC1);
        float off = -(deg * DEG_TO_RAD) * POLE_PAIRS;
        motor1.setOffset(off);
        snprintf(buf, sizeof(buf), "Motor1 uitlijning: %.2f°  offset=%.4f rad\r\n", deg, off);
        Serial.print(buf);
    }

    motor2.begin();
    motor2.getAdcMid(mA, mB);
    snprintf(buf, sizeof(buf), "Motor2 ADC nulpunt: IA=%.0f IB=%.0f\r\n", mA, mB);
    Serial.print(buf);
    motor2.setPhaseVoltage(0.0f, ALIGN_VOLTAGE_V);
    delay(3000);
    {
        float deg = mt6835_readAngle(DEV_ENC2);
        float off = -(deg * DEG_TO_RAD) * POLE_PAIRS;
        motor2.setOffset(off);
        snprintf(buf, sizeof(buf), "Motor2 uitlijning: %.2f°  offset=%.4f rad\r\n", deg, off);
        Serial.print(buf);
    }

    // ── Cogging kalibratie ────────────────────────────────────────
    // Rijdt elke motor 8 seconden op 1 r/s; bouwt per-graad spanningskaart.
    auto calibrateMotor = [&](Motor& mot, uint8_t encDev, const char* label) {
        const float CAL_SPEED = 1.0f;   // r/s
        const float GATE      = 0.35f;  // accepteer ±35% van setpoint
        Serial.printf("%s cogging kalibratie (8s op %.0f r/s)...\r\n", label, CAL_SPEED);
        mot.calibrateStart();
        VelocityMA calMA;
        float integral = 0.0f;
        uint32_t tStart = millis();
        while (millis() - tStart < 8000) {
            float deg  = mt6835_readAngle(encDev);
            float rads = calMA.update(deg);
            float err  = CAL_SPEED - rads;
            integral   = constrain(integral + err / SENSOR_HZ,
                                   -SPEED_OUT_MAX / fmaxf(SPEED_KI, 1e-6f),
                                    SPEED_OUT_MAX / fmaxf(SPEED_KI, 1e-6f));
            float drv  = constrain(SPEED_KP * err + SPEED_KI * integral,
                                   -SPEED_OUT_MAX, SPEED_OUT_MAX);
            mot.loop(drv, deg * DEG_TO_RAD);
            if (fabsf(rads - CAL_SPEED) < GATE * CAL_SPEED && rads > 0.1f)
                mot.calibrateStep(deg, drv);
            delay(1);
        }
        mot.loop(0.0f, mt6835_readAngle(encDev) * DEG_TO_RAD);  // stop
        mot.calibrateFinish();
        snprintf(buf, sizeof(buf), "%s klaar  amplitude=%.4f V_norm\r\n",
                 label, mot.coggingAmplitude());
        Serial.print(buf);
    };

  #if CALIBRATE_COGGING
    calibrateMotor(motor1, DEV_ENC1, "Motor1");
    motor1.printCoggingTable("motor1");
    calibrateMotor(motor2, DEV_ENC2, "Motor2");
    motor2.printCoggingTable("motor2");
  #else
    motor1.loadCoggingTable(cogging_motor1, 360);
    motor2.loadCoggingTable(cogging_motor2, 360);
  #endif

    xTaskCreatePinnedToCore(motorTask, "motor", 4096, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(slowTask,  "slow",  4096, nullptr, 1, nullptr, 0);

    Serial.print("Taken gestart.\r\n");
}

// ── Loop (staat stil; werk zit in taken) ─────────────────────
void loop() {
    vTaskDelay(portMAX_DELAY);
}
