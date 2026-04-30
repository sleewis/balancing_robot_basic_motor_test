// ============================================================
//  balancing_robot_basic_motor_test.ino  –  MKS-ESP32FOC V2.0
// 
//  Arduino IDE: Tools → Partition Scheme → Minimal SPIFFS (1.9MB APP with OTA/OTA)
//
//  Zet per sensor/motor op true als die is aangesloten:
#define USE_ENC1    true    // MT6835 encoder motor 1
#define USE_ENC2    false   // MT6835 encoder motor 2
#define USE_IMU     false   // BMI088 accel + gyro
#define USE_MOTOR1  true    // motor 1 aansturen
#define USE_MOTOR2  false   // motor 2 aansturen
//
//  WiFi / OTA:
#define WIFI_SSID      ""
#define WIFI_PASS      ""
#define OTA_HOSTNAME   "mks-balancing"
#define WIFI_TIMEOUT_S 10   // seconden wachten op WiFi; daarna verder zonder OTA
//
//  SPI sensor-bus  →  J5:
//    3.3V
//    GND
//    IO19 (SDA) = MISO BLAUW
//    IO18 (SCL) = SCK  WIT
//    IO22 (I_0) = MOSI GEEL
//
//  CS-lijnen  →  J6:
//    3.3V
//    GND
//    IO23 (SDA) = CS encoder 1
//    IO5  (SCL) = CS encoder 2
//    IO21 (I_1) = CS BMI088 accel
//    IO4        = CS BMI088 gyro   ← TODO: herindelen na 74HC42
//
//  Motoren (GM4108H-120T, 11 poolparen):
//    Motor 1: U=33 V=32 W=25  EN=12  IA=39 IB=36
//    Motor 2: U=26 V=27 W=14  EN=12  IA=34 IB=35
//
//  Taken:
//    Core 1 – motorTask  (1 kHz, prioriteit 5)
//    Core 0 – slowTask   (1 Hz,  prioriteit 1, seriële uitvoer)
//    Core 0 – otaTask    (50 ms, prioriteit 1, WiFi OTA)
// ============================================================

#include "Motor.h"
#include "mt6835.h"
#include "bmi088.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WiFiServer.h>
#include <WiFiClient.h>

// ── SPI sensor-bus ────────────────────────────────────────────
#define SENS_SCK   18
#define SENS_MISO  19
#define SENS_MOSI  22

// ── CS-lijnen ─────────────────────────────────────────────────
// TODO: vervangen door 74HC42 decoder zodra chip beschikbaar is
#define CS_ENC1     23   // J6 SDA
#define CS_ENC2      5   // J6 SCL
#define CS_BMI_ACC  21   // J6 I_1
#define CS_BMI_GYR   4   // vrije pin – definitief toewijzen na 74HC42

#define SENSOR_HZ        1000   // regelloop-frequentie [Hz]
#define MA_SIZE            16   // moving-average venster voor snelheid [samples = 16 ms]
#define ALIGN_VOLTAGE_V  0.3f

//  Als hij draait maar overshoort of oscilleert → Kp omlaag.
//  Als hij traag opkomt → Kp omhoog.
//  Als hij de 10 rad/s niet haalt in steady-state → Ki helpt dat.
#define SPEED_KP         0.05f  // proportionele gain  – afstellen na testen
#define SPEED_KI         0.10f  // integrerende gain   – afstellen na testen
#define SPEED_KD         0.00f  // differentiërende gain
#define SPEED_OUT_MAX    1.5f   // maximale rijspanning [V]
#define POLE_PAIRS       11

SPIClass sensorSPI(VSPI);

// ── Motor-objecten ────────────────────────────────────────────
#if USE_MOTOR1
Motor motor1(33, 32, 25, 12, 39, 36, POLE_PAIRS);
#endif
#if USE_MOTOR2
Motor motor2(26, 27, 14, 12, 34, 35, POLE_PAIRS);
#endif

// ── SensorData ────────────────────────────────────────────────
struct SensorData {
    float enc1_deg,  enc2_deg;
    float enc1_rads, enc2_rads;
    float accel_x, accel_y, accel_z;
    float gyro_x,  gyro_y,  gyro_z;
    float speedSetpoint, driveOut;  // PID telemetrie
};

// ── Telnet (alleen Core 0) ────────────────────────────────────
static WiFiServer  telnetServer(23);
static WiFiClient  telnetClient;

// ── Gedeelde data (Core 1 schrijft, Core 0 leest) ─────────────
static portMUX_TYPE     dataMux       = portMUX_INITIALIZER_UNLOCKED;
static SensorData       sharedSensors = {};
static volatile bool    otaActive     = false;

// ── Snelheidsberekening met moving-average ────────────────────
// Alleen toegankelijk vanuit Core 1 (motorTask) – geen lock nodig.

static float angleDelta(float cur, float prev) {
    float d = cur - prev;
    if (d >  180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

// Ringbuffer van hoekdeltas; geeft 0 terug zolang de buffer niet vol is.
struct VelocityMA {
    float buf[MA_SIZE] = {};
    float sum           = 0.0f;
    int   idx           = 0;
    int   count         = 0;
    float prevAngle     = -1.0f;  // -1 = nog niet geïnitialiseerd

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
// Alleen actieve sensoren worden gelezen; geen simulatiedata.
static void readSensors(SensorData& d) {
#if USE_ENC1
    d.enc1_deg  = mt6835_readAngle(CS_ENC1);
    d.enc1_rads = enc1MA.update(d.enc1_deg);
#endif
#if USE_ENC2
    d.enc2_deg  = mt6835_readAngle(CS_ENC2);
    d.enc2_rads = enc2MA.update(d.enc2_deg);
#endif
#if USE_IMU
    bmi088_readAccel(CS_BMI_ACC, d.accel_x, d.accel_y, d.accel_z);
    bmi088_readGyro (CS_BMI_GYR, d.gyro_x,  d.gyro_y,  d.gyro_z);
#endif
}

// ── Snelheids-PID (Core 1 only) ──────────────────────────────
struct SpeedPID {
    float setpoint = -10.0f;  // testwaarde: -10 rad/s = achteruit
    float integral = 0.0f;
    float prevMeas = 0.0f;

    float update(float measured) {
        static const float dt = 1.0f / SENSOR_HZ;
        float error  = setpoint - measured;
        integral    += error * dt;
        // Anti-windup: begrens het integraal-aandeel
        integral     = constrain(integral,
                                 -SPEED_OUT_MAX / fmaxf(SPEED_KI, 1e-6f),
                                  SPEED_OUT_MAX / fmaxf(SPEED_KI, 1e-6f));
        float deriv  = (measured - prevMeas) / dt;
        prevMeas     = measured;
        return constrain(SPEED_KP * error + SPEED_KI * integral - SPEED_KD * deriv,
                         -SPEED_OUT_MAX, SPEED_OUT_MAX);
    }
};
static SpeedPID speedPID;
static float    driveVoltage = 0.0f;

// ── Core 1: motor-regelloop (1 kHz) ──────────────────────────
void motorTask(void*) {
    SensorData local = {};
    TickType_t lastWake = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1));

        if (otaActive) continue;  // motoren zijn gestopt in OTA onStart callback

        readSensors(local);

#if USE_ENC1
        driveVoltage        = speedPID.update(local.enc1_rads);
        local.speedSetpoint = speedPID.setpoint;
        local.driveOut      = driveVoltage;
#endif
#if USE_ENC2
        static SpeedPID speedPID2;
        static float    driveVoltage2 = 0.0f;
        driveVoltage2 = speedPID2.update(local.enc2_rads);
#endif

#if USE_MOTOR1 && USE_ENC1
        motor1.loop(driveVoltage, local.enc1_deg * DEG_TO_RAD);
#endif
#if USE_MOTOR2 && USE_ENC2
        motor2.loop(driveVoltage2, local.enc2_deg * DEG_TO_RAD);
#endif

        portENTER_CRITICAL(&dataMux);
        sharedSensors = local;
        portEXIT_CRITICAL(&dataMux);
    }
}

// ── Hulpfunctie: print naar Serial én telnet client ───────────
static void tPrint(const char* s) {
    Serial.print(s);
    if (telnetClient && telnetClient.connected()) telnetClient.print(s);
}

// ── Core 0: seriële uitvoer + telnet (1 Hz) ──────────────────
void slowTask(void*) {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Nieuwe telnet-verbinding accepteren
        if (!telnetClient || !telnetClient.connected()) {
            WiFiClient nieuw = telnetServer.accept();
            if (nieuw) {
                telnetClient = nieuw;
                telnetClient.println("=== mks-balancing telnet ===");
            }
        }

        if (otaActive) continue;

        SensorData snap;
        portENTER_CRITICAL(&dataMux);
        snap = sharedSensors;
        portEXIT_CRITICAL(&dataMux);

        char buf[200];
        int n = 0;
#if USE_ENC1
        n += snprintf(buf+n, sizeof(buf)-n, "enc1=%.2f° %.1f rad/s  sp=%.1f drv=%.3fV  ",
            snap.enc1_deg, snap.enc1_rads, snap.speedSetpoint, snap.driveOut);
#endif
#if USE_ENC2
        n += snprintf(buf+n, sizeof(buf)-n, "enc2=%.2f° (%.1f rad/s)  ", snap.enc2_deg, snap.enc2_rads);
#endif
#if USE_IMU
        n += snprintf(buf+n, sizeof(buf)-n, "accel=(%.2f,%.2f,%.2f)  gyro=(%.2f,%.2f,%.2f)",
            snap.accel_x, snap.accel_y, snap.accel_z,
            snap.gyro_x,  snap.gyro_y,  snap.gyro_z);
#endif
        tPrint(buf);
        tPrint("\r\n");
    }
}

// ── Core 0: OTA afhandeling (50 ms) ──────────────────────────
void otaTask(void*) {
    while (true) {
        ArduinoOTA.handle();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    Serial.println("=== balancing_robot_basic_motor_test ===");

    // ── WiFi + OTA ───────────────────────────────────────────
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("WiFi verbinden");
    int tries = WIFI_TIMEOUT_S * 2;
    while (WiFi.status() != WL_CONNECTED && tries-- > 0) {
        delay(500);
        Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\nWiFi verbonden: %s\n", WiFi.localIP().toString().c_str());

        ArduinoOTA.setHostname(OTA_HOSTNAME);
        ArduinoOTA.onStart([]() {
            otaActive = true;
#if USE_MOTOR1
            motor1.setPhaseVoltage(0.0f, 0.0f);
#endif
#if USE_MOTOR2
            motor2.setPhaseVoltage(0.0f, 0.0f);
#endif
            Serial.println("\nOTA start – motoren gestopt.");
        });
        ArduinoOTA.onEnd([]() {
            Serial.println("\nOTA klaar, herstarten...");
        });
        ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("OTA: %u%%\r", progress * 100 / total);
        });
        ArduinoOTA.onError([](ota_error_t error) {
            Serial.printf("OTA fout [%u]\n", error);
            otaActive = false;
        });
        ArduinoOTA.begin();
        telnetServer.begin();
        telnetServer.setNoDelay(true);
        xTaskCreatePinnedToCore(otaTask, "ota", 8192, nullptr, 1, nullptr, 0);
        Serial.printf("OTA gereed. Upload via Arduino IDE → '%s'\n", OTA_HOSTNAME);
        Serial.printf("Telnet: verbind met %s op poort 23\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\nWiFi niet bereikbaar – OTA uitgeschakeld, normaal verder.");
    }

    // ── SPI + sensoren ───────────────────────────────────────
    sensorSPI.begin(SENS_SCK, SENS_MISO, SENS_MOSI, -1);

#if USE_ENC1
    pinMode(CS_ENC1,    OUTPUT); digitalWrite(CS_ENC1,    HIGH);
#endif
#if USE_ENC2
    pinMode(CS_ENC2,    OUTPUT); digitalWrite(CS_ENC2,    HIGH);
#endif
#if USE_IMU
    pinMode(CS_BMI_ACC, OUTPUT); digitalWrite(CS_BMI_ACC, HIGH);
    pinMode(CS_BMI_GYR, OUTPUT); digitalWrite(CS_BMI_GYR, HIGH);
    bmi088_init(CS_BMI_ACC, CS_BMI_GYR);
    Serial.println("BMI088 geïnitialiseerd.");
#endif

    // ── Motoren ──────────────────────────────────────────────
#if USE_MOTOR1
    motor1.begin();
    #if USE_ENC1
        motor1.setPhaseVoltage(0.0f, ALIGN_VOLTAGE_V);
        delay(3000);
        float m1deg = mt6835_readAngle(CS_ENC1);
        float m1off = -(m1deg * DEG_TO_RAD) * POLE_PAIRS;
        motor1.setOffset(m1off);
        Serial.printf("Motor1 uitlijning: %.2f°  offset=%.4f rad\n", m1deg, m1off);
    #else
        Serial.println("LET OP: USE_MOTOR1=true maar USE_ENC1=false – geen uitlijning mogelijk.");
    #endif
#endif

#if USE_MOTOR2
    motor2.begin();
    #if USE_ENC2
        motor2.setPhaseVoltage(0.0f, ALIGN_VOLTAGE_V);
        delay(3000);
        float m2deg = mt6835_readAngle(CS_ENC2);
        float m2off = -(m2deg * DEG_TO_RAD) * POLE_PAIRS;
        motor2.setOffset(m2off);
        Serial.printf("Motor2 uitlijning: %.2f°  offset=%.4f rad\n", m2deg, m2off);
    #else
        Serial.println("LET OP: USE_MOTOR2=true maar USE_ENC2=false – geen uitlijning mogelijk.");
    #endif
#endif

    // ── Taken starten ────────────────────────────────────────
    xTaskCreatePinnedToCore(motorTask, "motor", 4096, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(slowTask,  "slow",  4096, nullptr, 1, nullptr, 0);

    Serial.println("Taken gestart.");
}

// ── Loop (Arduino main-taak, Core 1) ─────────────────────────
// Staat stil; alle werk gebeurt in motorTask, slowTask en otaTask.
void loop() {
    vTaskDelay(portMAX_DELAY);
}
