// ============================================================
//  balancing_robot_basic_motor_test.ino  –  MKS-ESP32FOC V2.0
//
//  Zet per sensor/motor op true als die is aangesloten:
#define USE_ENC1    true    // MT6835 encoder motor 1
#define USE_ENC2    false   // MT6835 encoder motor 2
#define USE_IMU     false   // BMI088 accel + gyro
#define USE_MOTOR1  false   // motor 1 aansturen
#define USE_MOTOR2  false   // motor 2 aansturen
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
// ============================================================

#include "Motor.h"
#include <SPI.h>

// ── SPI sensor-bus ────────────────────────────────────────────
#define SENS_SCK   18
#define SENS_MISO  19
#define SENS_MOSI  22

// ── CS-lijnen ─────────────────────────────────────────────────
// TODO: vervangen door 74HC42 decoder (A/B/C → 1-van-8) zodra chip beschikbaar
#define CS_ENC1     23   // J6 SDA
#define CS_ENC2      5   // J6 SCL
#define CS_BMI_ACC  21   // J6 I_1
#define CS_BMI_GYR   4   // vrije pin – definitief toewijzen na 74HC42

#define SENSOR_HZ   1000   // gelijk aan motor-regelloop (1 kHz)
#define ALIGN_VOLTAGE_V  0.3f
#define DRIVE_VOLTAGE_V  0.6f
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
};

// ── MT6835 driver ─────────────────────────────────────────────
// Protocol: 24-bit frame  [cmd(4b)|addr[11:8](4b)] [addr[7:0]] [data]
// READ cmd = 0b0011 (0x3x), SPI_MODE3, max 16 MHz

static uint8_t mt6835_readReg(uint8_t cs, uint16_t addr) {
    uint8_t tx[3] = { (uint8_t)(0x30 | (addr >> 8)), (uint8_t)(addr & 0xFF), 0x00 };
    uint8_t rx[3] = {};
    sensorSPI.beginTransaction(SPISettings(16'000'000, MSBFIRST, SPI_MODE3));
    digitalWrite(cs, LOW);
    sensorSPI.transferBytes(tx, rx, 3);
    digitalWrite(cs, HIGH);
    sensorSPI.endTransaction();
    return rx[2];
}

static float mt6835_readAngle(uint8_t cs, uint8_t* statusOut = nullptr) {
    // Burst read: adres auto-incrementeert zolang CS laag blijft.
    // TX: [0x30][0x03][dummy x4]
    // RX: [----][----][0x003][0x004][0x005][0x006]
    uint8_t tx[6] = { 0x30, 0x03, 0x00, 0x00, 0x00, 0x00 };
    uint8_t rx[6] = {};
    sensorSPI.beginTransaction(SPISettings(16'000'000, MSBFIRST, SPI_MODE3));
    digitalWrite(cs, LOW);
    sensorSPI.transferBytes(tx, rx, 6);
    digitalWrite(cs, HIGH);
    sensorSPI.endTransaction();
    if (statusOut) *statusOut = rx[4] & 0x03;
    uint32_t raw = ((uint32_t)rx[2] << 13) | ((uint32_t)rx[3] << 5) | ((rx[4] >> 3) & 0x1F);
    return raw * 360.0f / (float)(1UL << 21);
}

// ── BMI088 driver ─────────────────────────────────────────────

static void bmi088_writeReg(uint8_t cs, uint8_t addr, uint8_t val) {
    sensorSPI.beginTransaction(SPISettings(10'000'000, MSBFIRST, SPI_MODE0));
    digitalWrite(cs, LOW);
    sensorSPI.transfer(addr & 0x7F);
    sensorSPI.transfer(val);
    digitalWrite(cs, HIGH);
    sensorSPI.endTransaction();
}

static void bmi088_readAccel(float& ax, float& ay, float& az) {
    uint8_t tx[8] = { 0x92, 0, 0, 0, 0, 0, 0, 0 };  // 0x80|0x12 + dummy + 6 bytes
    uint8_t rx[8] = {};
    sensorSPI.beginTransaction(SPISettings(10'000'000, MSBFIRST, SPI_MODE0));
    digitalWrite(CS_BMI_ACC, LOW);
    sensorSPI.transferBytes(tx, rx, 8);
    digitalWrite(CS_BMI_ACC, HIGH);
    sensorSPI.endTransaction();
    int16_t rx_raw = (int16_t)((rx[3] << 8) | rx[2]);
    int16_t ry_raw = (int16_t)((rx[5] << 8) | rx[4]);
    int16_t rz_raw = (int16_t)((rx[7] << 8) | rx[6]);
    const float scale = 9.80665f / 10920.0f;  // ±3g default
    ax = rx_raw * scale;
    ay = ry_raw * scale;
    az = rz_raw * scale;
}

static void bmi088_readGyro(float& gx, float& gy, float& gz) {
    uint8_t tx[7] = { 0x82, 0, 0, 0, 0, 0, 0 };  // 0x80|0x02, geen dummy, 6 bytes
    uint8_t rx[7] = {};
    sensorSPI.beginTransaction(SPISettings(10'000'000, MSBFIRST, SPI_MODE0));
    digitalWrite(CS_BMI_GYR, LOW);
    sensorSPI.transferBytes(tx, rx, 7);
    digitalWrite(CS_BMI_GYR, HIGH);
    sensorSPI.endTransaction();
    int16_t rx_raw = (int16_t)((rx[2] << 8) | rx[1]);
    int16_t ry_raw = (int16_t)((rx[4] << 8) | rx[3]);
    int16_t rz_raw = (int16_t)((rx[6] << 8) | rx[5]);
    const float scale = 1.0f / 16.384f;  // ±2000°/s default
    gx = rx_raw * scale;
    gy = ry_raw * scale;
    gz = rz_raw * scale;
}

static void bmi088_init() {
    bmi088_writeReg(CS_BMI_ACC, 0x7C, 0x00);  // PWR_CONF: active
    delay(5);
    bmi088_writeReg(CS_BMI_ACC, 0x7D, 0x04);  // PWR_CTRL: accel on
    delay(5);
    bmi088_writeReg(CS_BMI_ACC, 0x40, 0xA8);  // ACC_CONF: ODR=1600Hz, BW=normal
    bmi088_writeReg(CS_BMI_ACC, 0x41, 0x00);  // ACC_RANGE: ±3g
}

// ── Snelheidsberekening ───────────────────────────────────────
static float prevEnc1 = -1.0f;
static float prevEnc2 = -1.0f;

static float angleDelta(float cur, float prev) {
    float d = cur - prev;
    if (d >  180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

// ── Sensors uitlezen ──────────────────────────────────────────
// Alleen actieve sensoren worden gelezen; geen simulatiedata.
// TODO: bij zichtbare ruis in de regelaar een moving-average toevoegen over bijv.
//       8–16 samples (ringbuffer per encoder, gemiddelde van de deltas vóór omzetting).
static SensorData sensors = {};

void readSensors() {
#if USE_ENC1
    sensors.enc1_deg   = mt6835_readAngle(CS_ENC1);
    sensors.enc1_rads  = (prevEnc1 < 0.0f) ? 0.0f
                         : angleDelta(sensors.enc1_deg, prevEnc1) * (float(M_PI) / 180.0f) * SENSOR_HZ;
    prevEnc1 = sensors.enc1_deg;
#endif
#if USE_ENC2
    sensors.enc2_deg   = mt6835_readAngle(CS_ENC2);
    sensors.enc2_rads  = (prevEnc2 < 0.0f) ? 0.0f
                         : angleDelta(sensors.enc2_deg, prevEnc2) * (float(M_PI) / 180.0f) * SENSOR_HZ;
    prevEnc2 = sensors.enc2_deg;
#endif
#if USE_IMU
    bmi088_readAccel(sensors.accel_x, sensors.accel_y, sensors.accel_z);
    bmi088_readGyro (sensors.gyro_x,  sensors.gyro_y,  sensors.gyro_z);
#endif
}

// ── Hardware-timer ────────────────────────────────────────────
hw_timer_t*      motorTimer = NULL;
portMUX_TYPE     timerMux   = portMUX_INITIALIZER_UNLOCKED;
volatile bool    motorFlag  = false;

void IRAM_ATTR onMotorTimer() {
    motorFlag = true;
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    Serial.println("=== balancing_robot_basic_motor_test ===");

    // SPI bus
    sensorSPI.begin(SENS_SCK, SENS_MISO, SENS_MOSI, -1);

    // CS-pinnen hoog (niet geselecteerd)
#if USE_ENC1
    pinMode(CS_ENC1,    OUTPUT); digitalWrite(CS_ENC1,    HIGH);
#endif
#if USE_ENC2
    pinMode(CS_ENC2,    OUTPUT); digitalWrite(CS_ENC2,    HIGH);
#endif
#if USE_IMU
    pinMode(CS_BMI_ACC, OUTPUT); digitalWrite(CS_BMI_ACC, HIGH);
    pinMode(CS_BMI_GYR, OUTPUT); digitalWrite(CS_BMI_GYR, HIGH);
    bmi088_init();
    Serial.println("BMI088 geïnitialiseerd.");
#endif

    // Motor 1 init + uitlijning
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

    // Motor 2 init + uitlijning
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

    // Timer 1 kHz
    motorTimer = timerBegin(1000000);
    timerAttachInterrupt(motorTimer, &onMotorTimer);
    timerAlarm(motorTimer, 1000, true, 0);

    Serial.println("Gereed.");
}

// ── Loop ──────────────────────────────────────────────────────
static uint32_t lastPrint = 0;

void loop() {
    if (!motorFlag) return;
    motorFlag = false;

    readSensors();

#if USE_MOTOR1 && USE_ENC1
    motor1.loop(DRIVE_VOLTAGE_V, sensors.enc1_deg * DEG_TO_RAD);
#endif
#if USE_MOTOR2 && USE_ENC2
    motor2.loop(DRIVE_VOLTAGE_V, sensors.enc2_deg * DEG_TO_RAD);
#endif

    // Diagnostiek 1× per seconde
    if (millis() - lastPrint >= 1000) {
        lastPrint = millis();
#if USE_ENC1
        Serial.printf("enc1=%.2f° (%.1f rad/s)  ", sensors.enc1_deg, sensors.enc1_rads);
#endif
#if USE_ENC2
        Serial.printf("enc2=%.2f° (%.1f rad/s)  ", sensors.enc2_deg, sensors.enc2_rads);
#endif
#if USE_IMU
        Serial.printf("accel=(%.2f,%.2f,%.2f)  gyro=(%.2f,%.2f,%.2f)",
            sensors.accel_x, sensors.accel_y, sensors.accel_z,
            sensors.gyro_x,  sensors.gyro_y,  sensors.gyro_z);
#endif
        Serial.println();
    }
}
