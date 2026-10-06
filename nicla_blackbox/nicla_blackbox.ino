/*
 * Blackbox Challenge — Nicla Sense ME Firmware
 * FH Kufstein Tirol · Rapid Prototyping SPS.BBM.24
 * Drop deadline: 24 June 2026
 *
 * Board: Arduino Nicla Sense ME (ABX00050)
 *        nRF52832 SoC + BHI260AP IMU
 *
 * CRITICAL INIT ORDER: BHY2.begin() MUST come before BLE.begin()
 *   BLE-first silently kills BHI260AP sensor events (always reads 0,0,0)
 *
 * SCALE: BHI260AP returns Q12 fixed-point acceleration.
 *        1 g = 4096 LSB  (NOT 1000 — that caused false IMPACT at rest)
 *        Gyro raw LSB: divide by ~16 for approximate deg/s
 *        Quaternion: float ×10000 packed as int16 → divide by 10000 in Python
 *
 * BLE packet formats:
 *   Download (visualizer.py)    : 6 bytes   {ax,ay,az}           int16×3
 *   Live-stream (live_stream.py): 20 bytes  {ax,ay,az,gx,gy,gz,qx,qy,qz,qw} int16×10
 *     20 bytes = exactly the BLE default ATT MTU payload limit (no MTU negotiation needed)
 */

#include "Nicla_System.h"
#include "Arduino_BHY2.h"
#include <ArduinoBLE.h>

// ── BLE UUIDs (must match Python scripts) ────────────────────────────────────
#define SVC_UUID    "19b10000-e8f2-537e-4f6c-d104768a1214"
#define STATUS_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#define META_UUID   "19b10002-e8f2-537e-4f6c-d104768a1214"
#define DATA_UUID   "19b10003-e8f2-537e-4f6c-d104768a1214"
#define CTRL_UUID   "19b10004-e8f2-537e-4f6c-d104768a1214"

// ── Status byte values ────────────────────────────────────────────────────────
#define STATUS_IDLE      0x00
#define STATUS_FREEFALL  0x01
#define STATUS_RECORDING 0x02
#define STATUS_READY     0x03
#define STATUS_SENDING   0x04
#define STATUS_STREAMING 0x05

// ── Control commands from Python ──────────────────────────────────────────────
#define CMD_REQUEST 0x01   // download stored drop data
#define CMD_STREAM  0x02   // start live stream
#define CMD_RESET   0xFF   // reset to IDLE

// ── Sensor configuration ──────────────────────────────────────────────────────
#define SAMPLE_RATE_HZ       200
#define PRE_IMPACT_SAMPLES   100
#define POST_IMPACT_SAMPLES  400
#define TOTAL_SAMPLES        (PRE_IMPACT_SAMPLES + POST_IMPACT_SAMPLES)

// BHI260AP Q12 scale: 1g = 4096 LSB
#define ACCEL_SCALE  4096.0f

// Quaternion packing: float [-1,1] → int16 by ×10000
// Max |q| component ≤ 1.0 → max int16 = 10000, well within ±32767
#define QUAT_SCALE   10000.0f

// Phase detection thresholds (in TRUE g units after ACCEL_SCALE)
#define FREEFALL_THRESH_G  0.25f
#define IMPACT_THRESH_G    4.0f
#define FREEFALL_MIN_MS    80

// Sensor warm-up gate
#define STABLE_NEEDED  30

#define SAMPLE_INTERVAL_MS  (1000 / SAMPLE_RATE_HZ)  // 5 ms

// ── Download packet — 6 bytes (visualizer.py) ────────────────────────────────
struct __attribute__((packed)) Sample {
    int16_t ax, ay, az;
};
static_assert(sizeof(Sample) == 6, "Sample must be 6 bytes");

// ── Live-stream packet — 20 bytes (live_stream.py) ───────────────────────────
// Exactly fills BLE default ATT MTU payload (20 bytes = 23 MTU − 3 header)
struct __attribute__((packed)) StreamPkt {
    int16_t ax, ay, az;      // accel  (÷4096 → g)
    int16_t gx, gy, gz;      // gyro   (÷16 → approx deg/s)
    int16_t qx, qy, qz, qw; // quaternion (÷10000 → unit quaternion)
};
static_assert(sizeof(StreamPkt) == 20, "StreamPkt must be 20 bytes");

// ── Ring + linear buffers ─────────────────────────────────────────────────────
static Sample   sampleBuf[TOTAL_SAMPLES];
static int      ringHead   = 0;
static int      ringCount  = 0;
static int      recCount   = 0;

// ── State machine ─────────────────────────────────────────────────────────────
// LIVE_STREAM is now a parallel FLAG, not an exclusive state.
// Recording (IDLE→FREEFALL_DETECT→RECORDING→DATA_READY) always runs independently.
// Streaming is simply layered on top — if BLE disconnects mid-drop, recording continues.
enum State { IDLE, FREEFALL_DETECT, RECORDING, DATA_READY };
static State devState  = IDLE;
static bool  streamActive = false;   // true while live_stream.py is connected

// ── Sensor arm gate ───────────────────────────────────────────────────────────
static bool    sensorArmed = false;
static uint8_t stableCount = 0;

// ── Timing ────────────────────────────────────────────────────────────────────
static uint32_t lastSampleMs    = 0;
static uint32_t freefallStartMs = 0;
static uint32_t lastDebugMs     = 0;
static uint32_t lastLedMs       = 0;
static bool     ledPulseState   = false;

// ── BLE objects ───────────────────────────────────────────────────────────────
BLEService            bbService (SVC_UUID);
BLEByteCharacteristic statusChar(STATUS_UUID, BLERead | BLENotify);
BLECharacteristic     metaChar  (META_UUID,   BLERead, 12);
BLECharacteristic     dataChar  (DATA_UUID,   BLERead | BLENotify, sizeof(StreamPkt));  // 20 bytes
BLECharacteristic     ctrlChar  (CTRL_UUID,   BLEWrite, 1);

// ── Sensors ───────────────────────────────────────────────────────────────────
SensorXYZ       accel    (SENSOR_ID_ACC);
SensorXYZ       gyro     (SENSOR_ID_GYRO);
SensorQuaternion quaternion(SENSOR_ID_RV);   // Rotation Vector — gives orientation

// ── Helpers ───────────────────────────────────────────────────────────────────
#define SERIALP(fmt, ...) { char _b[128]; snprintf(_b,sizeof(_b),fmt,##__VA_ARGS__); Serial.print(_b); }

inline void ledGreen() { nicla::leds.setColor(green); }
inline void ledBlue()  { nicla::leds.setColor(blue);  }
inline void ledRed()   { nicla::leds.setColor(red);   }
inline void ledWhite() { nicla::leds.setColor(white); }
inline void ledOff()   { nicla::leds.setColor(off);   }

inline float resultantG(int16_t x, int16_t y, int16_t z) {
    float fx = x / ACCEL_SCALE;
    float fy = y / ACCEL_SCALE;
    float fz = z / ACCEL_SCALE;
    return sqrtf(fx*fx + fy*fy + fz*fz);
}

// Clamp float to int16 after scaling
inline int16_t floatToQ(float v, float scale) {
    float s = v * scale;
    if (s >  32767.0f) s =  32767.0f;
    if (s < -32767.0f) s = -32767.0f;
    return (int16_t)s;
}

void writeMeta(uint32_t total, uint32_t rate, uint32_t impact_ms) {
    uint8_t buf[12];
    memcpy(buf + 0, &total,     4);
    memcpy(buf + 4, &rate,      4);
    memcpy(buf + 8, &impact_ms, 4);
    metaChar.writeValue(buf, 12);
}

void resetDevice() {
    devState      = IDLE;
    streamActive  = false;
    ringHead      = 0;
    ringCount     = 0;
    recCount      = 0;
    sensorArmed   = false;
    stableCount   = 0;
    statusChar.writeValue(STATUS_IDLE);
    ledGreen();
    Serial.println("[BB] Reset → IDLE");
}

// ── setup() ───────────────────────────────────────────────────────────────────
void setup() {
    nicla::begin();
    nicla::leds.begin();
    Serial.begin(115200);
    delay(400);
    Serial.println("[BB] Blackbox Challenge — Nicla Sense ME");
    Serial.println("[BB] Scale: 1g = 4096 LSB | Quat ×10000 | StreamPkt = 20 bytes");

    // BHY2 MUST initialise before BLE
    BHY2.begin(NICLA_STANDALONE);
    accel.begin();
    gyro.begin();
    quaternion.begin();
    Serial.println("[BB] Sensors OK: accel + gyro + quaternion (RV)");

    if (!BLE.begin()) {
        Serial.println("[BB] BLE init failed — halting");
        ledRed();
        while (1) delay(500);
    }

    BLE.setLocalName("Blackbox-1");
    BLE.setAdvertisedService(bbService);
    bbService.addCharacteristic(statusChar);
    bbService.addCharacteristic(metaChar);
    bbService.addCharacteristic(dataChar);
    bbService.addCharacteristic(ctrlChar);
    BLE.addService(bbService);

    statusChar.writeValue(STATUS_IDLE);
    BLE.advertise();
    Serial.println("[BB] BLE advertising as 'Blackbox-1'");

    for (int i = 0; i < 3; i++) { ledGreen(); delay(200); ledOff(); delay(200); }
    ledGreen();

    lastSampleMs = millis();
    SERIALP("[BB] Ready — %d Hz, %d samples (%d pre + %d post)\n",
            SAMPLE_RATE_HZ, TOTAL_SAMPLES, PRE_IMPACT_SAMPLES, POST_IMPACT_SAMPLES);
}

// ── loop() ────────────────────────────────────────────────────────────────────
void loop() {
    BHY2.update();
    BLE.poll();

    uint32_t now = millis();

    // ── Handle ctrl command ───────────────────────────────────────────────────
    if (ctrlChar.written()) {
        uint8_t cmd = ctrlChar.value()[0];

        if (cmd == CMD_RESET) {
            resetDevice();
        }
        else if (cmd == CMD_STREAM) {
            streamActive = true;
            statusChar.writeValue(STATUS_STREAMING);
            // Do NOT change devState — recording pipeline runs independently.
            // If device was IDLE when stream started, it will still detect the drop.
            // If device was DATA_READY, it will still serve the stored data.
            Serial.println("[BB] CMD_STREAM → streaming ON (recording pipeline unaffected)");
        }
        else if (cmd == CMD_REQUEST && devState == DATA_READY) {
            statusChar.writeValue(STATUS_SENDING);
            ledWhite();
            SERIALP("[BB] Sending %d pre + %d post samples\n", ringCount, recCount);

            for (int i = 0; i < ringCount; i++) {
                int idx = (ringHead + i) % PRE_IMPACT_SAMPLES;
                if (BLE.connected()) {
                    dataChar.writeValue((uint8_t*)&sampleBuf[idx], sizeof(Sample));
                    BLE.poll();
                    delay(2);
                }
            }
            for (int i = 0; i < recCount; i++) {
                int idx = PRE_IMPACT_SAMPLES + i;
                if (BLE.connected()) {
                    dataChar.writeValue((uint8_t*)&sampleBuf[idx], sizeof(Sample));
                    BLE.poll();
                    delay(2);
                }
            }

            statusChar.writeValue(STATUS_READY);
            devState = DATA_READY;
            Serial.println("[BB] TX complete");
        }
    }

    // ── Rate-limit sampling ───────────────────────────────────────────────────
    if (now - lastSampleMs < SAMPLE_INTERVAL_MS) return;
    lastSampleMs = now;

    int16_t rawX  = accel.x();
    int16_t rawY  = accel.y();
    int16_t rawZ  = accel.z();
    int16_t rawGX = gyro.x();
    int16_t rawGY = gyro.y();
    int16_t rawGZ = gyro.z();

    // Read quaternion (float, range ≈ -1 to +1 per component)
    float fQX = quaternion.x();
    float fQY = quaternion.y();
    float fQZ = quaternion.z();
    float fQW = quaternion.w();

    float R = resultantG(rawX, rawY, rawZ);

    switch (devState) {

        // ─── IDLE ─────────────────────────────────────────────────────────────
        case IDLE: {
            if (!sensorArmed) {
                if (now - lastDebugMs >= 1000) {
                    lastDebugMs = now;
                    SERIALP("[BB] warmup x=%d y=%d z=%d  R=%.4f g\n",
                            rawX, rawY, rawZ, R);
                }
                bool validG = (rawX != 0 || rawY != 0 || rawZ != 0)
                           && (R > 0.70f && R < 1.30f);
                if (validG) {
                    if (++stableCount >= STABLE_NEEDED) {
                        sensorArmed = true;
                        ledGreen();
                        Serial.println("[BB] Sensor stable — detection ARMED");
                    }
                } else {
                    stableCount = 0;
                }
                break;
            }

            sampleBuf[ringHead] = { rawX, rawY, rawZ };
            ringHead = (ringHead + 1) % PRE_IMPACT_SAMPLES;
            if (ringCount < PRE_IMPACT_SAMPLES) ringCount++;

            if (R < FREEFALL_THRESH_G) {
                freefallStartMs = now;
                devState = FREEFALL_DETECT;
                statusChar.writeValue(STATUS_FREEFALL);
                ledBlue();
                SERIALP("[BB] Free-fall start  R=%.4f @ %lu ms\n", R, now);
            }
            break;
        }

        // ─── FREEFALL_DETECT ──────────────────────────────────────────────────
        case FREEFALL_DETECT: {
            sampleBuf[ringHead] = { rawX, rawY, rawZ };
            ringHead = (ringHead + 1) % PRE_IMPACT_SAMPLES;
            if (ringCount < PRE_IMPACT_SAMPLES) ringCount++;

            if (R >= FREEFALL_THRESH_G) {
                devState = IDLE;
                statusChar.writeValue(STATUS_IDLE);
                ledGreen();
                SERIALP("[BB] FF cancelled  R=%.4f\n", R);
            } else if (now - freefallStartMs >= FREEFALL_MIN_MS) {
                devState = RECORDING;
                recCount = 0;
                statusChar.writeValue(STATUS_RECORDING);
                ledRed();
                SERIALP("[BB] FF CONFIRMED → RECORDING  (pre=%d)\n", ringCount);
            }
            break;
        }

        // ─── RECORDING ────────────────────────────────────────────────────────
        case RECORDING: {
            int recIdx = PRE_IMPACT_SAMPLES + recCount;
            if (recIdx < TOTAL_SAMPLES) {
                sampleBuf[recIdx] = { rawX, rawY, rawZ };
            }
            recCount++;

            if (recCount >= POST_IMPACT_SAMPLES) {
                uint32_t total  = (uint32_t)(ringCount + recCount);
                uint32_t rate   = (uint32_t)SAMPLE_RATE_HZ;
                uint32_t imp_ms = (uint32_t)((float)PRE_IMPACT_SAMPLES * SAMPLE_INTERVAL_MS);
                writeMeta(total, rate, imp_ms);

                devState = DATA_READY;
                statusChar.writeValue(STATUS_READY);
                ledWhite();
                SERIALP("[BB] Recording done — total=%lu samples ready\n", total);
            }
            break;
        }

        // ─── DATA_READY ───────────────────────────────────────────────────────
        case DATA_READY: {
            if (now - lastLedMs >= 500) {
                lastLedMs = now;
                ledPulseState = !ledPulseState;
                ledPulseState ? ledWhite() : ledOff();
            }
            break;
        }

    }

    // ── Parallel live stream — runs every tick, independent of devState ────────
    // BLE disconnect stops the stream but never affects recording or DATA_READY.
    if (streamActive) {
        if (!BLE.connected()) {
            streamActive = false;
            BLE.advertise();   // re-advertise so visualizer.py can reconnect later
            Serial.println("[BB] BLE gone → stream OFF, data safe in RAM, re-advertising");
        } else {
            StreamPkt pkt = {
                rawX, rawY, rawZ,
                rawGX, rawGY, rawGZ,
                floatToQ(fQX, QUAT_SCALE),
                floatToQ(fQY, QUAT_SCALE),
                floatToQ(fQZ, QUAT_SCALE),
                floatToQ(fQW, QUAT_SCALE)
            };
            dataChar.writeValue((uint8_t*)&pkt, sizeof(StreamPkt));

            if (now - lastDebugMs >= 1000) {
                lastDebugMs = now;
                SERIALP("[BB] STREAM  state=%d  R=%.4fg  qw=%.3f\n",
                        (int)devState, R, fQW);
            }
        }
    }
}
