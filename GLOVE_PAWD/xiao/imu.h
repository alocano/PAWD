//prob logic and header file

#ifndef PROB_LOGIC_H
#define PROB_LOGIC_H

#include <Wire.h>
#include <Adafruit_LSM6DS3TRC.h>

// I2C Pins
static const int SDA_PIN = 8;
static const int SCL_PIN = 9;

// IMU Object
Adafruit_LSM6DS3TRC imu;
static bool imuReady = false;

// Tuning Constants
static const float alpha    = 0.01f;
static const float restThr  = 7.0f;
static const float deadband = 25.0f;
static const float peakThr  = 50.0f;
static const float minGap   = 0.25f;

// PROB Session Settings
static const uint32_t PROB_TARGET_CYCLES = 10;
static const uint32_t PROB_SAMPLE_MS     = 25;
static const float    PROB_SAMPLE_SEC    = PROB_SAMPLE_MS / 1000.0f;

// Per-Rotation Sample Buffer
// One sample taken every ROT_SAMPLE_INTERVAL ticks (5 * 25ms = 125ms).
// Stores up to ROT_MAX_SAMPLES raw gyro X values per rotation.
// These are sent as stateID 205 packets and logged to Serial on WROOM.
#define ROT_MAX_SAMPLES     8
#define ROT_SAMPLE_INTERVAL 5   // 1 sample every 5 ticks = every 125ms

static float   rotSamples[ROT_MAX_SAMPLES];
static uint8_t rotSampleCount = 0;
static uint8_t rotSampleTick  = 0;

// Cycle Counting State
static float         bias_dps            = 0.0f;
static bool          hadPeakPos          = false;
static bool          hadPeakNeg          = false;
static int           lastSign            = 0;
static float         lastCrossTime       = -1e9f;
static uint32_t      halfCycles          = 0;
static unsigned long t0_ms              = 0;
static bool          probRunning         = false;
static unsigned long probNextSampleMs    = 0;
static uint32_t      lastDisplayedCycles = 999;

// Per-Rotation Summary
// stateID 204 format: "R:N T:X.XXs A:XXXX.X"
//   R = rotation number  T = seconds  A = integrated angle (degrees)
// stateID 205 format: "S:N V:XXX.X"
//   S = sample index (1 to N)  V = gyro X dps at that moment
static unsigned long rotCycleStartMs = 0;
static float         rotAngleAccum   = 0.0f;
static char          rotDataBuf[32]  = "";

// Helpers
static inline float rad2deg(float r) { return r * 57.2957795f; }

int signWithDeadband(float x_dps) {
    if (x_dps >  deadband) return +1;
    if (x_dps < -deadband) return -1;
    return 0;
}

// IMU Init
static bool initIMUIfNeeded() {
    if (imuReady) return true;
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    Serial.println("Trying IMU on I2C...");
    if (imu.begin_I2C(0x6A)) { Serial.println("IMU found at 0x6A"); imuReady = true; return true; }
    if (imu.begin_I2C(0x6B)) { Serial.println("IMU found at 0x6B"); imuReady = true; return true; }
    Serial.println("ERROR: IMU not found");
    imuReady = false;
    return false;
}

// Reset — call once at the start of every new PROB session
void resetPROB() {
    bias_dps            = 0.0f;
    hadPeakPos          = false;
    hadPeakNeg          = false;
    lastSign            = 0;
    lastCrossTime       = -1e9f;
    halfCycles          = 0;
    t0_ms               = millis();
    lastDisplayedCycles = 999;
    probRunning         = true;
    probNextSampleMs    = millis();
    rotCycleStartMs     = millis();
    rotAngleAccum       = 0.0f;
    rotSampleCount      = 0;
    rotSampleTick       = 0;
    memset(rotSamples,  0, sizeof(rotSamples));
    memset(rotDataBuf,  0, sizeof(rotDataBuf));
}

// Reset sample buffer — call from MINI_MAIN after sending samples
void resetRotSamples() {
    memset(rotSamples, 0, sizeof(rotSamples));
    rotSampleCount = 0;
    rotSampleTick  = 0;
}

// Run One Sample Tick — call every loop() while in PROB state
//
// Returns:
//   0 = nothing new
//   1 = live count updated        → send COUNT    (stateID 203)
//   2 = all rotations done        → send DONE     (stateID 202)
//   3 = full rotation completed   → send SAMPLES  (stateID 205 x N)
//                                    send ROT_DATA (stateID 204)
//                                    send COUNT    (stateID 203)
int runPROBSample(uint32_t &fullCyclesOut, char* rotDataOut) {
    if (millis() < probNextSampleMs) return 0;
    probNextSampleMs += PROB_SAMPLE_MS;

    float tNow = (millis() - t0_ms) * 0.001f;

    sensors_event_t a, g, temp;
    imu.getEvent(&a, &g, &temp);
    float gx_dps = rad2deg(g.gyro.x);

    if (fabsf(gx_dps - bias_dps) < restThr)
        bias_dps = (1.0f - alpha) * bias_dps + alpha * gx_dps;
    float gxc_dps = gx_dps - bias_dps;

    // Integrate angle
    rotAngleAccum += gxc_dps * PROB_SAMPLE_SEC;

    // Collect evenly-spaced samples (every ROT_SAMPLE_INTERVAL ticks)
    rotSampleTick++;
    if (rotSampleTick >= ROT_SAMPLE_INTERVAL) {
        rotSampleTick = 0;
        if (rotSampleCount < ROT_MAX_SAMPLES) {
            rotSamples[rotSampleCount] = gxc_dps;
            rotSampleCount++;
        }
    }

    if (gxc_dps >=  peakThr) hadPeakPos = true;
    if (gxc_dps <= -peakThr) hadPeakNeg = true;

    int signNow = signWithDeadband(gxc_dps);
    bool newFullCycle = false;

    if (signNow != 0 && signNow != lastSign && (tNow - lastCrossTime) >= minGap) {
        bool valid = (signNow == +1 && hadPeakNeg) || (signNow == -1 && hadPeakPos);
        if (valid) {
            halfCycles++;
            lastCrossTime = tNow;

            if (halfCycles % 2 == 0) {
                uint32_t cycleNum = halfCycles / 2;
                float durSec      = (millis() - rotCycleStartMs) / 1000.0f;

                snprintf(rotDataBuf, sizeof(rotDataBuf),
                         "R:%lu T:%.2fs A:%.1f",
                         (unsigned long)cycleNum, durSec, rotAngleAccum);
                strncpy(rotDataOut, rotDataBuf, 32);

                // Reset rotation tracking (samples reset separately after send)
                rotCycleStartMs = millis();
                rotAngleAccum   = 0.0f;
                newFullCycle    = true;
            }

            if (signNow == +1) hadPeakNeg = false;
            else               hadPeakPos = false;
        }
        lastSign = signNow;
    }

    fullCyclesOut = halfCycles / 2;

    if (fullCyclesOut >= PROB_TARGET_CYCLES) { probRunning = false; return 2; }
    if (newFullCycle) { lastDisplayedCycles = fullCyclesOut; return 3; }
    if (fullCyclesOut != lastDisplayedCycles) { lastDisplayedCycles = fullCyclesOut; return 1; }

    return 0;
}

#endif