#include "imu.h"
#include <string.h>

// ─────────────────────────────────────────────
//  Module-private state
//  All variables are file-scoped (static).
//  Nothing is defined in the header, so there
//  are no multiple-definition linker errors.
// ─────────────────────────────────────────────

static Adafruit_LSM6DS3TRC s_imu;
static bool                s_imuReady = false;

// PROB algorithm state
static float         s_bias_dps         = 0.0f;
static bool          s_hadPeakPos       = false;
static bool          s_hadPeakNeg       = false;
static int           s_lastSign         = 0;
static float         s_lastCrossTime    = -1e9f;
static uint32_t      s_halfCycles       = 0;
static unsigned long s_t0_ms            = 0;
static bool          s_probRunning      = false;
static unsigned long s_probNextSampleMs = 0;
static uint32_t      s_lastDispCycles   = 0xFFFFFFFFu;

// Per-rotation accumulation
static float         s_rotAngleAccum    = 0.0f;
static unsigned long s_rotCycleStartMs  = 0;
static uint32_t      s_rotCycleNum      = 0;   // 1-based, incremented each full rotation

// Rotation sample buffer (reset after each rotation's samples are sent)
static float         s_rotSamples[ROT_MAX_SAMPLES];
static uint8_t       s_rotSampleCount   = 0;
static uint8_t       s_rotSampleTick    = 0;

// Snapshot of the last completed rotation — valid while
// imu_run_sample() return value is IMU_RESULT_ROT_COMPLETE
static float         s_snap_duration    = 0.0f;
static float         s_snap_angle       = 0.0f;
static uint32_t      s_snap_cycleNum    = 0;
static float         s_snap_samples[ROT_MAX_SAMPLES];
static uint8_t       s_snap_sampleCount = 0;

// ─────────────────────────────────────────────
//  Private helpers
// ─────────────────────────────────────────────

static inline float _rad2deg(float r) { return r * 57.2957795f; }

static int _signWithDeadband(float x_dps) {
    if (x_dps >  IMU_DEADBAND) return +1;
    if (x_dps < -IMU_DEADBAND) return -1;
    return 0;
}

static void _resetRotBuffer(void) {
    memset(s_rotSamples, 0, sizeof(s_rotSamples));
    s_rotSampleCount = 0;
    s_rotSampleTick  = 0;
    s_rotAngleAccum  = 0.0f;
    s_rotCycleStartMs = millis();
}

// ─────────────────────────────────────────────
//  imu_init
// ─────────────────────────────────────────────
bool imu_init(void) {
    if (s_imuReady) return true;

    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(400000);

    Serial.println("[IMU] Probing I2C...");
    if (s_imu.begin_I2C(0x6A)) {
        Serial.println("[IMU] Found at 0x6A");
        s_imuReady = true;
        return true;
    }
    if (s_imu.begin_I2C(0x6B)) {
        Serial.println("[IMU] Found at 0x6B");
        s_imuReady = true;
        return true;
    }

    Serial.println("[IMU] ERROR: not found on I2C bus");
    s_imuReady = false;
    return false;
}

// ─────────────────────────────────────────────
//  imu_reset
// ─────────────────────────────────────────────
void imu_reset(void) {
    s_bias_dps         = 0.0f;
    s_hadPeakPos       = false;
    s_hadPeakNeg       = false;
    s_lastSign         = 0;
    s_lastCrossTime    = -1e9f;
    s_halfCycles       = 0;
    s_lastDispCycles   = 0xFFFFFFFFu;
    s_t0_ms            = millis();
    s_probRunning      = true;
    s_probNextSampleMs = millis();
    s_rotCycleNum      = 0;
    _resetRotBuffer();

    memset(s_snap_samples, 0, sizeof(s_snap_samples));
    s_snap_sampleCount = 0;
    s_snap_duration    = 0.0f;
    s_snap_angle       = 0.0f;
    s_snap_cycleNum    = 0;
}

// ─────────────────────────────────────────────
//  imu_run_sample  (non-blocking tick)
// ─────────────────────────────────────────────
ImuResult_t imu_run_sample(void) {
    if (!s_probRunning) return IMU_RESULT_IDLE;
    if (millis() < s_probNextSampleMs) return IMU_RESULT_IDLE;

    s_probNextSampleMs += PROB_SAMPLE_MS;

    float tNow = (millis() - s_t0_ms) * 0.001f;

    // ── Read gyro ────────────────────────────────────────────────────
    sensors_event_t a, g, temp;
    s_imu.getEvent(&a, &g, &temp);
    float gx_dps = _rad2deg(g.gyro.x);

    // ── Adaptive bias filter ──────────────────────────────────────────
    if (fabsf(gx_dps - s_bias_dps) < IMU_REST_THR)
        s_bias_dps = (1.0f - IMU_ALPHA) * s_bias_dps + IMU_ALPHA * gx_dps;
    float gxc_dps = gx_dps - s_bias_dps;

    // ── Integrate angle for this rotation ────────────────────────────
    s_rotAngleAccum += gxc_dps * PROB_SAMPLE_SEC;

    // ── Collect evenly-spaced rotation samples ───────────────────────
    s_rotSampleTick++;
    if (s_rotSampleTick >= ROT_SAMPLE_INTERVAL) {
        s_rotSampleTick = 0;
        if (s_rotSampleCount < ROT_MAX_SAMPLES) {
            s_rotSamples[s_rotSampleCount++] = gxc_dps;
        }
    }

    // ── Peak detection ───────────────────────────────────────────────
    if (gxc_dps >=  IMU_PEAK_THR) s_hadPeakPos = true;
    if (gxc_dps <= -IMU_PEAK_THR) s_hadPeakNeg = true;

    // ── Zero-crossing / half-cycle detection ─────────────────────────
    int  signNow      = _signWithDeadband(gxc_dps);
    bool newFullCycle = false;

    if (signNow != 0 && signNow != s_lastSign &&
        (tNow - s_lastCrossTime) >= IMU_MIN_GAP_S) {

        bool valid = (signNow == +1 && s_hadPeakNeg) ||
                     (signNow == -1 && s_hadPeakPos);

        if (valid) {
            s_halfCycles++;
            s_lastCrossTime = tNow;

            if (s_halfCycles % 2 == 0) {
                // ── Full rotation complete ────────────────────────────
                s_rotCycleNum++;

                // Snapshot the rotation data before resetting accumulators.
                // The caller reads the snapshot via imu_fill_packet().
                s_snap_cycleNum    = s_rotCycleNum;
                s_snap_duration    = (millis() - s_rotCycleStartMs) / 1000.0f;
                s_snap_angle       = s_rotAngleAccum;
                s_snap_sampleCount = s_rotSampleCount;
                memcpy(s_snap_samples, s_rotSamples,
                       s_rotSampleCount * sizeof(float));

                Serial.printf("[IMU] Rot #%lu  T:%.2fs  A:%.1f°  samples:%d\n",
                              (unsigned long)s_snap_cycleNum,
                              s_snap_duration, s_snap_angle,
                              s_snap_sampleCount);

                // Reset per-rotation accumulators for the next cycle.
                // Sample buffer reset happens here so the caller doesn't
                // need to manage it — the snapshot is already safe.
                _resetRotBuffer();

                newFullCycle = true;
            }

            if (signNow == +1) s_hadPeakNeg = false;
            else               s_hadPeakPos = false;
        }
        s_lastSign = signNow;
    }

    uint32_t fullCycles = s_halfCycles / 2;

    // ── Determine return code (priority: complete > done > count) ─────

    if (newFullCycle) {
        // Don't stop yet — caller will check imu_full_cycles() vs target
        s_lastDispCycles = fullCycles;
        return IMU_RESULT_ROT_COMPLETE;
    }

    if (fullCycles >= PROB_TARGET_CYCLES) {
        s_probRunning = false;
        return IMU_RESULT_DONE;
    }

    if (fullCycles != s_lastDispCycles) {
        s_lastDispCycles = fullCycles;
        return IMU_RESULT_COUNT;
    }

    return IMU_RESULT_IDLE;
}

// ─────────────────────────────────────────────
//  imu_fill_packet
// ─────────────────────────────────────────────
void imu_fill_packet(SensorPacket_t *pkt,
                     uint8_t         subtype,
                     uint8_t         sample_index) {
    if (!pkt) return;

    memset(pkt, 0, sizeof(SensorPacket_t));
    pkt->sensor_id    = CMD_READ_LSM;
    pkt->status       = STATUS_OK;
    pkt->timestamp_ms = (uint32_t)millis();
    pkt->sample_count = 1;   // one logical record per packet

    ImuPacket_t *ip = reinterpret_cast<ImuPacket_t *>(pkt->payload);
    ip->subtype = subtype;

    switch (subtype) {

        case IMU_PKT_COUNT:
            ip->count.full_cycles = s_halfCycles / 2;
            break;

        case IMU_PKT_ROT_DATA:
            ip->rot_data.rotation_num = s_snap_cycleNum;
            ip->rot_data.duration_sec = s_snap_duration;
            ip->rot_data.angle_deg    = s_snap_angle;
            break;

        case IMU_PKT_SAMPLE:
            // sample_index is 0-based into the snapshot buffer
            if (sample_index < s_snap_sampleCount) {
                ip->sample.sample_index = sample_index + 1;      // 1-based for coordinator
                ip->sample.total        = s_snap_sampleCount;
                ip->sample.gyro_x_dps   = s_snap_samples[sample_index];
            } else {
                pkt->status = STATUS_ERR_SENS;
            }
            break;

        case IMU_PKT_DONE:
            ip->done.total_cycles = s_halfCycles / 2;
            break;

        case IMU_PKT_ERROR:
            strncpy(ip->error.message, "IMU init failed", sizeof(ip->error.message));
            pkt->status = STATUS_ERR_SENS;
            break;

        default:
            pkt->status = STATUS_ERR_SENS;
            break;
    }
}

// ─────────────────────────────────────────────
//  Accessors
// ─────────────────────────────────────────────
uint8_t  imu_sample_count(void) { return s_snap_sampleCount; }
uint32_t imu_full_cycles(void)  { return s_halfCycles / 2; }
bool     imu_is_ready(void)     { return s_imuReady; }