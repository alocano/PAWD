#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DS3TRC.h>
#include "espnow_handler.h"   // for SensorPacket_t, CMD_READ_LSM, STATUS_*
#include "comms.h"

// ─────────────────────────────────────────────
//  I2C pins (XIAO ESP32-S3)
// ─────────────────────────────────────────────
#ifndef IMU_SDA_PIN
  #define IMU_SDA_PIN   4
#endif
#ifndef IMU_SCL_PIN
  #define IMU_SCL_PIN   5
#endif

// ─────────────────────────────────────────────
//  PROB algorithm tuning
// ─────────────────────────────────────────────
static const float    IMU_ALPHA         = 0.01f;  // Bias filter coefficient
static const float    IMU_REST_THR      = 7.0f;   // dps — bias update threshold
static const float    IMU_DEADBAND      = 25.0f;  // dps — zero-crossing deadband
static const float    IMU_PEAK_THR      = 50.0f;  // dps — valid half-cycle peak
static const float    IMU_MIN_GAP_S     = 0.25f;  // s   — min time between crossings

static const uint32_t PROB_TARGET_CYCLES    = 10;
static const uint32_t PROB_SAMPLE_MS        = 25;
static const float    PROB_SAMPLE_SEC       = PROB_SAMPLE_MS / 1000.0f;

// ─────────────────────────────────────────────
//  Per-rotation sample buffer
// ─────────────────────────────────────────────
#define ROT_MAX_SAMPLES      8
#define ROT_SAMPLE_INTERVAL  5   // 1 sample every 5 ticks = every 125 ms

// ─────────────────────────────────────────────
//  Packet subtypes packed into ImuPacket_t
//  These map to the original stateIDs:
//    IMU_PKT_COUNT    ← stateID 203  live full-cycle count
//    IMU_PKT_ROT_DATA ← stateID 204  per-rotation summary
//    IMU_PKT_SAMPLE   ← stateID 205  raw gyro sample
//    IMU_PKT_DONE     ← stateID 202  session complete
// ─────────────────────────────────────────────
#define IMU_PKT_COUNT     0xC3   // 203
#define IMU_PKT_ROT_DATA  0xCC   // 204
#define IMU_PKT_SAMPLE    0xCD   // 205
#define IMU_PKT_DONE      0xC2   // 202
#define IMU_PKT_ERROR     0xFF

/**
 * Structured payload written into SensorPacket_t.payload.
 * subtype selects which union member is valid.
 * Total size must be <= ESPNOW_MAX_PAYLOAD (128 bytes).
 */


// Compile-time guard: ImuPacket_t must fit inside SensorPacket_t.payload
static_assert(sizeof(ImuPacket_t) <= ESPNOW_MAX_PAYLOAD,
              "ImuPacket_t exceeds SensorPacket_t payload capacity");

// ─────────────────────────────────────────────
//  Return codes from imu_run_sample()
// ─────────────────────────────────────────────
typedef enum {
    IMU_RESULT_IDLE        = 0,  // Nothing new this tick
    IMU_RESULT_COUNT       = 1,  // Live count changed  → send IMU_PKT_COUNT
    IMU_RESULT_DONE        = 2,  // Target reached      → send IMU_PKT_DONE
    IMU_RESULT_ROT_COMPLETE= 3,  // Full rotation done  → send samples + rot_data + count
} ImuResult_t;

// ─────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────

/**
 * Initialise I2C and probe the IMU at 0x6A then 0x6B.
 * Call once from setup(). Safe to call again after deep-sleep wakeup.
 * Returns true on success.
 */
bool imu_init(void);

/**
 * Reset all PROB session state.
 * Call at the start of every STATE_READ_LSM activation.
 */
void imu_reset(void);

/**
 * Run one sample tick (non-blocking).
 * Call every loop() iteration while in STATE_READ_LSM.
 * Respects the PROB_SAMPLE_MS interval internally — returns
 * IMU_RESULT_IDLE immediately if it's not yet time to sample.
 *
 * When the return code is not IMU_RESULT_IDLE the caller should
 * use imu_fill_packet() to populate a SensorPacket_t, then transmit.
 */
ImuResult_t imu_run_sample(void);

/**
 * Fill a SensorPacket_t with the payload appropriate for `result`.
 *
 * For IMU_RESULT_ROT_COMPLETE this must be called once per sample
 * (imu_sample_count() times) to get IMU_PKT_SAMPLE packets, then
 * once more with fill_rot_data=true to get the IMU_PKT_ROT_DATA
 * packet, then once for the IMU_PKT_COUNT packet.
 *
 * @param pkt           Output packet (cleared by this function).
 * @param result        The ImuResult_t returned by imu_run_sample().
 * @param subtype       Which IMU_PKT_* to write (caller drives the sequence).
 * @param sample_index  0-based index into the rotation sample buffer
 *                      (only used when subtype == IMU_PKT_SAMPLE).
 */
void imu_fill_packet(SensorPacket_t *pkt,
                     uint8_t         subtype,
                     uint8_t         sample_index = 0);

/**
 * Number of gyro samples collected in the current rotation buffer.
 * Valid after imu_run_sample() returns IMU_RESULT_ROT_COMPLETE.
 */
uint8_t imu_sample_count(void);

/**
 * Current full-cycle count.
 */
uint32_t imu_full_cycles(void);

/**
 * True if the IMU was successfully initialised.
 */
bool imu_is_ready(void);