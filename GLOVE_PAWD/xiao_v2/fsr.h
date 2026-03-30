#pragma once

#include <Arduino.h>
#include "espnow_handler.h"   // for SensorPacket_t
#include "comms.h"

// ─────────────────────────────────────────────
//  Pin / tuning constants
//  Override via build_flags in platformio.ini
// ─────────────────────────────────────────────
#ifndef FSR_PIN
  #define FSR_PIN           A1
#endif
#ifndef FSR_THRESHOLD
  #define FSR_THRESHOLD     500
#endif
#ifndef FSR_DEBOUNCE_MS
  #define FSR_DEBOUNCE_MS   200
#endif
#ifndef FSR_TARGET_COUNT
  #define FSR_TARGET_COUNT  10
#endif

// Maximum press events that fit in SensorPacket_t.payload
// Each FsrSample_t is 10 bytes → 128 / 10 = 12 max
#define FSR_MAX_SAMPLES     12

/**
 * One press event — packed into SensorPacket_t.payload as a flat array.
 */


// ─────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────

/**
 * One-time hardware setup: ADC resolution, attenuation, timer init.
 * Call from setup() — does NOT start sampling.
 */
void fsr_init(void);

/**
 * Block until exactly ONE valid press is detected (above threshold,
 * debounced), then write it into the next slot of packet->payload.
 *
 * Updates packet->sample_count and packet->timestamp_ms (set on first call).
 * Returns the 1-based press index, or 0 on error (bad pointer / buffer full).
 *
 * The state machine calls this in a loop until the return value reaches
 * FSR_TARGET_COUNT, calling espnow_send() after each return.
 */
uint16_t fsr_capture_one(SensorPacket_t *packet);

/**
 * Reset internal state (triggerCount, fsrActive flag, timestamps).
 * Call at the start of each new STATE_READ_FSR activation so a
 * re-issued command starts a fresh count.
 */
void fsr_reset(void);

/**
 * Return true if the FSR pin is currently above threshold.
 * Useful for diagnostics / serial monitor.
 */
bool fsr_is_pressed(void);