#pragma once

// ═══════════════════════════════════════════════════════════════
//  comms.h  —  shared between WROOM (coordinator) and XIAO (sensor node)
//
//  Include this file on BOTH boards. Never define packet structs
//  anywhere else. If a field changes here it changes everywhere.
// ═══════════════════════════════════════════════════════════════

#include <stdint.h>

// ─────────────────────────────────────────────
//  ESP-NOW channel  (must match on both boards)
// ─────────────────────────────────────────────
#define COMMS_CHANNEL       6

// ─────────────────────────────────────────────
//  Command bytes  (WROOM → XIAO)
// ─────────────────────────────────────────────
#define CMD_READ_FSR        0x01   // Start FSR tap session
#define CMD_READ_LSM        0x02   // Start IMU PROB session
#define CMD_PING            0xF0   // Health-check; XIAO replies with STATUS_OK

// ─────────────────────────────────────────────
//  Status codes  (inside every SensorPacket_t)
// ─────────────────────────────────────────────
#define STATUS_OK           0x00
#define STATUS_ERR_SENS     0x01   // Sensor read failed
#define STATUS_ERR_TX       0x02   // TX failed after retries
#define STATUS_ERR_CMD      0xFF   // Unknown command

// ─────────────────────────────────────────────
//  IMU packet subtypes  (inside ImuPacket_t)
//  Numbers kept close to the original stateIDs
//  so the WROOM display logic maps 1-to-1.
// ─────────────────────────────────────────────
#define IMU_PKT_COUNT       0xC3   // stateID 203 — live full-cycle count
#define IMU_PKT_ROT_DATA    0xCC   // stateID 204 — per-rotation summary
#define IMU_PKT_SAMPLE      0xCD   // stateID 205 — raw gyro snapshot
#define IMU_PKT_DONE        0xC2   // stateID 202 — session complete
#define IMU_PKT_ERROR       0xFF

// ─────────────────────────────────────────────
//  Maximum sensor payload bytes
// ─────────────────────────────────────────────
#define ESPNOW_MAX_PAYLOAD  128

// ═══════════════════════════════════════════════════════════════
//  CommandPacket_t  —  WROOM → XIAO
//  Replaces the old struct_message used as a command.
// ═══════════════════════════════════════════════════════════════
typedef struct __attribute__((packed)) {
    uint8_t  command;             // CMD_READ_FSR / CMD_READ_LSM / CMD_PING
    uint16_t sample_count_req;    // Requested samples (0 = use sensor default)
    uint32_t seq;                 // Sequence number — XIAO deduplicates on this
} CommandPacket_t;                // 7 bytes total

// ═══════════════════════════════════════════════════════════════
//  SensorPacket_t  —  XIAO → WROOM
//  Single unified outbound packet for both sensors.
// ═══════════════════════════════════════════════════════════════
typedef struct __attribute__((packed)) {
    uint8_t  sensor_id;                       // CMD_READ_FSR or CMD_READ_LSM
    uint8_t  status;                          // STATUS_* constant
    uint32_t timestamp_ms;                    // millis() at capture start
    uint16_t sample_count;                    // Valid records in payload
    uint8_t  payload[ESPNOW_MAX_PAYLOAD];     // Sensor-specific data (below)
    uint8_t  checksum;                        // XOR of all preceding bytes
} SensorPacket_t;                             // 136 bytes total

// ═══════════════════════════════════════════════════════════════
//  FSR payload  —  packed into SensorPacket_t.payload[]
//  One FsrSample_t per press event.
//  Max presses per packet: ESPNOW_MAX_PAYLOAD / sizeof = 12
// ═══════════════════════════════════════════════════════════════
#define FSR_MAX_SAMPLES     12

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;   // millis() at press detection
    uint16_t adc_value;      // Raw 12-bit ADC reading
    uint16_t count;          // 1-based press index
    uint16_t reserved;       // Pad to 10 bytes
} FsrSample_t;               // 10 bytes

// ═══════════════════════════════════════════════════════════════
//  IMU payload  —  packed into SensorPacket_t.payload[]
//  subtype selects which union member is valid.
// ═══════════════════════════════════════════════════════════════
typedef struct __attribute__((packed)) {
    uint8_t subtype;          // IMU_PKT_* constant

    union {
        // IMU_PKT_COUNT  (stateID 203)
        struct { uint32_t full_cycles; } count;

        // IMU_PKT_ROT_DATA  (stateID 204)
        struct {
            uint32_t rotation_num;
            float    duration_sec;
            float    angle_deg;
        } rot_data;

        // IMU_PKT_SAMPLE  (stateID 205)
        struct {
            uint8_t sample_index;   // 1-based
            uint8_t total;
            float   gyro_x_dps;
        } sample;

        // IMU_PKT_DONE  (stateID 202)
        struct { uint32_t total_cycles; } done;

        // IMU_PKT_ERROR
        struct { char message[32]; } error;
    };
} ImuPacket_t;               // 13 bytes worst-case (rot_data branch)