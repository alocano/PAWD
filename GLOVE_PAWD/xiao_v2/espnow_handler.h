#pragma once

// All packet structs, command bytes, and status codes live in comms.h.
// Both boards include comms.h — this file adds the XIAO-side transport API.
#include "comms.h"
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"

// ─────────────────────────────────────────────
//  Transport configuration
// ─────────────────────────────────────────────
#define ESPNOW_RETRY_LIMIT    3
#define ESPNOW_TX_TIMEOUT_MS  500

// ─────────────────────────────────────────────
//  Callback types
// ─────────────────────────────────────────────
typedef void (*espnow_cmd_callback_t)(const CommandPacket_t *cmd);
typedef void (*espnow_send_callback_t)(bool success);

// ─────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────
bool espnow_init(const uint8_t coordinator_mac[6],
                 espnow_cmd_callback_t  on_command,
                 espnow_send_callback_t on_send_done = nullptr);

bool espnow_send(SensorPacket_t *packet);
bool espnow_send_ack(uint8_t status_code);
void espnow_deinit(void);
esp_now_send_status_t espnow_last_send_status(void);