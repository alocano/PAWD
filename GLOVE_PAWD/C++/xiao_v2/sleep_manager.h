#pragma once

#include <Arduino.h>
#include "driver/gpio.h"
#include "esp_sleep.h"

// ─────────────────────────────────────────────
//  Configuration
// ─────────────────────────────────────────────

/**
 * GPIO used as the ESP-NOW hardware wakeup pin.
 * The coordinator (or a dedicated sync line) drives this LOW to
 * signal an incoming command before sending the ESP-NOW packet.
 * Must be a RTC-capable GPIO. On the XIAO ESP32-S3:
 *   D0=GPIO1, D1=GPIO2, D2=GPIO3, D3=GPIO4, D4=GPIO5, D5=GPIO6
 *   D6=GPIO43(TX), D7=GPIO44(RX), D8=GPIO7, D9=GPIO8, D10=GPIO9
 * Recommended: D0 (GPIO1) — free of strapping / UART concerns.
 */
#ifndef SLEEP_WAKEUP_GPIO
  #define SLEEP_WAKEUP_GPIO       GPIO_NUM_1
#endif

/**
 * Active-low or active-high wakeup level for the GPIO pin.
 * Default: LOW (pin pulled high via internal/external resistor,
 * coordinator drives it LOW to assert wakeup).
 */
#ifndef SLEEP_WAKEUP_LEVEL
  #define SLEEP_WAKEUP_LEVEL      0   // 0 = LOW, 1 = HIGH
#endif

/**
 * After this many milliseconds of inactivity (no command received),
 * the device escalates from light sleep to deep sleep to save power.
 * Set to 0 to disable deep-sleep escalation.
 */
#ifndef SLEEP_DEEP_TIMEOUT_MS
  #define SLEEP_DEEP_TIMEOUT_MS   30000UL   // 30 s
#endif

/**
 * Maximum time (µs) the device stays in deep sleep before waking to
 * check for missed activity. 0 = sleep indefinitely until GPIO wakeup.
 * Only relevant when deep sleep escalation is enabled.
 */
#ifndef SLEEP_DEEP_DURATION_US
  #define SLEEP_DEEP_DURATION_US  0ULL
#endif

// ─────────────────────────────────────────────
//  Wakeup cause helpers
// ─────────────────────────────────────────────

/**
 * Reasons the device woke up — returned by sleep_wakeup_cause().
 */
typedef enum {
    WAKEUP_CAUSE_UNKNOWN  = 0,
    WAKEUP_CAUSE_GPIO     = 1,   // Hardware wakeup pin asserted
    WAKEUP_CAUSE_TIMER    = 2,   // RTC timer expired (deep sleep only)
    WAKEUP_CAUSE_RESET    = 3,   // Power-on or external reset
} WakeupCause_t;

// ─────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────

/**
 * One-time setup: configure the wakeup GPIO (input + pull-up/pull-down).
 * Call once from setup() before entering any sleep mode.
 *
 * @param pin    GPIO number (default: SLEEP_WAKEUP_GPIO).
 * @param level  Wakeup-active level — 0=LOW, 1=HIGH
 *               (default: SLEEP_WAKEUP_LEVEL).
 */
void sleep_manager_init(gpio_num_t pin   = SLEEP_WAKEUP_GPIO,
                        uint8_t    level = SLEEP_WAKEUP_LEVEL);

/**
 * Enter light sleep.
 * - CPU is halted, RAM and peripheral state are preserved.
 * - ESP-NOW / Wi-Fi stack is NOT powered down; wakeup latency ~2 ms.
 * - Returns immediately when the wakeup GPIO is asserted.
 * - Use this as the primary idle state between commands.
 */
void sleep_enter_light(void);

/**
 * Enter deep sleep.
 * - Almost all power domains are cut (~20–50 µA typical).
 * - Execution restarts from setup() on wakeup — call espnow_init()
 *   again and restore state from RTC_DATA_ATTR variables.
 * - duration_us == 0 → sleep until GPIO wakeup only (no timer).
 *
 * NOTE: Call espnow_deinit() BEFORE this function so the Wi-Fi
 *       MAC is properly shut down before power is cut.
 *
 * @param duration_us  RTC timer wakeup in microseconds (0 = disabled).
 */
void sleep_enter_deep(uint64_t duration_us = SLEEP_DEEP_DURATION_US);

/**
 * Run the inactivity watchdog.
 * Call this every loop iteration (cheap — just checks millis()).
 * If no activity has been recorded for SLEEP_DEEP_TIMEOUT_MS the
 * function calls espnow_deinit() and sleep_enter_deep() automatically.
 *
 * Requires SLEEP_DEEP_TIMEOUT_MS > 0 to have any effect.
 */
void sleep_activity_watchdog(void);

/**
 * Record activity — resets the inactivity timer used by the watchdog.
 * Call this whenever a command is received or a TX completes.
 */
void sleep_record_activity(void);

/**
 * Return the cause of the most recent wakeup.
 * Useful in setup() to distinguish cold boot from sleep wakeup.
 */
WakeupCause_t sleep_wakeup_cause(void);

/**
 * Return true if the device woke from deep sleep
 * (i.e. not a cold power-on or external reset).
 */
bool sleep_woke_from_deep(void);