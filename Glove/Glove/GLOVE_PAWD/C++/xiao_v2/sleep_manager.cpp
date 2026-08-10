#include "sleep_manager.h"
#include "espnow_handler.h"          // for espnow_deinit()
#include "driver/rtc_io.h"
#include "esp_wifi.h"

// ─────────────────────────────────────────────
//  RTC-retained state (survives deep sleep)
// ─────────────────────────────────────────────

RTC_DATA_ATTR static uint32_t s_deep_sleep_count = 0;  // diagnostic counter

// ─────────────────────────────────────────────
//  Module-local state
// ─────────────────────────────────────────────

static gpio_num_t s_wakeup_pin   = SLEEP_WAKEUP_GPIO;
static uint8_t    s_wakeup_level = SLEEP_WAKEUP_LEVEL;
static uint32_t   s_last_activity_ms = 0;

// ─────────────────────────────────────────────
//  sleep_manager_init
// ─────────────────────────────────────────────

void sleep_manager_init(gpio_num_t pin, uint8_t level) {
    s_wakeup_pin   = pin;
    s_wakeup_level = level;

    // Configure as digital input with the appropriate pull resistor so the
    // pin is in a defined state while the coordinator is idle.
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << pin);
    cfg.mode         = GPIO_MODE_INPUT;
    cfg.pull_up_en   = (level == 0) ? GPIO_PULLUP_ENABLE   : GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = (level == 1) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
    cfg.intr_type    = GPIO_INTR_DISABLE;   // ESP-NOW RX drives wakeup, not a GPIO ISR
    gpio_config(&cfg);

    // RTC IO hold must be disabled to allow GPIO state changes after wakeup
    rtc_gpio_hold_dis(pin);

    // Seed the activity timer so we don't immediately deep-sleep on first boot
    s_last_activity_ms = millis();
}

// ─────────────────────────────────────────────
//  sleep_enter_light
// ─────────────────────────────────────────────

void sleep_enter_light(void) {
    // Enable GPIO wakeup source — level-triggered, not edge.
    // The pin must stay asserted until the CPU actually sleeps, so the
    // coordinator should hold it LOW for ≥ 5 ms before sending the packet.
    esp_sleep_enable_gpio_wakeup();
    gpio_wakeup_enable(s_wakeup_pin,
        s_wakeup_level == 0 ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL);

    // Wi-Fi modem sleep: keeps the MAC active for ESP-NOW but gates
    // the RF PLL when not actively receiving/transmitting (~2–3 mA idle).
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);

    // Blocks here until wakeup condition is met; returns normally.
    esp_light_sleep_start();

    // Re-enable full modem performance after wakeup for the TX burst.
    esp_wifi_set_ps(WIFI_PS_NONE);

    // Record wakeup as activity
    sleep_record_activity();
}

// ─────────────────────────────────────────────
//  sleep_enter_deep
// ─────────────────────────────────────────────

void sleep_enter_deep(uint64_t duration_us) {
    // Hold the wakeup GPIO configuration through the power cycle.
    // Without this the RTC IO floats and causes spurious wakeups.
    rtc_gpio_init(s_wakeup_pin);
    rtc_gpio_set_direction(s_wakeup_pin, RTC_GPIO_MODE_INPUT_ONLY);

    if (s_wakeup_level == 0) {
        rtc_gpio_pullup_en(s_wakeup_pin);
        rtc_gpio_pulldown_dis(s_wakeup_pin);
        esp_sleep_enable_ext0_wakeup(s_wakeup_pin, 0);  // Wake on LOW
    } else {
        rtc_gpio_pulldown_en(s_wakeup_pin);
        rtc_gpio_pullup_dis(s_wakeup_pin);
        esp_sleep_enable_ext0_wakeup(s_wakeup_pin, 1);  // Wake on HIGH
    }

    // Optional timer wakeup for periodic check-ins
    if (duration_us > 0) {
        esp_sleep_enable_timer_wakeup(duration_us);
    }

    s_deep_sleep_count++;

    // This function does not return — execution resumes at setup()
    esp_deep_sleep_start();
}

// ─────────────────────────────────────────────
//  Inactivity watchdog
// ─────────────────────────────────────────────

void sleep_activity_watchdog(void) {
#if SLEEP_DEEP_TIMEOUT_MS > 0
    if (millis() - s_last_activity_ms >= SLEEP_DEEP_TIMEOUT_MS) {
        // Cleanly tear down ESP-NOW before cutting power
        espnow_deinit();
        sleep_enter_deep(SLEEP_DEEP_DURATION_US);
        // Never reaches here
    }
#endif
}

void sleep_record_activity(void) {
    s_last_activity_ms = millis();
}

// ─────────────────────────────────────────────
//  Wakeup cause helpers
// ─────────────────────────────────────────────

WakeupCause_t sleep_wakeup_cause(void) {
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    switch (cause) {
        case ESP_SLEEP_WAKEUP_EXT0:
        case ESP_SLEEP_WAKEUP_EXT1:
        case ESP_SLEEP_WAKEUP_GPIO:
            return WAKEUP_CAUSE_GPIO;
        case ESP_SLEEP_WAKEUP_TIMER:
            return WAKEUP_CAUSE_TIMER;
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            // UNDEFINED means first boot / external reset
            return WAKEUP_CAUSE_RESET;
    }
}

bool sleep_woke_from_deep(void) {
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    // Any cause other than UNDEFINED means we woke from a sleep mode
    // (light or deep). For deep sleep specifically, RTC data will be valid
    // and s_deep_sleep_count will be > 0.
    return (cause != ESP_SLEEP_WAKEUP_UNDEFINED) && (s_deep_sleep_count > 0);
}