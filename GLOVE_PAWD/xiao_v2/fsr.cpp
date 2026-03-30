#include "fsr.h"
#include <string.h>

// ─────────────────────────────────────────────
//  Hardware timer — used only for ADC polling
//  ESP-NOW / WiFi are owned by espnow_handler
// ─────────────────────────────────────────────
static hw_timer_t*   s_timer     = nullptr;
static volatile bool s_sampleNow = false;

void IRAM_ATTR _fsr_onTimer() {
    s_sampleNow = true;
}

// ─────────────────────────────────────────────
//  Module state
// ─────────────────────────────────────────────
static volatile int           s_triggerCount    = 0;
static volatile unsigned long s_lastTriggerTime = 0;
static bool                   s_fsrActive       = false;

// ─────────────────────────────────────────────
//  fsr_init
// ─────────────────────────────────────────────
void fsr_init(void) {
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // Timer initialised but alarm deliberately NOT enabled here.
    // fsr_capture_one() enables it, fsr_reset() disables it,
    // keeping the ISR silent when the FSR state is inactive.
    s_timer = timerBegin(0, 80, true);          // 1 µs tick
    timerAttachInterrupt(s_timer, &_fsr_onTimer, true);
    timerAlarmWrite(s_timer, 10000, true);       // fire every 10 ms
    // timerAlarmEnable() intentionally omitted
}

// ─────────────────────────────────────────────
//  fsr_reset
// ─────────────────────────────────────────────
void fsr_reset(void) {
    if (s_timer) timerAlarmDisable(s_timer);
    s_triggerCount    = 0;
    s_lastTriggerTime = 0;
    s_fsrActive       = false;
    s_sampleNow       = false;
}

// ─────────────────────────────────────────────
//  fsr_is_pressed  (diagnostic helper)
// ─────────────────────────────────────────────
bool fsr_is_pressed(void) {
    return analogRead(FSR_PIN) >= FSR_THRESHOLD;
}

// ─────────────────────────────────────────────
//  fsr_capture_one
//
//  Blocks inside a tight yield-loop until one debounced press event
//  occurs, then writes a FsrSample_t into the packet payload and
//  returns the updated press count.
//
//  The state machine in main.cpp calls this, then immediately calls
//  espnow_send() so the coordinator gets a packet per press.
// ─────────────────────────────────────────────
uint16_t fsr_capture_one(SensorPacket_t *packet) {
    if (!packet) return 0;

    // Guard: payload can hold at most FSR_MAX_SAMPLES entries
    if (packet->sample_count >= FSR_MAX_SAMPLES) return 0;

    // Stamp start time on the very first press of this session
    if (packet->sample_count == 0) {
        packet->timestamp_ms = (uint32_t)millis();
    }

    // Enable the ADC poll timer for the duration of this wait
    timerAlarmEnable(s_timer);

    // ── Wait for one valid press ──────────────────────────────────────
    bool pressDetected = false;
    int  capturedAdc   = 0;
    uint32_t capturedTime = 0;

    while (!pressDetected) {
        // Yield to the RTOS / ESP-NOW WiFi task while waiting
        // so the Wi-Fi stack stays healthy during the blocking loop
        vTaskDelay(1);

        if (!s_sampleNow) continue;
        s_sampleNow = false;

        int adcVal        = analogRead(FSR_PIN);
        unsigned long now = millis();

        // Rising edge: pin crossed threshold and debounce window passed
        if (!s_fsrActive && adcVal >= FSR_THRESHOLD) {
            if ((now - s_lastTriggerTime) >= FSR_DEBOUNCE_MS) {
                s_fsrActive       = true;
                s_lastTriggerTime = now;
                s_triggerCount++;

                capturedAdc   = adcVal;
                capturedTime  = (uint32_t)now;
                pressDetected = true;

                Serial.printf("[PRESS #%02d]  ADC = %4d\n",
                              s_triggerCount, adcVal);
            }
        }

        // Track release so next press edge is detected correctly
        if (s_fsrActive && adcVal < FSR_THRESHOLD) {
            s_fsrActive = false;
            Serial.printf("           [release] ADC = %4d\n", adcVal);
        }
    }

    // ── Disable timer until next call ────────────────────────────────
    timerAlarmDisable(s_timer);

    // ── Write sample into packet payload ─────────────────────────────
    FsrSample_t *samples = reinterpret_cast<FsrSample_t *>(packet->payload);
    uint16_t     slot    = packet->sample_count;   // 0-based slot index

    samples[slot].timestamp_ms = capturedTime;
    samples[slot].adc_value    = (uint16_t)capturedAdc;
    samples[slot].count        = (uint16_t)s_triggerCount;
    samples[slot].reserved     = 0;

    packet->sample_count++;     // advance packet slot counter
    packet->sensor_id  = CMD_READ_FSR;
    packet->status     = STATUS_OK;

    return (uint16_t)s_triggerCount;
}