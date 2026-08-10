#include "fsr.h"
#include <string.h>

// ─────────────────────────────────────────────
//  Hardware timer — used only for ADC polling
//  ESP-NOW / WiFi are owned by espnow_handler
//
//  FIX: Core 3.x timer API is completely different:
//    OLD: timerBegin(num, prescaler, countUp)
//    NEW: timerBegin(frequency_hz)
//
//    OLD: timerAttachInterrupt(t, fn, edge)
//    NEW: timerAttachInterrupt(t, fn)
//
//    OLD: timerAlarmWrite + timerAlarmEnable/Disable
//    NEW: timerAlarm(t, period, autoreload, count)
//         timerDetachInterrupt + timerEnd to stop
// ─────────────────────────────────────────────
static hw_timer_t*   s_timer     = nullptr;
static volatile bool s_sampleNow = false;
static volatile bool s_timerRunning = false;

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
//  Helper: start / stop the 100 Hz poll timer
//
//  Core 3.x has no timerAlarmEnable/Disable, so
//  we tear down and rebuild the timer to toggle.
// ─────────────────────────────────────────────
static void _fsr_timer_start(void) {
    if (s_timerRunning) return;
    s_timer = timerBegin(1000000);                  // 1 MHz = 1 µs ticks
    timerAttachInterrupt(s_timer, &_fsr_onTimer);   // no 3rd arg in Core 3.x
    timerAlarm(s_timer, 10000, true, 0);            // 10000 µs = 10 ms, autoreload, infinite
    s_timerRunning = true;
}

static void _fsr_timer_stop(void) {
    if (!s_timerRunning || !s_timer) return;
    timerDetachInterrupt(s_timer);
    timerEnd(s_timer);
    s_timer = nullptr;
    s_timerRunning = false;
}

// ─────────────────────────────────────────────
//  fsr_init
// ─────────────────────────────────────────────
void fsr_init(void) {
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    // Timer is NOT started here — _fsr_timer_start()
    // is called inside fsr_capture_one() on demand.
}

// ─────────────────────────────────────────────
//  fsr_reset
// ─────────────────────────────────────────────
void fsr_reset(void) {
    _fsr_timer_stop();
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
// ─────────────────────────────────────────────
uint16_t fsr_capture_one(SensorPacket_t *packet) {
    if (!packet) return 0;
    if (packet->sample_count >= FSR_MAX_SAMPLES) return 0;

    if (packet->sample_count == 0) {
        packet->timestamp_ms = (uint32_t)millis();
    }

    // Enable the ADC poll timer for the duration of this wait
    _fsr_timer_start();

    // ── Wait for one valid press ──────────────────────
    bool pressDetected = false;
    int  capturedAdc   = 0;
    uint32_t capturedTime = 0;

    while (!pressDetected) {
        vTaskDelay(1);
        if (!s_sampleNow) continue;
        s_sampleNow = false;

        int adcVal        = analogRead(FSR_PIN);
        unsigned long now = millis();

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

        if (s_fsrActive && adcVal < FSR_THRESHOLD) {
            s_fsrActive = false;
            Serial.printf("           [release] ADC = %4d\n", adcVal);
        }
    }

    // ── Disable timer until next call ────────────────
    _fsr_timer_stop();

    // ── Write sample into packet payload ─────────────
    FsrSample_t *samples = reinterpret_cast<FsrSample_t *>(packet->payload);
    uint16_t     slot    = packet->sample_count;

    samples[slot].timestamp_ms = capturedTime;
    samples[slot].adc_value    = (uint16_t)capturedAdc;
    samples[slot].count        = (uint16_t)s_triggerCount;
    samples[slot].reserved     = 0;

    packet->sample_count++;
    packet->sensor_id  = CMD_READ_FSR;
    packet->status     = STATUS_OK;

    return (uint16_t)s_triggerCount;
}