#include <Arduino.h>
#include "espnow_handler.h"
#include "sleep_manager.h"
#include "fsr.h"
#include "imu.h"
#include "comms.h" 

// ─────────────────────────────────────────────
//  Coordinator MAC — update to match your device
// ─────────────────────────────────────────────
static const uint8_t COORDINATOR_MAC[6] = { 0x20, 0xE7, 0xC8, 0xAB, 0xED, 0x24 };

// ─────────────────────────────────────────────
//  State machine
// ─────────────────────────────────────────────
typedef enum {
    STATE_SLEEP,
    STATE_IDLE,
    STATE_READ_FSR,
    STATE_READ_LSM,
    STATE_TRANSMIT,    // single-shot TX for LSM error path
} AppState_t;

static volatile AppState_t s_state       = STATE_SLEEP;
static volatile uint8_t    s_pending_cmd = 0;
static volatile uint16_t   s_sample_req  = 0;
static SensorPacket_t      s_tx_packet   = {};

// ─────────────────────────────────────────────
//  ESP-NOW callbacks  (ISR context — no blocking)
// ─────────────────────────────────────────────
void IRAM_ATTR on_command(const CommandPacket_t *cmd) {
    if (cmd->command == CMD_PING) {
        espnow_send_ack(STATUS_OK);
        return;
    }
    s_pending_cmd = cmd->command;
    s_sample_req  = cmd->sample_count_req;
    s_state       = STATE_IDLE;
    sleep_record_activity();
}

void on_send_done(bool success) { (void)success; }

// ─────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────

// Send a packet and record activity; returns espnow_send() result.
static bool _send(SensorPacket_t *pkt) {
    bool ok = espnow_send(pkt);
    sleep_record_activity();
    return ok;
}

// Send the three-packet burst produced after every full IMU rotation:
//   1. One IMU_PKT_SAMPLE packet per captured gyro snapshot  (stateID 205)
//   2. One IMU_PKT_ROT_DATA packet with summary              (stateID 204)
//   3. One IMU_PKT_COUNT   packet with running cycle count   (stateID 203)
static void _imu_send_rotation_burst(void) {
    // 1. Raw gyro samples (one packet each, mirrors original stateID 205 × N)
    uint8_t nSamples = imu_sample_count();
    for (uint8_t i = 0; i < nSamples; i++) {
        imu_fill_packet(&s_tx_packet, IMU_PKT_SAMPLE, i);
        if (!_send(&s_tx_packet)) {
            Serial.printf("[IMU] Sample %d TX failed\n", i + 1);
        }
        delay(5);   // brief gap — coordinator needs time to process
    }

    // 2. Per-rotation summary
    imu_fill_packet(&s_tx_packet, IMU_PKT_ROT_DATA);
    if (!_send(&s_tx_packet)) {
        Serial.println("[IMU] ROT_DATA TX failed");
    }
    delay(5);

    // 3. Live count update
    imu_fill_packet(&s_tx_packet, IMU_PKT_COUNT);
    _send(&s_tx_packet);
}

// ─────────────────────────────────────────────
//  setup()
// ─────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("setup1");
    sleep_manager_init();
    fsr_init();
    // imu_init() is deferred to STATE_READ_LSM so we only power the
    // I2C bus when the coordinator actually requests IMU data.

    if (sleep_woke_from_deep()) {
        Serial.println("Woke from deep sleep");
    }

    bool ok = espnow_init(COORDINATOR_MAC, on_command, on_send_done);
    if (!ok) {
        Serial.println("ESP-NOW init failed — retrying in 5 s");
        sleep_enter_deep(5000000ULL);
    }

    Serial.println("Ready — waiting for command.");
    s_state = STATE_SLEEP;
}

// ─────────────────────────────────────────────
//  loop()
// ─────────────────────────────────────────────
void loop() {
    sleep_activity_watchdog();

    switch (s_state) {

        // ── Light sleep until coordinator asserts wakeup GPIO ─────────
        case STATE_SLEEP:
            sleep_enter_light();
            s_state = STATE_IDLE;
            break;

        // ── Route to the correct sensor state ────────────────────────
        case STATE_IDLE:
            if      (s_pending_cmd == CMD_READ_FSR) s_state = STATE_READ_FSR;
            else if (s_pending_cmd == CMD_READ_LSM) s_state = STATE_READ_LSM;
            else                                     s_state = STATE_SLEEP;
            s_pending_cmd = 0;
            break;

        // ─────────────────────────────────────────────────────────────
        //  FSR — block on each press, transmit immediately after each
        // ─────────────────────────────────────────────────────────────
        case STATE_READ_FSR: {
            uint16_t target = (s_sample_req > 0 &&
                               s_sample_req <= FSR_MAX_SAMPLES)
                              ? s_sample_req : FSR_TARGET_COUNT;

            memset(&s_tx_packet, 0, sizeof(s_tx_packet));
            fsr_reset();

            while (true) {
                uint16_t pressIndex = fsr_capture_one(&s_tx_packet);

                if (pressIndex == 0) {
                    s_tx_packet.status = STATUS_ERR_SENS;
                    _send(&s_tx_packet);
                    break;
                }

                if (!_send(&s_tx_packet)) {
                    Serial.printf("[FSR] Press #%d TX failed\n", pressIndex);
                }

                if (pressIndex >= target) {
                    Serial.println("[FSR] Target count reached.");
                    break;
                }
            }

            s_state = STATE_SLEEP;
            break;
        }

        // ─────────────────────────────────────────────────────────────
        //  IMU — non-blocking tick loop
        //
        //  imu_run_sample() is called every iteration; it returns
        //  immediately if PROB_SAMPLE_MS has not elapsed yet, so
        //  loop() remains responsive to the watchdog throughout.
        //
        //  Packet burst sequence per rotation (mirrors original):
        //    stateID 205 × N  (raw samples)
        //    stateID 204      (rotation summary)
        //    stateID 203      (live count)
        //  On session complete:
        //    stateID 202      (done)
        // ─────────────────────────────────────────────────────────────
        case STATE_READ_LSM: {
            // Initialise IMU on first entry; bail out if not found
            if (!imu_is_ready()) {
                if (!imu_init()) {
                    imu_fill_packet(&s_tx_packet, IMU_PKT_ERROR);
                    _send(&s_tx_packet);
                    s_state = STATE_SLEEP;
                    break;
                }
            }

            // Reset PROB state for a fresh session
            imu_reset();

            // ── Non-blocking PROB loop ────────────────────────────────
            bool sessionDone = false;
            while (!sessionDone) {
                // Let the watchdog run even during the IMU session.
                // Activity is recorded on each TX so the timer stays
                // fresh as long as the IMU is producing data.
                sleep_activity_watchdog();

                ImuResult_t result = imu_run_sample();

                switch (result) {

                    case IMU_RESULT_IDLE:
                        // Nothing to do this tick — yield to RTOS
                        vTaskDelay(1);
                        break;

                    case IMU_RESULT_COUNT:
                        // Live count changed — send one COUNT packet (stateID 203)
                        imu_fill_packet(&s_tx_packet, IMU_PKT_COUNT);
                        if (!_send(&s_tx_packet)) {
                            Serial.println("[IMU] COUNT TX failed");
                        }
                        break;

                    case IMU_RESULT_ROT_COMPLETE:
                        // Full rotation — send samples + summary + count
                        _imu_send_rotation_burst();

                        // Check if we've now hit the target
                        if (imu_full_cycles() >= PROB_TARGET_CYCLES) {
                            imu_fill_packet(&s_tx_packet, IMU_PKT_DONE);
                            _send(&s_tx_packet);
                            sessionDone = true;
                        }
                        break;

                    case IMU_RESULT_DONE:
                        // Target reached mid-tick without a full rotation event
                        imu_fill_packet(&s_tx_packet, IMU_PKT_DONE);
                        _send(&s_tx_packet);
                        sessionDone = true;
                        break;
                }
            }

            Serial.println("[IMU] Session complete.");
            s_state = STATE_SLEEP;
            break;
        }

        // ── Generic single transmit (error / ping paths) ──────────────
        case STATE_TRANSMIT:
            _send(&s_tx_packet);
            s_state = STATE_SLEEP;
            break;
    }
}