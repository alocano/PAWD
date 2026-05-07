#include "espnow_handler.h"
//#include "sleep_manager.h"
#include "fsr.h"
#include "imu.h"
#include "comms.h" 

// ─────────────────────────────────────────────
//  Coordinator MAC — update to match your device
//  Open WROOM Serial Monitor and look for:
//    [WROOM] My MAC: XX:XX:XX:XX:XX:XX
// ─────────────────────────────────────────────
static const uint8_t COORDINATOR_MAC[6] = {0x20, 0xE7, 0xC8, 0xAB, 0xED, 0x24};
// Esp32-s3 Wroom Mac : 0xDC, 0xB4, 0xD9, 0x05, 0x25, 0xF8
//0x20, 0xE7, 0xC8, 0xAB, 0xED, 0x24
//0x68, 0xFE, 0x71, 0xF9, 0xAD0 ,0x40

// ─────────────────────────────────────────────
//  State machine  (STATE_SLEEP removed)
// ─────────────────────────────────────────────
typedef enum {
    STATE_IDLE,        // was STATE_SLEEP — WiFi stays ON now
    STATE_READ_FSR,
    STATE_READ_LSM,
    STATE_TRANSMIT,    // single-shot TX for LSM error path
} AppState_t;

static volatile AppState_t s_state       = STATE_IDLE;   // was STATE_SLEEP
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

    Serial.printf("[XIAO] Got cmd=0x%02X\n", cmd->command);
}

void on_send_done(bool success) { (void)success; }

// ─────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────

// Send a packet; returns espnow_send() result.
static bool _send(SensorPacket_t *pkt) {
    bool ok = espnow_send(pkt);
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
    delay(1000);
    Serial.begin(115200);
    unsigned long start = millis();
    while (!Serial && (millis() - start < 3000)) {
        delay(10);
    }

    // Print MAC so you can verify xiaoAddress on the WROOM
    WiFi.mode(WIFI_STA);
    delay(100);
    Serial.print("[XIAO] My MAC: ");
    Serial.println(WiFi.macAddress());

    
    fsr_init();
    // imu_init() is deferred to STATE_READ_LSM so we only power the
    // I2C bus when the coordinator actually requests IMU data.

    bool ok = espnow_init(COORDINATOR_MAC, on_command, on_send_done);
    if (!ok) {
        Serial.println("ESP-NOW init failed — retrying in 5 s");
        delay(5000);
        ESP.restart();
    }

    Serial.println("Ready — waiting for command.");
    s_state = STATE_IDLE;   // was STATE_SLEEP
}

// ─────────────────────────────────────────────
//  loop()
// ─────────────────────────────────────────────
void loop() {
    // sleep_activity_watchdog() removed — it called sleep_enter_deep()
    // after 30s of inactivity, which also kills WiFi

    switch (s_state) {

        // ── IDLE — WiFi stays ON, ESP-NOW callback fires normally ──
        case STATE_IDLE:
            if      (s_pending_cmd == CMD_READ_FSR) { s_state = STATE_READ_FSR; s_pending_cmd = 0; }
            else if (s_pending_cmd == CMD_READ_LSM) { s_state = STATE_READ_LSM; s_pending_cmd = 0; }
            else    delay(10);   // yield, WiFi stays alive
            break;

        // ─────────────────────────────────────────────────────────────
        //  FSR — block on each press, transmit immediately after each
        // ─────────────────────────────────────────────────────────────
        case STATE_READ_FSR: {
            Serial.println("IN FSR");
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

            s_state = STATE_IDLE;   // was STATE_SLEEP
            break;
        }

        // ─────────────────────────────────────────────────────────────
        //  IMU — non-blocking tick loop
        // ─────────────────────────────────────────────────────────────
        case STATE_READ_LSM: {
            Serial.println("IN IMU");
            // Initialise IMU on first entry; bail out if not found
            if (!imu_is_ready()) {
                if (!imu_init()) {
                    imu_fill_packet(&s_tx_packet, IMU_PKT_ERROR);
                    _send(&s_tx_packet);
                    s_state = STATE_IDLE;   // was STATE_SLEEP
                    break;
                }
            }

            // Reset PROB state for a fresh session
            imu_reset();

            // ── Non-blocking PROB loop ────────────────────────────────
            bool sessionDone = false;
            while (!sessionDone) {

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
            s_state = STATE_IDLE;   // was STATE_SLEEP
            break;
        }

        // ── Generic single transmit (error / ping paths) ──────────────
        case STATE_TRANSMIT:
            Serial.println("IN Transmitt");
            _send(&s_tx_packet);
            s_state = STATE_IDLE;   // was STATE_SLEEP
            break;
    }
}
