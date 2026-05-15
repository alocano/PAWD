// ═══════════════════════════════════════════════════════════════
//  wroom.cpp  —  Coordinator  (ESP32 WROOM + ST7735 + Rotary Encoder)
//
//  Packet layer is now driven by comms.h, shared with the XIAO.
//  All struct_message / stateID code has been replaced.
// ═══════════════════════════════════════════════════════════════


#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Encoder.h>
#include "esp_wifi.h"
#include "comms.h"            // shared packet definitions

// ─────────────────────────────────────────────
//  Pin definitions
// ─────────────────────────────────────────────

//#define TFT_SDA  11   // SDA on module, aka mosi
//#define TFT_SCLK  12 
#define TFT_CS     5   // old Wroom 5 New  Wroom 10
#define TFT_RST    4 // " 4  9
#define TFT_DC     2   // " 2  8

#define ENC_A    32  //32    4
#define ENC_B    35  //35    5   
#define ENC_SW   34//34  6

// ─────────────────────────────────────────────
//  XIAO sensor-node MAC  (update to match your board)
// ─────────────────────────────────────────────
uint8_t xiaoAddress[] = {0xDC,0xB4,0xD9,0x3B,0x47,0x4C};
//0x9C, 0x13, 0x9E, 0xAB, 0xD5, 0xA0
//0x1c, 0xdb, 0xd4, 0x76, 0x1a, 0xd4
// ─────────────────────────────────────────────
//  Peripherals
// ─────────────────────────────────────────────
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

ESP32Encoder    encoder;

// ─────────────────────────────────────────────
//  State machine
// ─────────────────────────────────────────────
enum CoordState { CHOICE, SENDING_CMD, WAITING, RECEIVING, RECEIVED };
CoordState currentState = CHOICE;

// ─────────────────────────────────────────────
//  Command sequencing
// ─────────────────────────────────────────────
static uint32_t s_seq         = 0;    // incremented on every send
static uint8_t  s_pendingCmd  = 0;    // set by menu, consumed by SENDING_CMD

// ─────────────────────────────────────────────
//  Incoming data buffers
// ─────────────────────────────────────────────
static SensorPacket_t s_rxPacket = {};   // last complete packet from XIAO
static volatile bool  s_newPacket = false;

// Decoded FSR session results (accumulated across per-press packets)
static uint16_t s_fsrLastCount = 0;
static uint16_t s_fsrLastAdc   = 0;
static uint16_t s_fsrLastTime =0;

// Decoded IMU session results (latest values for display)
static uint32_t s_imuCycles    = 0;
static uint32_t s_imuRotNum    = 0;
static float    s_imuDuration  = 0.0f;
static float    s_imuAngle     = 0.0f;

// ─────────────────────────────────────────────
//  Menu state
// ─────────────────────────────────────────────
static int  selection     =  0;
static int  lastSelection = -1;
static bool isMenuDrawn   = false;
static int  screenclear   =  0;

// Non-blocking waiting animation
static unsigned long lastAnimTime  = 0;
static int           dotCount      = 0;
static const int     animInterval  = 500;

// ═══════════════════════════════════════════════════════════════
//  TFT helpers  (unchanged from original)
// ═══════════════════════════════════════════════════════════════

void updateTFT(String header, String body, uint16_t color) {
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(0, 5);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);
    tft.println(header);
    tft.drawFastHLine(0, 15, 128, ST77XX_WHITE);
    tft.drawFastHLine(0, 16, 160, ST77XX_WHITE);
    tft.setCursor(0, 80);
    tft.setTextColor(color);
    tft.setTextSize(2);
    tft.println(body);
    tft.drawFastHLine(0, 120, 160, ST77XX_WHITE);
    tft.drawFastHLine(0, 121, 160, ST77XX_WHITE);
}

void printLine(uint8_t size, int16_t x, int16_t y,
               uint16_t textcolor, uint16_t background,
               String label, uint8_t itemIdx, bool isMenu, int sel) {
    tft.setTextColor(textcolor, background);
    tft.setTextSize(size);
    tft.setCursor(x, y);
    if (isMenu) {
        tft.print(sel == itemIdx ? "> " : "  ");
    }
    tft.print(label);
}

// ═══════════════════════════════════════════════════════════════
//  Checksum verification for incoming SensorPacket_t
// ═══════════════════════════════════════════════════════════════
static bool _verify_checksum(const SensorPacket_t *pkt) {
    uint8_t cs = 0;
    const uint8_t *b = reinterpret_cast<const uint8_t *>(pkt);
    for (size_t i = 0; i < sizeof(SensorPacket_t) - 1; i++) cs ^= b[i];
    return cs == pkt->checksum;
}

// ═══════════════════════════════════════════════════════════════
//  ESP-NOW send  (coordinator → XIAO)
//  Sends a CommandPacket_t, not a SensorPacket_t.
// ═══════════════════════════════════════════════════════════════
static void _send_command(uint8_t cmd, uint16_t sample_count_req = 0) {
    CommandPacket_t pkt = {};
    pkt.command          = cmd;
    pkt.sample_count_req = sample_count_req;
    pkt.seq              = ++s_seq;
    esp_now_send(xiaoAddress, reinterpret_cast<uint8_t *>(&pkt), sizeof(pkt));
}

// ═══════════════════════════════════════════════════════════════
//  ESP-NOW callbacks
// ═══════════════════════════════════════════════════════════════
void OnDataSent(const uint8_t *mac, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        tft.fillScreen(ST77XX_BLACK);
        tft.setCursor(0, 40);
        tft.setTextColor(ST77XX_RED);
        tft.setTextSize(2);
        tft.println("TX FAILED");
        currentState = CHOICE;
    }
}

// ─────────────────────────────────────────────
//  OnDataRecv — decodes SensorPacket_t and updates
//  display state. Heavy display work is deferred
//  to loop() via s_newPacket flag so the ISR stays short.
// ─────────────────────────────────────────────
void OnDataRecv(const uint8_t *mac,
                const uint8_t *data, int len) {

    // Accept only properly-sized, checksum-valid packets
    if ((size_t)len != sizeof(SensorPacket_t)) return;

    const SensorPacket_t *pkt = reinterpret_cast<const SensorPacket_t *>(data);
    if (!_verify_checksum(pkt)) return;

    memcpy(&s_rxPacket, pkt, sizeof(SensorPacket_t));

    // ── FSR packets ─────────────────────────────────────────────
    if (pkt->sensor_id == CMD_READ_FSR && pkt->status == STATUS_OK) {
        // Each packet carries the cumulative sample list.
        // Read the latest entry (last slot written).
        if (pkt->sample_count > 0) {
            const FsrSample_t *samples =
                reinterpret_cast<const FsrSample_t *>(pkt->payload);
            uint16_t latest = pkt->sample_count - 1;
			
            s_fsrLastCount  = samples[latest].count;
            s_fsrLastAdc    = samples[latest].adc_value;
            s_fsrLastTime   = samples[latest].timestamp_ms;


        }
        // Live update — partial data display during the session
        currentState = RECEIVING;
        s_newPacket  = true;

        return;
    }

    // ── IMU packets ─────────────────────────────────────────────
    if (pkt->sensor_id == CMD_READ_LSM) {
        const ImuPacket_t *ip =
            reinterpret_cast<const ImuPacket_t *>(pkt->payload);

        switch (ip->subtype) {

            case IMU_PKT_COUNT:
                // Live count update — mirror of original stateID 203
                s_imuCycles  = ip->count.full_cycles;
                currentState = RECEIVING;
                s_newPacket  = true;
                break;

            case IMU_PKT_ROT_DATA:
                // Per-rotation summary — mirror of original stateID 204
                s_imuRotNum   = ip->rot_data.rotation_num;
                s_imuDuration = ip->rot_data.duration_sec;
                s_imuAngle    = ip->rot_data.angle_deg;
                Serial.printf("[ROT] #%lu  T:%.2fs  A:%.1f°\n",
                              (unsigned long)s_imuRotNum,
                              s_imuDuration, s_imuAngle);
                // Not triggering a full redraw for every rotation;
                // the COUNT update that follows will refresh the screen.
                break;

            case IMU_PKT_SAMPLE:
                // Raw gyro snapshot — mirror of original stateID 205
                Serial.printf("[SAMPLE] #%d/%d  %.1f dps\n",
                              ip->sample.sample_index,
                              ip->sample.total,
                              ip->sample.gyro_x_dps);
                // Logged to serial only, consistent with original behaviour.
                break;

            case IMU_PKT_DONE:
                // Session complete — mirror of original stateID 202
                s_imuCycles  = ip->done.total_cycles;
                currentState = RECEIVED;
                s_newPacket  = true;
                tft.fillScreen(ST77XX_BLACK);
                break;

            case IMU_PKT_ERROR:
                Serial.printf("[IMU ERROR] %s\n", ip->error.message);
                currentState = CHOICE;
                break;
        }
        return;
    }

    // ── Ping ACK / generic status ────────────────────────────────
    if (pkt->sensor_id == 0x00 && pkt->status == STATUS_OK) {
        Serial.println("[PING] XIAO alive");
    }
}

// ═══════════════════════════════════════════════════════════════
//  Menu screen  (encoder-driven selection)
// ═══════════════════════════════════════════════════════════════
void printMenu() {
    if (!isMenuDrawn) {
        tft.drawFastHLine(0,  0, 160, ST77XX_YELLOW);
        printLine(2, 0,  2, ST77XX_YELLOW, ST77XX_BLACK, "  Parkinson", 0, false, selection);
        printLine(2, 0, 22, ST77XX_YELLOW, ST77XX_BLACK, "  Test Menu",      0, false, selection);
        tft.drawFastHLine(0, 43, 160, ST77XX_WHITE);
        tft.drawFastHLine(0, 44, 160, ST77XX_WHITE);

        long count = encoder.getCount() / 2;
        selection = abs(count % 2);

        if (selection != lastSelection) {
            printLine(2, 20, 46, ST77XX_GREEN, ST77XX_BLACK, "Rotations", 0, true, selection);
            printLine(2, 20, 86, 0xFD20,       ST77XX_BLACK, "Taps", 1, true, selection);
            lastSelection = selection;
        }

        if (digitalRead(ENC_SW) == LOW) {
            tft.fillRect(0, 100, 160, 28, ST77XX_GREEN);
            printLine(2, 10, 105, ST77XX_WHITE, ST77XX_GREEN, "PICKED: ", 1, false, selection);

            if (selection == 0) {
                tft.print("Prob");
                s_pendingCmd = CMD_READ_LSM;   // maps to original stateID 101 / "PROB"
            } else {
                tft.print("Taps");
                s_pendingCmd = CMD_READ_FSR;   // maps to original stateID 102 / "TAP"
            }

            currentState = SENDING_CMD;
            delay(1000);
            tft.fillScreen(ST77XX_BLACK);
            isMenuDrawn = true;
        }
    } else {
        // Non-blocking waiting animation (unchanged from original)
        if (screenclear == 0) { tft.fillScreen(ST77XX_BLACK); screenclear = 1; }

        if (millis() - lastAnimTime >= (unsigned long)animInterval) {
            lastAnimTime = millis();
            dotCount++;
            if (dotCount > 3) dotCount = 0;

            tft.setCursor(0, 5);
            tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
            tft.setTextSize(2);
            String w = "Waiting";
            for (int i = 0; i < dotCount; i++) w += ".";
            while (w.length() < 10) w += " ";
            tft.println(w);
            tft.drawFastHLine(0, 34, 160, ST77XX_WHITE);
            tft.drawFastHLine(0, 35, 160, ST77XX_WHITE);
        }
    }
}

// ═══════════════════════════════════════════════════════════════
//  RECEIVING  — live data display during an active session
// ═══════════════════════════════════════════════════════════════
void printLiveData() {
    if (!s_newPacket) return;
    s_newPacket = false;

    if (s_rxPacket.sensor_id == CMD_READ_FSR) {

        Serial.printf("[TAP] #%u  T:%us  Adc:%u\n",
                              s_fsrLastCount,
                              s_fsrLastTime,
                              s_fsrLastAdc);
        
        // Show press count and latest ADC value
        tft.fillRect(0, 17, 160, 90, ST77XX_BLACK);
        tft.setCursor(0, 20);
        tft.setTextColor(ST77XX_CYAN);
        tft.setTextSize(2);
        tft.printf("Tap #%d", s_fsrLastCount);
        tft.setCursor(0, 50);
        tft.setTextColor(ST77XX_WHITE);
        tft.setTextSize(1);
        tft.printf("ADC: %d", s_fsrLastAdc);
        
        if (s_fsrLastCount == 10 ){
            currentState = RECEIVED;
            tft.fillScreen(ST77XX_BLACK);
        }

    } else if (s_rxPacket.sensor_id == CMD_READ_LSM) {
        // Show live rotation count — mirrors original stateID 203 display
        tft.fillRect(0, 17, 160, 35, ST77XX_BLACK);
        tft.setCursor(0, 20);
        tft.setTextColor(ST77XX_CYAN);
        tft.setTextSize(2);
        char buf[24];
        tft.setCursor(0,41);
        snprintf(buf, sizeof(buf), "COUNT:%lu", (unsigned long)s_imuCycles);
        tft.println(buf);
    }
}

// ═══════════════════════════════════════════════════════════════
//  RECEIVED  — session complete, final results display
// ═══════════════════════════════════════════════════════════════
void printResults() {
    isMenuDrawn    = false;
    screenclear    = 0;

    tft.setCursor(0, 5);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);
    tft.println("Test Complete");
    tft.drawFastHLine(0, 15, 128, ST77XX_WHITE);

    tft.setCursor(0, 80);
    tft.setTextColor(ST77XX_CYAN);
    tft.setTextSize(2);

    if (s_rxPacket.sensor_id == CMD_READ_LSM) {
        // Show final cycle count and last rotation summary
        char buf[32];
        snprintf(buf, sizeof(buf), "%lu cyc", (unsigned long)s_imuCycles);
        tft.println(buf);
        tft.setCursor(0, 105);
        tft.setTextSize(1);
        tft.setTextColor(ST77XX_WHITE);
        snprintf(buf, sizeof(buf), "R%lu %.2fs %.1fdeg",
                 (unsigned long)s_imuRotNum, s_imuDuration, s_imuAngle);
        tft.println(buf);
    } else {
        // Show final FSR tap count
        char buf[24];
        snprintf(buf, sizeof(buf), "%d taps", s_fsrLastCount);
        tft.println(buf);
    }

    tft.setCursor(0, 25);
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE);
    tft.println("Press button for menu");

    if (digitalRead(ENC_SW) == LOW) {
        tft.fillScreen(ST77XX_BLACK);
        while (digitalRead(ENC_SW) == LOW) {
            tft.setCursor(0, 25);
            tft.println("Release Button");
        }
        // Reset all session state
        lastSelection  = -1;
        isMenuDrawn    = false;
        screenclear    = 0;
        s_fsrLastCount = 0;
        s_fsrLastAdc   = 0;
        s_imuCycles    = 0;
        s_imuRotNum    = 0;
        encoder.setCount(0);
        delay(100);
        tft.fillScreen(ST77XX_BLACK);
        currentState = CHOICE;
    }
}

// ═══════════════════════════════════════════════════════════════
//  setup()
// ═══════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);

    tft.initR(INITR_BLACKTAB);
    tft.setRotation(1);
    updateTFT("PAWD", "Initializing", ST77XX_YELLOW);
    tft.setCursor(0, 25); tft.println("Parkinson");
    tft.setCursor(0, 40); tft.println("Wearable");
    delay(3000);
    tft.fillScreen(ST77XX_BLACK);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    // Lock to the shared channel defined in comms.h
    esp_wifi_set_channel(COMMS_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        tft.println("ESP-NOW FAIL");
        return;
    }
    esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

    // Register XIAO as the peer node
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, xiaoAddress, 6);
    peer.channel = COMMS_CHANNEL;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder.attachHalfQuad(ENC_A, ENC_B);
    encoder.setFilter(1023);
    encoder.setCount(0);
    pinMode(ENC_SW, INPUT_PULLUP);
}

// ═══════════════════════════════════════════════════════════════
//  loop()
// ═══════════════════════════════════════════════════════════════
void loop() {
    switch (currentState) {

        case CHOICE:
            printMenu();
            break;

        case SENDING_CMD:
            // Show what we're sending, then transmit and wait
            updateTFT("SENDING...",
                      s_pendingCmd == CMD_READ_LSM ? "PROB" : "TAPS",
                      ST77XX_MAGENTA);
            delay(1000);
            _send_command(s_pendingCmd);
            currentState = WAITING;
            break;

        case WAITING:
            // OnDataRecv drives us to RECEIVING or RECEIVED.
            // Nothing to do here; display is static.
            break;

        case RECEIVING:
            // Live data trickles in; update display on each new packet
            printLiveData();
            break;

        case RECEIVED:
            printResults();
            break;
    }
}

// ─────────────────────────────────────────────
//  Screen layout reference (unchanged)
//  160×128  Size1=10px/line  Size2=20px/line
// ─────────────────────────────────────────────