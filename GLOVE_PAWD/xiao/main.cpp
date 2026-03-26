// PAWD MINI — Main Sketch
// Board: ESP32 Dev Module (Super Mini)
// Pair with: WROOM_FIXED.ino on the dongle board
//
// Tabs required in this sketch folder:
//   prob_logic.h  ← IMU + rotation counting logic

#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include "prob_logic.h"

// MAC Address of WROOM dongle
uint8_t broadcastAddress[] = {0x68, 0xFE, 0x71, 0xF9, 0xAD, 0x40};

typedef struct struct_message {
    int  stateID;
    char payload[32];
} struct_message;

struct_message myData;
struct_message incomingData;

enum State { IDLE, SENDING, WAITING, RECEIVED, TAP, PROB };
State currentState = IDLE;

// stateID reference
// 101 = WROOM → MINI: run PROB
// 102 = WROOM → MINI: run TAP
// 202 = MINI → WROOM: final result        "PROB_DONE:10"
// 203 = MINI → WROOM: live count update   "COUNT:N"
// 204 = MINI → WROOM: rotation summary    "R:N T:X.XXs A:XXXX.X"
// 205 = MINI → WROOM: one gyro X sample   "S:N V:XXX.X"

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (currentState == WAITING) currentState = IDLE;
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingDataRaw, int len) {
    if (len != sizeof(incomingData)) return;
    memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));
    String cmd = String(incomingData.payload);
    cmd.trim();
    if      (cmd == "TAP")  currentState = TAP;
    else if (cmd == "PROB") currentState = PROB;
    else                    currentState = RECEIVED;
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) return;
    esp_now_register_send_cb((esp_now_send_cb_t)OnDataSent);
    esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
}

// Send all queued gyro samples for a completed rotation.
// Sends stateID 205 packets, one per sample, with a 10ms gap each.
void sendRotSamples() {
    for (uint8_t i = 0; i < rotSampleCount; i++) {
        myData.stateID = 205;
        snprintf(myData.payload, sizeof(myData.payload),
                 "S:%d V:%.1f", i + 1, rotSamples[i]);
        esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
        delay(10);
    }
    resetRotSamples(); // clear buffer for next rotation
}

void loop() {
    switch (currentState) {

        case IDLE:
            break;

        case RECEIVED:
            delay(1000);
            currentState = IDLE;
            break;

        // TAP: logic goes here when ready
        case TAP:
            delay(3000);
            strcpy(myData.payload, "TAP_COMPLETE: 12");
            currentState = SENDING;
            break;

        // PROB: rotation counting + per-rotation gyro sample collection
        case PROB:
            if (!probRunning) {
                if (!initIMUIfNeeded()) {
                    strcpy(myData.payload, "PROB_ERR: IMU");
                    currentState = SENDING;
                    break;
                }
                resetPROB();
            }

            {
                uint32_t fullCycles = 0;
                char rotData[32]    = "";
                int result = runPROBSample(fullCycles, rotData);

                if (result == 3) {
                    // Full rotation complete — send in this order:

                    // 1. Raw gyro samples (stateID 205 × N)
                    sendRotSamples();

                    // 2. Rotation summary (stateID 204)
                    myData.stateID = 204;
                    strncpy(myData.payload, rotData, sizeof(myData.payload));
                    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
                    delay(10);

                    // 3. Live count update (stateID 203)
                    myData.stateID = 203;
                    snprintf(myData.payload, sizeof(myData.payload),
                             "COUNT:%lu", (unsigned long)fullCycles);
                    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
                }
                else if (result == 1) {
                    myData.stateID = 203;
                    snprintf(myData.payload, sizeof(myData.payload),
                             "COUNT:%lu", (unsigned long)fullCycles);
                    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
                }
                else if (result == 2) {
                    myData.stateID = 202;
                    snprintf(myData.payload, sizeof(myData.payload),
                             "PROB_DONE:%lu", (unsigned long)fullCycles);
                    currentState = SENDING;
                }
            }
            break;

        case SENDING:
            myData.stateID = 202;
            esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
            currentState = WAITING;
            break;

        case WAITING:
            break;
    }
}