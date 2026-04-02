#include "espnow_handler.h"
#include <string.h>

static uint8_t                  s_coordinator_mac[6] ;
static espnow_cmd_callback_t    s_cmd_callback   = nullptr;
static espnow_send_callback_t   s_send_callback  = nullptr;

static volatile bool                  s_tx_done   = false;
static volatile esp_now_send_status_t s_tx_status = ESP_NOW_SEND_FAIL;
static uint32_t s_last_seq = 0xFFFFFFFFu;

// ─────────────────────────────────────────────
//  Checksum
// ─────────────────────────────────────────────
static uint8_t _xor_checksum(const uint8_t *data, size_t len) {
    uint8_t cs = 0;
    for (size_t i = 0; i < len; i++) cs ^= data[i];
    return cs;
}

// ─────────────────────────────────────────────
//  Internal ESP-NOW callbacks  (WiFi task / IRAM)
//
//  FIX: Core 3.x changed the callback signatures:
//    recv → first param is  const esp_now_recv_info_t*  (MAC is in ->src_addr)
//    send → first param is  const wifi_tx_info_t*       (not a raw MAC pointer)
// ─────────────────────────────────────────────

// OLD:  void _on_data_recv(const uint8_t *mac, const uint8_t *data, int len)
// NEW:  void _on_data_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
static void IRAM_ATTR _on_data_recv(const esp_now_recv_info_t *info,
                                    const uint8_t *data, int len) {
    // The MAC address moved from a direct parameter into info->src_addr
    if (memcmp(info->src_addr, s_coordinator_mac, 6) != 0) return;
    if ((size_t)len < sizeof(CommandPacket_t)) return;

    const CommandPacket_t *cmd = reinterpret_cast<const CommandPacket_t *>(data);
    if (cmd->seq == s_last_seq) return;   // deduplicate
    s_last_seq = cmd->seq;

    if (s_cmd_callback) s_cmd_callback(cmd);
}

// OLD:  void _on_data_sent(const uint8_t *mac, esp_now_send_status_t status)
// NEW:  void _on_data_sent(const wifi_tx_info_t *info, esp_now_send_status_t status)
static void IRAM_ATTR _on_data_sent(const wifi_tx_info_t *info,
                                    esp_now_send_status_t status) {
    // We don't need the MAC here — just capture the status
    s_tx_status = status;
    s_tx_done   = true;
    if (s_send_callback) s_send_callback(status == ESP_NOW_SEND_SUCCESS);
}

// ─────────────────────────────────────────────
//  Init
// ─────────────────────────────────────────────
bool espnow_init(const uint8_t coordinator_mac[6],
                 espnow_cmd_callback_t  on_command,
                 espnow_send_callback_t on_send_done) {

    memcpy(s_coordinator_mac, coordinator_mac, 6);
    s_cmd_callback  = on_command;
    s_send_callback = on_send_done;

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(COMMS_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) return false;

    // No cast needed now — signatures match the Core 3.x typedefs exactly
    esp_now_register_recv_cb(_on_data_recv);
    esp_now_register_send_cb(_on_data_sent);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, coordinator_mac, 6);
    peer.channel = COMMS_CHANNEL;
    peer.encrypt = false;

    if (esp_now_add_peer(&peer) != ESP_OK) {
        esp_now_deinit();
        return false;
    }
    return true;
}

// ─────────────────────────────────────────────
//  Send  (blocking with retry)
// ─────────────────────────────────────────────
bool espnow_send(SensorPacket_t *packet) {
    packet->checksum = _xor_checksum(
        reinterpret_cast<const uint8_t *>(packet),
        sizeof(SensorPacket_t) - 1);

    for (int attempt = 0; attempt < ESPNOW_RETRY_LIMIT; attempt++) {
        s_tx_done = false;

        esp_err_t err = esp_now_send(
            s_coordinator_mac,
            reinterpret_cast<const uint8_t *>(packet),
            sizeof(SensorPacket_t));

        if (err != ESP_OK) { delay(10); continue; }

        uint32_t deadline = millis() + ESPNOW_TX_TIMEOUT_MS;
        while (!s_tx_done && millis() < deadline) delay(1);

        if (s_tx_done && s_tx_status == ESP_NOW_SEND_SUCCESS) return true;
        delay(20 * (attempt + 1));
    }
    return false;
}

bool espnow_send_ack(uint8_t status_code) {
    SensorPacket_t pkt = {};
    pkt.sensor_id    = 0x00;
    pkt.status       = status_code;
    pkt.timestamp_ms = millis();
    pkt.sample_count = 0;
    return espnow_send(&pkt);
}

void espnow_deinit(void) {
    esp_now_del_peer(s_coordinator_mac);
    esp_now_deinit();
    WiFi.mode(WIFI_OFF);
}

esp_now_send_status_t espnow_last_send_status(void) { return s_tx_status; }