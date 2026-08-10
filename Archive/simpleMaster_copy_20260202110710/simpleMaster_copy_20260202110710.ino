#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h> 

/* Definitions */
#define ESPNOW_WIFI_CHANNEL 6

// --- Minimal Shared Data Structure ---
typedef struct struct_message {
    int value; // The simplest message: a single integer
} struct_message;

struct_message outgoingData;

// Timer variables for periodic sending
unsigned long lastTime = 0;  
unsigned long timerDelay = 2000; // Send data every 2 seconds

/* Classes (Same as before) */
class ESP_NOW_Broadcast_Peer : public ESP_NOW_Peer {
public:
  ESP_NOW_Broadcast_Peer(uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(ESP_NOW.BROADCAST_ADDR, channel, iface, lmk) {}
  ~ESP_NOW_Broadcast_Peer() { remove(); }
  bool begin() {
    if (!ESP_NOW.begin() || !add()) { return false; }
    return true;
  }
  bool send_message(const uint8_t *data, size_t len) {
    if (!send(data, len)) { return false; }
    return true;
  }
};

/* Global Variables */
ESP_NOW_Broadcast_Peer broadcast_peer(ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, nullptr);

/* Main */
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) { delay(100); }

  if (!broadcast_peer.begin()) { ESP.restart(); }

  outgoingData.value = 0;
  Serial.println("Minimal Master setup complete. Broadcasting integer every 2s.");
}

void loop() {
  if ((millis() - lastTime) > timerDelay) {
    
    // 1. Prepare data (the simplest message: incrementing an integer)
    outgoingData.value++;
    
    // 2. Broadcast the struct data
    Serial.printf("Broadcasting Value: %d\n", outgoingData.value);
    
    // Send the binary data (the single integer)
    if (!broadcast_peer.send_message((uint8_t *)&outgoingData, sizeof(outgoingData))) {
      Serial.println("Failed to broadcast message");
    }

    lastTime = millis();
  }
}