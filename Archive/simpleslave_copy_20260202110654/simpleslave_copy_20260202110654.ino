#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h> 
#include <vector>

// --- OLED Libraries ---
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* Definitions */
#define ESPNOW_WIFI_CHANNEL 6
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1    
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- Minimal Shared Data Structure ---
typedef struct struct_message {
    int value; // The simplest message: a single integer
} struct_message;

volatile struct_message incomingData;
volatile bool dataReceived = false;

/* Classes for Dynamic Peer Registration */
class ESP_NOW_Peer_Class : public ESP_NOW_Peer {
public:
  ESP_NOW_Peer_Class(const uint8_t *mac_addr, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}
  bool add_peer() { return add(); }

  // MODIFIED onReceive to handle the minimal struct
  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    if (len == sizeof(struct_message)) {
      memcpy((void*)&incomingData, data, len);
      dataReceived = true; // Set the flag
      Serial.printf("Received Value: %d\n", incomingData.value);
    } 
  }
};

/* Global Variables and Callbacks (Same as before) */
std::vector<ESP_NOW_Peer_Class *> masters;

void register_new_master(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
  if (memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, 6) == 0) {
    ESP_NOW_Peer_Class *new_master = new ESP_NOW_Peer_Class(info->src_addr, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, nullptr);
    if (new_master->add_peer()) {
      masters.push_back(new_master);
      Serial.printf("Master " MACSTR " registered.\n", MAC2STR(new_master->addr()));
    } else {
      delete new_master;
    }
  }
}

// Function to draw the content to the OLED
void updateOLED() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);

    display.println("RECEIVED VALUE:");
    
    display.setTextSize(4); 
    display.setCursor(0, 15);
    display.println(incomingData.value);

    display.display();
}

/* Main */
void setup() {
  Serial.begin(115200);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) { while(true); } // Simplified OLED init
  display.clearDisplay(); display.display();

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) { delay(100); }
  
  if (!ESP_NOW.begin()) { ESP.restart(); }

  ESP_NOW.onNewPeer(register_new_master, nullptr);
  Serial.printf("MAC: %s | Channel: %d\n", WiFi.macAddress().c_str(), ESPNOW_WIFI_CHANNEL);
}

void loop() {
  if (dataReceived) {
      updateOLED();
      dataReceived = false;
  }
  delay(10);
}