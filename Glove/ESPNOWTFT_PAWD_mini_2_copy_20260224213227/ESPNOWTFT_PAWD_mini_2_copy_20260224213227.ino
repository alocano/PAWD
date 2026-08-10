//160x128 screen
//Size 1 10pixels_offsetfory Max_12lines 6 pixels_offsetforx
//Size 2 20pixels_offsetfory Max_8lines  12 pixels_offsetforx
//Size 3 28pixels_offsetfory Max_5lines  18 pixels_offsetforx
//Size 4 36pixels_offsetfory Max_4lines  24 pixels_offsetforx
//#define ST77XX_BLACK 0x0000
//#define ST77XX_WHITE 0xFFFF
//#define ST77XX_RED 0xF800
//#define ST77XX_GREEN 0x07E0
//#define ST77XX_BLUE 0x001F
//#define ST77XX_CYAN 0x07FF
//#define ST77XX_MAGENTA 0xF81F
//#define ST77XX_YELLOW 0xFFE0
//#define ST77XX_ORANGE 0xFC00

#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>    
#include <Adafruit_ST7735.h> 
#include <SPI.h>

// Pins for the Mini Board (Modify if your specific board uses different ones)
#define TFT_CS     15
#define TFT_RST    4 
#define TFT_DC     27

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// 1. mac address wroom
uint8_t broadcastAddress[] = {0x68, 0xFE, 0x71, 0xF9, 0xAD, 0x40}; 

typedef struct struct_message {
    int stateID;
    char payload[32];
} struct_message;

struct_message myData; // Buffer to send data
struct_message incomingData; // Buffer to recieve data found in updatetft

enum State { IDLE, SENDING, WAITING, RECEIVED, TAP, PROB };
State currentState = IDLE;

// Helper to update the 128x160 TFT
void updateTFT(String localStatus, uint16_t color) {
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 5);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.println("PAWD: MINI UNIT");
  tft.drawFastHLine(0, 15, 128, ST77XX_WHITE);
  
  tft.setCursor(0, 40);
  tft.setTextColor(color);
  tft.setTextSize(2);
  tft.println(localStatus);
  
  tft.setCursor(0, 90);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.println("LAST WROOM CMD:");
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(0, 105);
  tft.println(incomingData.payload); // print incoming data from wroom
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // After sending data back to WROOM, go back to listening
  currentState = IDLE;
}

// Callback when data is received from WROOM
void OnDataRecv(const esp_now_recv_info_t * recv_info, const uint8_t *incomingDataRaw, int len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));
  
  String cmd = String(incomingData.payload);
  cmd.trim();

  // Switch state based on command string
  if (cmd == "TAP") {
    currentState = TAP;
  } 
  else if (cmd == "PROB") {
    currentState = PROB;
  } 
  else {
    currentState = RECEIVED; 
  }
}

void setup() {
  Serial.begin(115200);
  
  // Use Blacktab for standard ST7735
  tft.initR(INITR_BLACKTAB); 
  tft.setRotation(1); // 0 is vertical, 1 landscape, 2 is upside down vertical, 3 upside down landscape
  
  updateTFT("SYSTEM READY", ST77XX_YELLOW);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return;

  // Register Callbacks
  esp_now_register_send_cb((esp_now_send_cb_t)OnDataSent);
  esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);
  
  // Peer Registration
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void loop() {
  switch (currentState) {
    case IDLE:
      // Screen remains static showing last received data
      break;

    case RECEIVED:
      updateTFT("CMD RECV", ST77XX_CYAN);
      delay(1000);
      currentState = IDLE;
      break;

    case TAP:
      updateTFT("TAP ACTIVE", ST77XX_ORANGE);
      
      // Sensor
      delay(3000);
      
      strcpy(myData.payload, "TAP_COMPLETE: 12"); 
      currentState = SENDING;
      break;

    case PROB:
      updateTFT("PROB ACTIVE", ST77XX_BLUE);
      
      // Sensor
      delay(3000); 
      
      strcpy(myData.payload, "PROB_RESULT: 0.85");
      currentState = SENDING;
      break;

    case SENDING:
      myData.stateID = 202;
      esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
      updateTFT("UPLOADING", ST77XX_MAGENTA);
      currentState = WAITING; // Wait for OnDataSent callback
      break;

    case WAITING:
      // Loop does nothing while radio is busy
      break;
  }
}

