//160x128 screen
//Size 1 10pixels_offsetfory Max_12lines 6 pixels_offsetforx
//Size 2 20pixels_offsetfory Max_8lines  12 pixels_offsetforx
//Size 3 28pixels_offsetfory Max_5lines  18 pixels_offsetforx
//Size 4 36pixels_offsetfory Max_4lines  24 pixels_offsetforx


#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>    
#include <Adafruit_ST7789.h> 
#include <SPI.h>


// Verified Pinout for NHD-2.4-240320CF & ESP32-S3
#define TFT_CS     14
#define TFT_RST    17
#define TFT_DC     15
#define TFT_MOSI   11  
#define TFT_SCLK   12  

// Button Definitions
#define ENC_SW   7
#define UP_BTN   4
#define DOWN_BTN 5

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

int selection = 0; 
int lastSelection = -1; 
String choice= "Picked: Pronation!";
bool isMenuDrawn = false;
int screenclear = 0;

unsigned long lastAnimTime = 0; //none blocking wait not using delay()
int dotCount = 0;
const int animInterval = 500; // Speed of dots in milliseconds

//uint8_t broadcastAddress[] = {0xE0, 0x72, 0xA1, 0x6B, 0x2F, 0xF4}; //mini

uint8_t broadcastAddress[] = {0x20, 0xE7, 0xC8, 0xAD, 0xC0, 0x8C}; //wroom mac address confirmed



typedef struct struct_message {
    int stateID;
    char payload[32];
} struct_message;

struct_message myData;  
struct_message incomingData; 

enum State { CHOICE, SENDING_CMD, WAITING, RECEIVED };
State currentState = CHOICE;

// --- Helper Functions ---

void updateTFT(String header, String body, uint16_t color) {
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 5);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(3);
  tft.println(header);
  tft.drawFastHLine(0, 34, 320, ST77XX_WHITE); 
  tft.drawFastHLine(0, 35, 320, ST77XX_WHITE); 

  tft.drawFastVLine(2, 0, 35, ST77XX_YELLOW);
  tft.drawFastVLine(3, 0, 35, ST77XX_YELLOW);
  tft.drawFastVLine(318, 0, 35, ST77XX_YELLOW);
  tft.drawFastVLine(317, 0, 35, ST77XX_YELLOW);
  
  tft.setCursor(0, 180);
  tft.setTextColor(color,ST77XX_BLACK);
  tft.setTextSize(4);
  tft.println(body);
  tft.drawFastHLine(0, 216, 320, ST77XX_WHITE); 
  tft.drawFastHLine(0, 217, 320, ST77XX_WHITE); 
}

void printLine(uint8_t size, int16_t x, int16_t y, uint16_t textcolor, uint16_t background, String label, uint8_t itemIndex, bool menu, int selection) {
    tft.setTextColor(textcolor, background);
    tft.setTextSize(size);
    tft.setCursor(x, y);
    if(menu == true){
      if (selection == itemIndex) tft.print("> ");
      else tft.print("  ");
      tft.print(label);
    } else {
      tft.print(label);
    }
}

// --- ESP-NOW Callbacks ---

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("Delivery Failed");
    currentState = CHOICE; 
  }
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingDataRaw, int len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));
  tft.fillScreen(ST77XX_BLACK);
  currentState = RECEIVED; 
}

void setup() {
  Serial.begin(115200);

  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS); 

  tft.init(240, 320); 
  tft.setRotation(3);

  tft.invertDisplay(true);// colors were opposite so this command needs to be true
  updateTFT("       PAWD"," Initializing" ,ST77XX_YELLOW); 
  
  delay(2000);
  tft.fillScreen(ST77XX_BLACK);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return;

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  // Set pins to Input Pullup
  pinMode(ENC_SW, INPUT_PULLUP); 
  pinMode(UP_BTN, INPUT_PULLUP);
  pinMode(DOWN_BTN, INPUT_PULLUP);
}

void loop() {
  switch (currentState) {
    case CHOICE:{
      if(!isMenuDrawn){
        tft.drawFastHLine(0, 0, 320, ST77XX_YELLOW);
        printLine(3, 0, 2, ST77XX_YELLOW, ST77XX_BLACK, "  Parkison Test ", 0, false, selection);
        printLine(3, 0, 31, ST77XX_YELLOW, ST77XX_BLACK, "      Menu", 0, false, selection);
        tft.drawFastHLine(0, 60, 320, ST77XX_WHITE);
        tft.drawFastHLine(0, 61, 320, ST77XX_WHITE);
        
      // Check UP Button - Moves selection UP (towards 0)
        if (digitalRead(UP_BTN) == LOW) {
          if (selection > 0) {
            choice = "Picked: Pronation!";
            selection--; 
          }
          delay(200); // Debounce to prevent multi-trigger
        }

      // Check DOWN Button - Moves selection DOWN (towards 1)
        if (digitalRead(DOWN_BTN) == LOW) {
          if (selection < 1) {
            selection++; 
            choice = "Picked: Taps!";
          }
          delay(200); // Debounce to prevent multi-trigger
        }

      // Only Redraw if selection actually changed
        if (selection != lastSelection) {
          // We redraw BOTH lines so the old ">" disappears and the new one appears
          printLine(3, 30, 65, 0x07FF/*ST77XX_RED*/, ST77XX_BLACK, "1. Pronation", 0, true, selection); //IF you do red gives cyan instead so find color code of cyan to get red
          printLine(3, 30, 150, ST77XX_GREEN, ST77XX_BLACK, "2. Taps", 1, true, selection); //green give purple
          lastSelection = selection;
        }
      
      // Confirm Selection with Center Switch
        if (digitalRead(ENC_SW) == LOW) {
          tft.fillRect(0, 200, 320, 28, ST77XX_GREEN);
          printLine(2, 10, 205, ST77XX_WHITE, ST77XX_GREEN, choice, 1, false, selection);

          if(selection == 0) strcpy(myData.payload, "PRONATION");
          else strcpy(myData.payload, "TAP");

          currentState = SENDING_CMD;
          delay(1000);
          tft.fillScreen(ST77XX_BLACK);
          isMenuDrawn = true;
        }

    }
    else {
        // --- NON-BLOCKING WAITING ANIMATION ---
        if(screenclear == 0){
          tft.fillScreen(ST77XX_BLACK);
          screenclear =1;

        }
        if (millis() - lastAnimTime >= animInterval) {
          lastAnimTime = millis();
          dotCount++;
          if (dotCount > 3) dotCount = 0;

          // Clear the small area for dots or redraw the text
          tft.setCursor(0, 5);
          tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
          tft.setTextSize(3);
          
          String waitText = "Waiting";
          for(int i=0; i < dotCount; i++) waitText += ".";
          
          // Print spaces at the end to "erase" old dots
          while(waitText.length() < 10) waitText += " "; 
          
          tft.println(waitText);
          
          tft.drawFastHLine(0, 34, 320, ST77XX_WHITE);
          tft.drawFastHLine(0, 35, 320, ST77XX_WHITE);
        }
      }
      break;
    }
    case SENDING_CMD:{
      updateTFT("UPLOADING...", myData.payload, ST77XX_MAGENTA);
      delay(1000);
      esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
      currentState = WAITING;
      break;
    }

    case WAITING: break;

    case RECEIVED:{
       isMenuDrawn = false;
       screenclear =0;
       tft.setCursor(0, 5);
       tft.setTextColor(ST77XX_RED);
       tft.setTextSize(3);
       tft.println("  Test Complete");
       tft.drawFastHLine(0, 34, 320, ST77XX_WHITE); 
  
       tft.setCursor(0, 200);
       tft.setTextColor(ST77XX_CYAN);
       tft.setTextSize(3);
       tft.println(String(incomingData.payload));
      
       tft.setCursor(0, 80);
       tft.setTextSize(3);
       tft.setTextColor(ST77XX_GREEN);
       tft.println("   Press Button");
       tft.println("   to return menu");

       if (digitalRead(ENC_SW) == LOW) { 
          tft.fillScreen(ST77XX_BLACK);
          while(digitalRead(ENC_SW) == LOW){ 
            tft.setCursor(0, 25);
            tft.println("Release Button");
          }
          lastSelection = -1;
          delay(100);
          tft.fillScreen(ST77XX_BLACK);
          currentState = CHOICE;
       }
       break;
    }
  }
}