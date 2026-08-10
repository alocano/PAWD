//wroom board
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>    
#include <Adafruit_ST7735.h> 
#include <SPI.h>
#include <ESP32Encoder.h>

#define TFT_CS     15
#define TFT_RST    4 
#define TFT_DC     27
//sda23
//scl18
#define ENC_A    32
#define ENC_B    33
#define ENC_SW   25
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
ESP32Encoder encoder;
int selection = 0; //current postion  encoder dial cur
int lastSelection = -1; //last postion encoder dial prev
bool menu = true; //for arrow on menu
uint8_t itemIndex;  // index number of option

// Mac Address ("mini")
uint8_t broadcastAddress[] = {0x20, 0xE7, 0xC8, 0xAD, 0xC0, 0x8C};
// Mac address wroom board
//uint8_t broadcastAddress[] = {0x68, 0xFE, 0x71, 0xF9, 0xAD, 0x40}; //correct

typedef struct struct_message {
    int stateID;
    char payload[32];
} struct_message;

struct_message myData;  //Wroom buffer sending to mini
struct_message incomingData; //wroom buffer recieving from mini

enum State { CHOICE, SENDING_CMD, WAITING, RECEIVED };
State currentState = CHOICE;

// Helper to update the Master TFT
void updateTFT(String header, String body, uint16_t color) {
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 5);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.println(header);
  tft.drawFastHLine(0, 15, 128, ST77XX_WHITE);
  
  tft.setCursor(0, 80);
  tft.setTextColor(color);
  tft.setTextSize(2);
  tft.println(body);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) { //interrupts for send and recieve
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("Delivery Failed");
    currentState = CHOICE; 
  }
}

void OnDataRecv(const esp_now_recv_info_t * recv_info, const uint8_t *incomingDataRaw, int len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));
  tft.fillScreen(ST77XX_BLACK);
  currentState = RECEIVED; 
}

void setup() {
  Serial.begin(115200);
  tft.initR(INITR_BLACKTAB); 
  tft.setRotation(3);// 0 is vertical 1landscape 2 is upside down vertical 3ypside down land
  updateTFT("PAWD","Initializing" ,ST77XX_YELLOW); //reuires top line text,bottom text,color
  tft.setCursor(0, 25); //Line below
  tft.println("Parkison "); //prints where the cursor is
    tft.setCursor(0, 40); //Line below  x and y axis
  tft.println("Wearable");
  tft.setCursor(0, 60);

  delay(3000);
  tft.fillScreen(ST77XX_BLACK);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return;

  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  
  encoder.attachHalfQuad(ENC_A, ENC_B);
  encoder.setFilter(1023); // Helps with "bouncy" switches
  encoder.setCount(0);

  pinMode(ENC_SW, INPUT_PULLUP); //button


}

void loop() {
  static unsigned long lastSend = 0;
  switch (currentState) {
    case CHOICE:{
      
      printLine(2,0, 0, ST77XX_WHITE,ST77XX_BLACK,"Trans> ", 0, false,selection );
      
        long count = encoder.getCount() / 2; //counts the ticks and resets each time it stops if odd postion 1 if even postion 2
        selection = abs(count % 2); 

       if (selection != lastSelection) {
         printLine(2,20, 30, ST77XX_WHITE,ST77XX_BLACK,"Prob", 0, true,selection );
         printLine(2,20, 70, ST77XX_WHITE,ST77XX_BLACK,"Taps", 1, true,selection );
         lastSelection = selection;
       }
       if (digitalRead(ENC_SW) == LOW) {
         tft.fillRect(0, 100, 160, 28, ST77XX_GREEN);

         printLine(2,10, 105, ST77XX_WHITE,ST77XX_GREEN,"PICKED: ", 1, false,selection ); //1 is ignored due to false menu

         if(selection == 0)
         {
            tft.print("Prob");
			myData.stateID = 101;
            strcpy(myData.payload, "PROB");
            currentState =  SENDING_CMD;;
            delay(1000);
            tft.fillScreen(ST77XX_BLACK);
            break;
         }
         else
         {
           tft.print("Taps");
		   myData.stateID = 102;
           strcpy(myData.payload, "TAP");
           currentState =  SENDING_CMD;
           delay(1000);
           tft.fillScreen(ST77XX_BLACK);
           break;
         }
        delay(1000); 
        lastSelection = -1; 
      }

      
      break;
    }
    case SENDING_CMD:{
      updateTFT("UPLOADING...", myData.payload, ST77XX_MAGENTA);
      delay(2000);
      esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
      currentState = WAITING;
      break;
    }

    case WAITING:{
      // Static display to avoid flicker while waiting for Mini
      // The OnDataRecv callback will move us to the next state
      break;
    }

    case RECEIVED:{
      // 1. Print and Hold the Data on Screen
      //updateTFT("TEST COMPLETE", String(incomingData.payload), ST77XX_CYAN);
       tft.setCursor(0, 5);
       tft.setTextColor(ST77XX_WHITE);
       tft.setTextSize(1);
       tft.println("Test Complete");
       tft.drawFastHLine(0, 15, 128, ST77XX_WHITE);
  
       tft.setCursor(0, 80);
       tft.setTextColor(ST77XX_CYAN);
       tft.setTextSize(2);
       tft.println(String(incomingData.payload));
       //incoming data 
      
     
      tft.setCursor(0, 25);
      tft.setTextSize(1);
      tft.setTextColor(ST77XX_WHITE);
      tft.println("Press Button to return menu");

      
     if (digitalRead(ENC_SW) == LOW) { //if button is pressed start the return menu process
      tft.fillScreen(ST77XX_BLACK);
          //Serial.println("Returning to Main Menu...");
          while(digitalRead(ENC_SW) == LOW){ // if button is held down give propmt
            tft.setCursor(0, 25);
            tft.println("Release Button");

          }
          lastSelection = -1;
          encoder.setCount(0); //resets dial variables
          delay(100);
          tft.fillScreen(ST77XX_BLACK);
          currentState = CHOICE;
          
          
        
      }

      break;
    }
  }
}

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

void printLine(uint8_t size, int16_t x, int16_t y, uint16_t textcolor,uint16_t background,String label, uint8_t itemIndex, bool menu,int selection ) 
{
    tft.setTextColor(textcolor, background);
    tft.setTextSize(size);

    tft.setCursor(x, y);
    if(menu == true){
      if (selection == itemIndex) {
       tft.print("> ");
       } 
      else {
          tft.print("  "); // Two spaces to overwrite the old ">"

       }
       tft.print(label);
    }
    else{
      tft.print(label);
    }
}