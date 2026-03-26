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

#include <Wire.h>
#include <Adafruit_LSM6DS3TRC.h>

// Pins for the Mini Board (Modify if your specific board uses different ones)
//#define TFT_CS     15
//define TFT_RST    4 
//#define TFT_DC     27

//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

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

//imu

// Super Mini I2C pins 
static const int SDA_PIN = 8;
static const int SCL_PIN = 9;

Adafruit_LSM6DS3TRC imu;
static bool imuReady = false;


static const float alpha   = 0.01f;  // bias tracking rate
static const float restThr = 14.0f;  // dps: near-still threshold for bias update

static const float deadband = 25.0f; // dps: hysteresis around zero
static const float peakThr  = 50.0f; // dps: must hit peak before counting crossing
static const float minGap   = 0.25f; // seconds: minimum time between crossings

// State
float bias_dps = 0.0f;

bool hadPeakPos = false;
bool hadPeakNeg = false;

int lastSign = 0;               // -1, 0, +1
float lastCrossTime = -1e9f;

uint32_t halfCycles = 0;

unsigned long t0_ms = 0;

static inline float rad2deg(float r) { return r * 57.2957795f; }

int signWithDeadband(float x_dps) {
  if (x_dps > deadband) return +1;
  if (x_dps < -deadband) return -1;
  return 0;
}

static bool initIMUIfNeeded() {
  if (imuReady) return true;

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println("Trying IMU on I2C...");

  if (imu.begin_I2C(0x6A)) {
    Serial.println("IMU found at 0x6A");
    imuReady = true;
    return true;
  }

  if (imu.begin_I2C(0x6B)) {
    Serial.println("IMU found at 0x6B");
    imuReady = true;
    return true;
  }

  Serial.println("ERROR: IMU not found at 0x6A or 0x6B");
  imuReady = false;
  return false;
}
static bool probRunning = false;
static unsigned long probNextSampleMs = 0;
static const uint32_t PROB_TARGET_CYCLES = 10;
static const uint32_t PROB_SAMPLE_MS = 25; // sample period
static uint32_t lastDisplayedCycles = 999;


// Helper to update the 128x160 TFT
/*void updateTFT(String header, String body, uint16_t color) {
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 5);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.println(header);
  tft.drawFastHLine(0, 15, 128, ST77XX_WHITE);
  
  tft.setCursor(0, 40);
  tft.setTextColor(color);
  tft.setTextSize(2);
  tft.println(body);
  
  tft.setCursor(0, 90);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.println("LAST WROOM CMD:");

  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(0, 105);
  tft.println(incomingData.payload); // print incoming data from wroom
}
*/
// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // After sending data back to WROOM, go back to listening
  if (currentState == WAITING) {
    currentState = IDLE;
  }
}
// Callback when data is received from WROOM
void OnDataRecv(const esp_now_recv_info_t * recv_info, const uint8_t *incomingDataRaw, int len) {
  if (len != sizeof(incomingData)) return;  //new code testing
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData)) ;
  
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
  //tft.initR(INITR_BLACKTAB); 
  //tft.setRotation(1); // 0 is vertical, 1 landscape, 2 is upside down vertical, 3 upside down landscape
  
  //updateTFT("SYSTEM READY", ST77XX_YELLOW);

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
     // updateTFT("CMD RECV", ST77XX_CYAN);
      delay(1000);
      currentState = IDLE;
      break;

    case TAP:
      //updateTFT("TAP ACTIVE", ST77XX_ORANGE);
      
      // Sensor
      delay(3000);
      
      strcpy(myData.payload, "TAP_COMPLETE: 12"); 
      currentState = SENDING;
      break;

    case PROB:
      

      if (!probRunning) {
        //updateTFT("PROB ACTIVE", "0", ST77XX_BLUE);

        if (!initIMUIfNeeded()) {
          strcpy(myData.payload, "PROB_ERR: IMU");
          currentState = SENDING;
          break;
        }
        bias_dps = 0.0f;
        hadPeakPos = false;
        hadPeakNeg = false;
        lastSign = 0;
        lastCrossTime = -1e9f;
        halfCycles = 0;
        t0_ms = millis();
        lastDisplayedCycles = 999;
       
       // updateTFT("PROB ACTIVE", "0", ST77XX_BLUE);

        probRunning = true;
        probNextSampleMs = millis(); // start immediately
      }

      // Sample at fixed rate (25ms)
      if (millis() >= probNextSampleMs) {
        probNextSampleMs += PROB_SAMPLE_MS;

        float tNow = (millis() - t0_ms) * 0.001f;

        // Read gyro X (deg/s)
        sensors_event_t a, g, temp;
        imu.getEvent(&a, &g, &temp);
        float gx_dps = rad2deg(g.gyro.x); // ONLY X axis

        // Auto-zero bias when near still
        if (fabsf(gx_dps - bias_dps) < restThr) {
          bias_dps = (1.0f - alpha) * bias_dps + alpha * gx_dps;
        }
        float gxc_dps = gx_dps - bias_dps;

        // Track whether each side reached a peak
        if (gxc_dps >=  peakThr) hadPeakPos = true;
        if (gxc_dps <= -peakThr) hadPeakNeg = true;

        // Deadbanded sign to avoid flicker near zero
        int signNow = signWithDeadband(gxc_dps);

        // Count half-cycle on valid sign change with peak + time guard
        if (signNow != 0 && signNow != lastSign && (tNow - lastCrossTime) >= minGap) {
          bool valid = (signNow == +1 && hadPeakNeg) || (signNow == -1 && hadPeakPos);
          if (valid) {
            halfCycles++;
            lastCrossTime = tNow;

            // Clear the opposite peak so we require a new lobe next time
            if (signNow == +1) hadPeakNeg = false;
            else               hadPeakPos = false;
          }
          lastSign = signNow;
        }

        uint32_t fullCycles = halfCycles / 2;

        if (fullCycles != lastDisplayedCycles) {
          lastDisplayedCycles = fullCycles;
          myData.stateID = 203;
           snprintf(myData.payload, sizeof(myData.payload), "COUNT:%lu", (unsigned long)fullCycles);
          esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
         
        }


        // Stop condition: reached 10 full cycles
        if (fullCycles >= PROB_TARGET_CYCLES) {
          snprintf(myData.payload, sizeof(myData.payload),
                   "PROB_RESULT: %lu", (unsigned long)fullCycles);

          probRunning = false;
          currentState = SENDING;
        }
      }
      
      break;
    case SENDING:
      myData.stateID = 202;
      esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
      //updateTFT("UPLOADING", ST77XX_MAGENTA);
      currentState = WAITING; // Wait for OnDataSent callback
      break;

    case WAITING:
      // Loop does nothing while radio is busy
      break;
  }
}
