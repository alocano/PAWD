#include "imu.h"

typedef struct struct_message {
    int stateID;
    char payload[32];
} struct_message;

struct_message myData; // Buffer to send data
struct_message incomingData; // Buffer to recieve data found in updatetft



//imu

// Super Mini I2C pins 
static const int SDA_PIN = 8;
static const int SCL_PIN = 9;

Adafruit_LSM6DS3TRC imu;
static bool imuReady = false;


static const float alpha   = 0.01f;  // bias tracking rate
static const float restThr = 7.0f;  // dps: near-still threshold for bias update

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