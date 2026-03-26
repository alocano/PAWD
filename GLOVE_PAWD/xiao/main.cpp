#include <Arduino.h>

// main.cpp
#include "espnow_handler.h"
#include "sleep_manager.h"
#include "fsr.h"
#include "imu.h"

typedef enum { STATE_IDLE, STATE_READ_FSR, STATE_READ_LSM, STATE_TRANSMIT, STATE_SLEEP } AppState;
volatile AppState currentState = STATE_IDLE;
volatile uint8_t pendingCommand = 0;

void setup() {
    espnow_init(onCommandReceived); // registers your recv callback
    currentState = STATE_SLEEP;
}

void loop() {
    switch (currentState) {
        case STATE_SLEEP:
            sleep_enter_light(GPIO_WAKEUP_PIN);      // blocks until wakeup
            currentState = STATE_IDLE;
            break;

        case STATE_IDLE:
            if (pendingCommand == 0x01) currentState = STATE_READ_FSR;
            else if (pendingCommand == 0x02) currentState = STATE_READ_LSM;
            break;

        case STATE_READ_FSR:
            fsr_capture(&txBuffer);                  // your existing code
            currentState = STATE_TRANSMIT;
            break;

        case STATE_READ_LSM:
            lsm6ds_capture(&txBuffer);               // your existing code
            currentState = STATE_TRANSMIT;
            break;

        case STATE_TRANSMIT:
            espnow_send(&txBuffer);                  // async, ACK in callback
            break;
    }
}

// Called from ESP-NOW RX interrupt context — keep it short!
void IRAM_ATTR onCommandReceived(uint8_t cmd) {
    pendingCommand = cmd;
    currentState = STATE_IDLE;
}