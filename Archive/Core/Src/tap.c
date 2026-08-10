/*
 * tap.c
 *
 *  Created on: Nov 7, 2025
 *      Author: waith
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "Adafruit_VCNL4020.h"
#include "tap.h"
#include "main.h"


extern I2C_HandleTypeDef hi2c2;

SensorBuffer_t pre_capture_buffer = {0};
SensorBuffer_t post_capture_buffer = {0};



void StartPreCapture(void){

    pre_capture_buffer.capturing = true;
    pre_capture_buffer.write_index = 0;
}

void PreCaptureTask(VCNL4020_HandleTypeDef *dev){

    if (!pre_capture_buffer.capturing) return;

    uint32_t current_index = pre_capture_buffer.write_index;

    if (current_index < BUFFER_SIZE) {
        // Read sensor data
        pre_capture_buffer.proximity_data[current_index] = VCNL4020_ReadProximity(dev);
        pre_capture_buffer.timestamp[current_index] = HAL_GetTick();

        pre_capture_buffer.write_index++;


    } else {
        // Buffer full - wrap around (circular buffer)
        pre_capture_buffer.write_index = 0;
    }
}


void PostCaptureTask(VCNL4020_HandleTypeDef *dev){

    if (!post_capture_buffer.capturing) return;

    uint32_t current_index = post_capture_buffer.write_index;

    if (current_index < BUFFER_SIZE) {
        // Read sensor data
        post_capture_buffer.proximity_data[current_index] = VCNL4020_ReadProximity(dev);
        post_capture_buffer.timestamp[current_index] = HAL_GetTick();

        post_capture_buffer.write_index++;

        //HAL_Delay(1000);
    } else {
        // Buffer full - stop capturing
        post_capture_buffer.capturing = false;
    }
}

void ExportSensorData(SensorBuffer_t* pre, SensorBuffer_t* post){

    printf("Pre-Tap data:\r\n");

    for (uint32_t i = 0; i < pre->write_index; i++) {
        printf("Time: %lu, Prox: %u\r\n", pre->timestamp[i], pre->proximity_data[i]);
    }

    printf("Post-Tap data:\r\n");

    for (uint32_t i = 0; i < post->write_index; i++) {
        printf("Time: %lu, Prox: %u\r\n", post->timestamp[i], post->proximity_data[i]);
    }
}

void ProcessCapturedData(void){
    // Save pre-capture data (last N samples before switch)
    uint32_t pre_samples = pre_capture_buffer.write_index;
    uint32_t post_samples = post_capture_buffer.write_index;

    // Export data via UART, USB, or save to external memory
    ExportSensorData(&pre_capture_buffer, &post_capture_buffer);
}
