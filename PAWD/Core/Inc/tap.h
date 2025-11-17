/*
 * tap.h
 *
 *  Created on: Nov 7, 2025
 *      Author: waith
 */

#ifndef INC_TAP_H_
#define INC_TAP_H_

#define BUFFER_SIZE 1000

typedef struct {
    uint16_t proximity_data[BUFFER_SIZE];
    uint32_t timestamp[BUFFER_SIZE];
    volatile uint32_t write_index;
    bool capturing;
} SensorBuffer_t;


void StartPreCapture(void);
void PreCaptureTask(VCNL4020_HandleTypeDef *dev);
void PostCaptureTask(VCNL4020_HandleTypeDef *dev);
void ExportSensorData(SensorBuffer_t* pre, SensorBuffer_t* post);
void ProcessCapturedData(void);

#endif /* INC_TAP_H_ */
