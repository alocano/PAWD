#ifndef IMU_H
#define IMU_H

#include "main.h"
#include <stdint.h>

/* Global cycle counter (full pronation–supination cycles) */
extern volatile uint32_t cycles;

/* IMU high-level API (same names you already use in main.c) */

/* Detect LSM6DS3 / LSM6DSOX; returns 0 on success, -1 on failure */
int  lsm6_detect(void);

/* Configure IMU registers; returns 0 on success, -1 on failure */
int  lsm6_init(void);

/* Read raw gyro/accel data from the IMU over I2C */
void lsm6_read_raw(int16_t *gx, int16_t *gy, int16_t *gz,
                   int16_t *ax, int16_t *ay, int16_t *az);

/* Convert raw data to physical units (dps and g) */
void lsm6_convert(int16_t gx, int16_t gy, int16_t gz,
                  int16_t ax, int16_t ay, int16_t az,
                  float *gx_dps, float *gy_dps, float *gz_dps,
                  float *ax_g,  float *ay_g,  float *az_g);

/* Update on-board cycle count using gyro X in dps */
void update_stats_from_gx(float gx_dps);

#endif /* IMU_H */
