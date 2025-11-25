#include "imu.h"
#include <math.h>
#include <stdio.h>

/* ---- LSM6DS3 register map ---- */
#define LSM6DS3_WHO_AM_I     0x0F
#define LSM6DS3_CTRL1_XL     0x10
#define LSM6DS3_CTRL2_G      0x11
#define LSM6DS3_CTRL3_C      0x12
#define LSM6DS3_OUTX_L_G     0x22  /* GxL,GxH,GyL,GyH,GzL,GzH */
#define LSM6DS3_OUTX_L_XL    0x28  /* AxL,AxH,AyL,AyH,AzL,AzH */

/* Expected IDs */
#define WHO_AM_I_DS3   0x69
#define WHO_AM_I_DSOX  0x6C

/* 7-bit I2C addresses shifted left by 1 for HAL */
#define ADDR_6A              (0x6A << 1)   /* Adafruit default */
#define ADDR_6B              (0x6B << 1)

/* Global cycle counter (definition; declaration is in imu.h) */

extern volatile uint32_t cycles;
/* IMU I2C address selected at runtime by lsm6_detect() */
static uint16_t lsm6_addr = ADDR_6A;

/* hi2c1 is defined in main.c; we just reference it here */
extern I2C_HandleTypeDef hi2c1;

/* ---------- Local I2C helpers ---------- */
static HAL_StatusTypeDef i2c_write(uint16_t addr, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c1, addr, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

static HAL_StatusTypeDef i2c_read(uint16_t addr, uint8_t reg,
                                  uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, addr, reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

/* ---------- Public IMU functions ---------- */

int lsm6_detect(void)
{
    uint8_t id = 0;

    /* Try 0x6A first */
    if (i2c_read(ADDR_6A, LSM6DS3_WHO_AM_I, &id, 1) == HAL_OK) {
        printf("WHO_AM_I @0x6A = 0x%02X\r\n", id);
        if (id == WHO_AM_I_DS3 || id == WHO_AM_I_DSOX || id == 0x6A) {
            lsm6_addr = ADDR_6A;
            return 0;
        }
    }

    /* Then try 0x6B */
    if (i2c_read(ADDR_6B, LSM6DS3_WHO_AM_I, &id, 1) == HAL_OK) {
        printf("WHO_AM_I @0x6B = 0x%02X\r\n", id);
        if (id == WHO_AM_I_DS3 || id == WHO_AM_I_DSOX || id == 0x6A) {
            lsm6_addr = ADDR_6B;
            return 0;
        }
    }

    return -1;
}

/* Configure: BDU+auto-increment, accel 104Hz ±4g, gyro 104Hz 2000dps */
int lsm6_init(void)
{
    /* CTRL3_C: BDU=1 (bit6), IF_INC=1 (bit2) => 0x44 */
    if (i2c_write(lsm6_addr, LSM6DS3_CTRL3_C, 0x44) != HAL_OK) return -1;

    /* CTRL1_XL: ODR=104Hz (0100<<4), FS=±4g (10<<2), BW=00 => 0x48 */
    if (i2c_write(lsm6_addr, LSM6DS3_CTRL1_XL, 0x48) != HAL_OK) return -1;

    /* CTRL2_G:  ODR=104Hz (0100<<4), FS=2000dps (11<<2) => 0x4C */
    if (i2c_write(lsm6_addr, LSM6DS3_CTRL2_G, 0x4C) != HAL_OK) return -1;

    return 0;
}

void lsm6_read_raw(int16_t *gx, int16_t *gy, int16_t *gz,
                   int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];

    if (i2c_read(lsm6_addr, LSM6DS3_OUTX_L_G, buf, 6) == HAL_OK) {
        *gx = (int16_t)((buf[1] << 8) | buf[0]);
        *gy = (int16_t)((buf[3] << 8) | buf[2]);
        *gz = (int16_t)((buf[5] << 8) | buf[4]);
    } else {
        *gx = *gy = *gz = 0;
    }

    if (i2c_read(lsm6_addr, LSM6DS3_OUTX_L_XL, buf, 6) == HAL_OK) {
        *ax = (int16_t)((buf[1] << 8) | buf[0]);
        *ay = (int16_t)((buf[3] << 8) | buf[2]);
        *az = (int16_t)((buf[5] << 8) | buf[4]);
    } else {
        *ax = *ay = *az = 0;
    }
}

void lsm6_convert(int16_t gx, int16_t gy, int16_t gz,
                  int16_t ax, int16_t ay, int16_t az,
                  float *gx_dps, float *gy_dps, float *gz_dps,
                  float *ax_g,  float *ay_g,  float *az_g)
{
    /* Sensitivities:
       Gyro 2000 dps: 0.07 dps/LSB
       Accel ±4 g:    0.122 mg/LSB = 0.000122 g/LSB */
    *gx_dps = gx * 0.07f;
    *gy_dps = gy * 0.07f;
    *gz_dps = gz * 0.07f;

    *ax_g = ax * 0.000122f;
    *ay_g = ay * 0.000122f;
    *az_g = az * 0.000122f;
}

/* MATLAB cycle-detector  */
void update_stats_from_gx(float gx_dps)
{
    /* Parameters from your MATLAB code */
    const float restThr   = 15.0f;   // near-still threshold (dps) for bias
    const float alpha     = 0.01f;   // bias filter
    const float deadband  = 15.0f;   // hysteresis around zero (dps)
    const float peakThr   = 50.0f;   // must see lobe exceed this (dps)
    const float minGap    = 0.25f;   // refractory gap between crossings (s)

    static uint8_t  init = 0;
    static float    bias = 0.0f;

    static uint32_t t0_ms = 0;
    static uint32_t prev_ms = 0;

    static int8_t   lastSign = 0;          // -1,0,+1
    static float    lastCrossTime = 0.0f;  // seconds
    static uint8_t  hadPeakPos = 0;
    static uint8_t  hadPeakNeg = 0;
    static uint32_t halfCycles = 0;        // two half cycles = one full cycle

    /* Time base from HAL_GetTick() */
    uint32_t now_ms = HAL_GetTick();
    if (!init) {
        init = 1;
        t0_ms = now_ms;
        prev_ms = now_ms;
    }
    float tNow = (now_ms - t0_ms) / 1000.0f;          // seconds since start
    float dt   = (now_ms - prev_ms) / 1000.0f;        // seconds since last sample
    if (dt < 0.0f) dt = 0.0f;
    prev_ms = now_ms;

    /* Auto-zero bias */
    if (fabsf(gx_dps - bias) < restThr) {
        bias = (1.0f - alpha) * bias + alpha * gx_dps;
    }
    float gxc = gx_dps - bias;    // centered gyro X

    /* ----- Cycle counting logic ----- */
    if (gxc >=  peakThr) hadPeakPos = 1;
    if (gxc <= -peakThr) hadPeakNeg = 1;

    int8_t signNow;
    if      (gxc >  deadband) signNow = +1;
    else if (gxc < -deadband) signNow = -1;
    else                      signNow = 0;

    if (signNow != 0 && signNow != lastSign && (tNow - lastCrossTime) >= minGap) {
        uint8_t valid =
            (signNow == +1 && hadPeakNeg) ||
            (signNow == -1 && hadPeakPos);

        if (valid) {
            halfCycles++;
            lastCrossTime = tNow;

            if (signNow == +1) hadPeakNeg = 0;
            else               hadPeakPos = 0;

            cycles = halfCycles / 2;    // update global full-cycle counter
        }
        lastSign = signNow;
    }
}
