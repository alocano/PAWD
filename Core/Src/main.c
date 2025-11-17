/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "Adafruit_VCNL4020.h"
#include "tap.h"

/* ---- LSM6DS3 register map ---- */
#define LSM6DS3_WHO_AM_I     0x0F
#define LSM6DS3_CTRL1_XL     0x10
#define LSM6DS3_CTRL2_G      0x11
#define LSM6DS3_CTRL3_C      0x12
#define LSM6DS3_OUTX_L_G     0x22  /* GxL,GxH,GyL,GyH,GzL,GzH */
#define LSM6DS3_OUTX_L_XL    0x28  /* AxL,AxH,AyL,AyH,AzL,AzH */

/* Expected IDs */
#define WHO_AM_I_DS3   0x69
#define WHO_AM_I_DSOX  0x6C  /* just in case user actually has DSOX */

/* 7-bit I2C addresses shifted left by 1 for HAL */
#define ADDR_6A              (0x6A << 1)   /* Adafruit default */
#define ADDR_6B              (0x6B << 1)

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define TAP_GPIO_Port GPIOG
#define TAP_GPIO_Pin GPIO_PIN_0

// OLED State Machine
typedef enum{
  Start_State,
  Trans_State,
  Taps_State,
  Rot_State,
  End_State
}eSystemState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
// OLED State Machine Variables
eSystemState eNextState = Start_State;
uint32_t oled_counter = 0;

// FingerTap Variables
extern SensorBuffer_t pre_capture_buffer;
extern SensorBuffer_t post_capture_buffer;
VCNL4020_HandleTypeDef dev;

// IMU Variables
static uint16_t lsm6_addr = ADDR_6A; /* Will be set by detect() */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

// OLED State Machine Functions
void StartHandler(uint32_t* counter);
void TransHandler(void);
void TapsHandler(uint32_t* counter);
void RotHandler(uint32_t* counter);
void EndHandler(void);

// FingerTap Functions
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

// IMU Functions
int _write(int file, char *ptr, int len);
static HAL_StatusTypeDef i2c_write(uint16_t addr, uint8_t reg, uint8_t val);
static HAL_StatusTypeDef i2c_read(uint16_t addr, uint8_t reg, uint8_t *buf, uint16_t len);
static int  lsm6_detect(void);
static int  lsm6_init(void);
static void lsm6_read_raw(int16_t *gx, int16_t *gy, int16_t *gz,
                          int16_t *ax, int16_t *ay, int16_t *az);
static void lsm6_convert(int16_t gx, int16_t gy, int16_t gz,
                         int16_t ax, int16_t ay, int16_t az,
                         float *gx_dps, float *gy_dps, float *gz_dps,
                         float *ax_g, float *ay_g, float *az_g);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// OLED State Machine Handlers
void StartHandler(uint32_t* counter){
	*counter = 0;
	ssd1306_Fill(0);
    ssd1306_UpdateScreen();
	ssd1306_SetCursor(0,1);
    ssd1306_WriteString("Hello", Font_11x18, White);
    ssd1306_UpdateScreen();
    HAL_Delay(3*1000);
}

void TransHandler(){
    ssd1306_Fill(0);
	ssd1306_UpdateScreen();

	ssd1306_SetCursor(1,1);
    ssd1306_WriteString("Choose :D", Font_11x18, White);

    ssd1306_SetCursor(1,20);
    ssd1306_WriteString("1.Pronation", Font_11x18, White);

    ssd1306_SetCursor(1,40);
    ssd1306_WriteString("2.Taps", Font_11x18, White);
    ssd1306_UpdateScreen();
    HAL_Delay(3*1000);
    ssd1306_Fill(0);
    ssd1306_UpdateScreen();
}

void TapsHandler(uint32_t* counter){
    ssd1306_SetCursor(1,1);
    ssd1306_WriteString("Taps Count:", Font_11x18, White);
    char text[20] = {0};
    sprintf(text, "%ld", *counter);
    ssd1306_SetCursor(1, 30);
    ssd1306_WriteString(text, Font_11x18, White);
    ssd1306_UpdateScreen();
    (*counter)++;
}

void RotHandler(uint32_t* counter){
    ssd1306_Fill(0);
    ssd1306_UpdateScreen();
    ssd1306_SetCursor(1,1);
    ssd1306_WriteString("Rotations:", Font_11x18, White);
    char text[20] = {0};
    sprintf(text, "%ld", *counter);
    ssd1306_SetCursor(1, 30);
    ssd1306_WriteString(text, Font_11x18, White);
    ssd1306_UpdateScreen();
    (*counter)++;
}

void EndHandler(){
    ssd1306_Fill(0);
    ssd1306_UpdateScreen();
    ssd1306_SetCursor(1,1);
    ssd1306_WriteString("Goodbye :D", Font_11x18, White);
    ssd1306_UpdateScreen();
    HAL_Delay(3*1000);
}

// FingerTap User Code
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == TAP_GPIO_Pin) {
        if (HAL_GPIO_ReadPin(TAP_GPIO_Port, TAP_GPIO_Pin) == GPIO_PIN_RESET) {
            // Switch pressed - start post-capture
            pre_capture_buffer.capturing = false;
            post_capture_buffer.capturing = true;
            post_capture_buffer.write_index = 0;
        } else {
            // Switch released - stop capturing
            post_capture_buffer.capturing = false;
        }
    }
}

// IMU User Code
/* Route printf() to USART3 so text shows on the ST-LINK Virtual COM port */
int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}

/* I2C helpers */
static HAL_StatusTypeDef i2c_write(uint16_t addr, uint8_t reg, uint8_t val) {
  return HAL_I2C_Mem_Write(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

static HAL_StatusTypeDef i2c_read(uint16_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
  return HAL_I2C_Mem_Read(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

/* Try Adafruit default 0x6A first, then 0x6B. Accept DS3 (0x69) and also report DSOX (0x6C). */
static int lsm6_detect(void) {
  uint8_t id = 0;

  if (i2c_read(ADDR_6A, LSM6DS3_WHO_AM_I, &id, 1) == HAL_OK) {
    printf("WHO_AM_I @0x6A = 0x%02X\r\n", id);
    if (id == 0x69 || id == 0x6A || id == 0x6C) { lsm6_addr = ADDR_6A; return 0; }
  }
  if (i2c_read(ADDR_6B, LSM6DS3_WHO_AM_I, &id, 1) == HAL_OK) {
    printf("WHO_AM_I @0x6B = 0x%02X\r\n", id);
    if (id == 0x69 || id == 0x6A || id == 0x6C) { lsm6_addr = ADDR_6B; return 0; }
  }
  return -1;
}

/* Configure: BDU+auto-increment, accel 104Hz ±4g, gyro 104Hz 2000dps */
static int lsm6_init(void) {
  /* CTRL3_C: BDU=1 (bit6), IF_INC=1 (bit2) => 0x44 */
  if (i2c_write(lsm6_addr, LSM6DS3_CTRL3_C, 0x44) != HAL_OK) return -1;

  /* CTRL1_XL: ODR=104Hz (0100<<4), FS=±4g (10<<2), BW=00 => 0x48 */
  if (i2c_write(lsm6_addr, LSM6DS3_CTRL1_XL, 0x48) != HAL_OK) return -1;

  /* CTRL2_G:  ODR=104Hz (0100<<4), FS=2000dps (11<<2) => 0x4C */
  if (i2c_write(lsm6_addr, LSM6DS3_CTRL2_G, 0x4C) != HAL_OK) return -1;

  return 0;
}

static void lsm6_read_raw(int16_t *gx, int16_t *gy, int16_t *gz,
                          int16_t *ax, int16_t *ay, int16_t *az) {
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

static void lsm6_convert(int16_t gx, int16_t gy, int16_t gz,
                         int16_t ax, int16_t ay, int16_t az,
                         float *gx_dps, float *gy_dps, float *gz_dps,
                         float *ax_g, float *ay_g, float *az_g) {
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  setbuf(stdout, NULL); /* unbuffered printf */

  // OLED Initialization
  ssd1306_Init();

  // FingerTap Initialization
  if (!VCNL4020_Init(&dev, &hi2c2)) {
      printf("Failed to initialize VCNL4020!\r\n");
  } else {
      printf("VCNL4020 initialized successfully\r\n");
      StartPreCapture();

      VCNL4020_SetProxRate(&dev, PROX_RATE_16_6_PER_S);
      VCNL4020_SetProxLEDmA(&dev, 200);
      VCNL4020_SetProxFrequency(&dev, PROX_FREQ_390_625_KHZ);
  }

  // IMU Initialization
  if (lsm6_detect() != 0) {
      printf("LSM6DS3 detection failed!\r\n");
  }
  if (lsm6_init() != 0) {
      printf("LSM6DS3 initialization failed!\r\n");
  }

  printf("All systems initialized successfully\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // OLED State Machine
    switch(eNextState){
        case Start_State:
            StartHandler(&oled_counter);
            eNextState = Trans_State;
            break;
        case Trans_State:
            TransHandler();
            eNextState = Taps_State;  // Can be modified to choose between Taps_State and Rot_State
            break;
        case Taps_State:
            TapsHandler(&oled_counter);
            // Run FingerTap tasks while in Taps state
            if(VCNL4020_IsProxReady(&dev)){
                PreCaptureTask(&dev);
                PostCaptureTask(&dev);
            }
            ProcessCapturedData();
            VCNL4020_clearInterrupts(&dev, true, false, false, false);

            eNextState = (oled_counter < 9) ? Taps_State : End_State;
            HAL_Delay(500); // Slower update for tap counting
            break;
        case Rot_State:
            RotHandler(&oled_counter);
            // Run IMU tasks while in Rotation state
            int16_t gx, gy, gz, ax, ay, az;
            float gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g;

            lsm6_read_raw(&gx, &gy, &gz, &ax, &ay, &az);
            lsm6_convert(gx, gy, gz, ax, ay, az,
                        &gx_dps, &gy_dps, &gz_dps, &ax_g, &ay_g, &az_g);

            printf("G[dps]: %6.1f %6.1f %6.1f | A[g]: %6.3f %6.3f %6.3f | Rotations: %ld\r\n",
                  gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, oled_counter);

            eNextState = (oled_counter < 9) ? Rot_State : End_State;
            HAL_Delay(500); // Slower update for rotation counting
            break;
        case End_State:
            EndHandler();
            eNextState = Start_State;
            break;
        default:
            eNextState = Start_State;
            break;
    }
  }
  /* USER CODE END 3 */
}
/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{
  /* USER CODE BEGIN ETH_Init 0 */
  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */
  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */
  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */
  /* USER CODE END ETH_Init 2 */
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */
  /* USER CODE BEGIN I2C1_Init 1 */
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{
  /* USER CODE BEGIN I2C2_Init 0 */
  /* USER CODE END I2C2_Init 0 */
  /* USER CODE BEGIN I2C2_Init 1 */
  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x2000090E;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */
  /* USER CODE END I2C2_Init 2 */
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */
  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  /* USER CODE BEGIN USART3_Init 0 */
  /* USER CODE END USART3_Init 0 */
  /* USER CODE BEGIN USART3_Init 1 */
  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  /* USER CODE END USART3_Init 2 */
}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{
  /* USER CODE BEGIN USB_OTG_FS_Init 0 */
  /* USER CODE END USB_OTG_FS_Init 0 */
  /* USER CODE BEGIN USB_OTG_FS_Init 1 */
  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */
  /* USER CODE END USB_OTG_FS_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PG0 (Tap Sensor) */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
