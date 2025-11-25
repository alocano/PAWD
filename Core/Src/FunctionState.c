/*
 * FunctionState.c
 *
 *  Created on: Nov 17, 2025
 *      Author: gbara
 */

#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "Adafruit_VCNL4020.h"
#include "imu.h"

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c4;
extern UART_HandleTypeDef huart3;

void Starth(uint32_t* counter){
	*counter = 0;
	ssd1306_Fill(0);
  ssd1306_UpdateScreen();
	ssd1306_SetCursor(0,1);
  ssd1306_WriteString("Hello", Font_11x18, White);
  ssd1306_UpdateScreen();
  HAL_Delay(3*1000);
}
void TransH(){
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
void TapsH(volatile uint32_t* counter){
  //TO-DO: Implement state
	 VCNL4020_HandleTypeDef dev;
	 VCNL4020_Init(&dev, &hi2c2);
	// uint32_t previousTick = HAL_GetTick();
	 char tx_buffer[100];

	  if (VCNL4020_IsProxReady(&dev)){
		//	  if ( HAL_GetTick() - previousTick >= 100){

				  uint16_t proximity = VCNL4020_ReadProximity(&dev);

				  sprintf(tx_buffer, "%u,%ln\r\n", proximity, counter);

				  HAL_UART_Transmit(&huart3, (uint8_t*)tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);

			//	  previousTick = HAL_GetTick();
				  VCNL4020_clearInterrupts(&dev, true, false, false, false);
				  ssd1306_SetCursor(1,1);
				  ssd1306_WriteString("Taps Count:", Font_11x18, White);
				  char text[20] = {0};
				  sprintf(text, "%ld", *counter);
				  ssd1306_SetCursor(1, 30);
				  ssd1306_WriteString(text, Font_11x18, White);
				  ssd1306_UpdateScreen();

		//	  }
		}

}


void RotH(volatile uint32_t* counter){
   //TO-DO: Implement state
	int _write(int file, char *ptr, int len);
	setbuf(stdout, NULL); /* unbuffered printf */

	  if (lsm6_detect() != 0) { while (1) { HAL_Delay(500); } }
	  if (lsm6_init()   != 0) { while (1) { HAL_Delay(500); } }

	  int16_t gx, gy, gz, ax, ay, az;
	 	     float gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g;


	 	     lsm6_read_raw(&gx, &gy, &gz, &ax, &ay, &az);
	 	     lsm6_convert(gx, gy, gz, ax, ay, az,
	 	                 &gx_dps, &gy_dps, &gz_dps, &ax_g, &ay_g, &az_g);

	 	     /* NEW: update on-board stats using gyro X (in dps) */
	 	     update_stats_from_gx(gx_dps);

	 	     /*printf("G[dps]: %6.1f %6.1f %6.1f | A[g]: %6.3f %6.3f %6.3f\r\n",
	 	            gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g);*/



	     HAL_Delay(100); /* ~10 Hz print rate */

  //ssd1306_Fill(0);
  //ssd1306_UpdateScreen();
  ssd1306_SetCursor(1,1);
  ssd1306_WriteString("Rotations:", Font_11x18, White);
  char text[20] = {0};
  sprintf(text, "%ld", *counter);
  ssd1306_SetCursor(1, 30);
  ssd1306_WriteString(text, Font_11x18, White);
  ssd1306_UpdateScreen();
}
void EndH(){
  ssd1306_Fill(0);
  ssd1306_UpdateScreen();
  ssd1306_SetCursor(1,1);
  ssd1306_WriteString("Goodbye :D", Font_11x18, White);
  ssd1306_UpdateScreen();
  HAL_Delay(3*1000);
}

