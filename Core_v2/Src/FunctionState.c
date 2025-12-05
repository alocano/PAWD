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
#include <stdlib.h>
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

void delay( volatile uint32_t* clock){
	//sec is the total amount of seconds
	//clock keep track of amount of seconds then resets to 0



	ssd1306_SetCursor(1,1);
					  ssd1306_WriteString("Delay:", Font_11x18, White);
					  char text[20] = {0};
					  sprintf(text, "%lu", (*clock+1));
					  ssd1306_SetCursor(1, 30);
					  ssd1306_WriteString(text, Font_11x18, White);
					  ssd1306_UpdateScreen();

	//HAL_Delay(1000);




}
void delay2( volatile uint32_t* clock){
	//sec is the total amount of seconds
	//clock keep track of amount of seconds then resets to 0



	ssd1306_SetCursor(1,1);
					  ssd1306_WriteString("Delay:", Font_11x18, White);
					  char text[20] = {0};
					  sprintf(text, "%lu", (*clock+1));
					  ssd1306_SetCursor(1, 30);
					  ssd1306_WriteString(text, Font_11x18, White);
					  ssd1306_UpdateScreen();

	//HAL_Delay(1000);




}
void TapsH(volatile uint32_t* counter){
  //TO-DO: Implement state

	ssd1306_Fill(0);
	ssd1306_UpdateScreen();

	 VCNL4020_HandleTypeDef dev;

	 VCNL4020_Init(&dev, &hi2c1);
	// uint32_t previousTick = HAL_GetTick();
	 char tx_buffer[100];


	 printf("RUN_MATLAB_SCRIPT\n");

	 while (*counter < 10){
	  if (VCNL4020_IsProxReady(&dev)){
		//	  if ( HAL_GetTick() - previousTick >= 100){

				  uint16_t proximity = VCNL4020_ReadProximity(&dev);

				  sprintf(tx_buffer, "%u,%lu\r\n", proximity, *counter);

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

}


void RotH(volatile uint32_t* counter){
   //TO-DO: Implement state

	static uint8_t imu_initialized = 0;

	    // ---------- One-time IMU init ----------
	    if (!imu_initialized)
	    {
	        // Make printf unbuffered (for UART debug prints)
	        setbuf(stdout, NULL);


	        if (lsm6_detect() != 0) {

	        }

	        // Configure IMU (ODR, ranges, etc.)
	        if (lsm6_init() != 0) {

	        }

	        imu_initialized = 1;
	    }
	    // ------- End one-time IMU init ---------

	    int16_t gx = 0;
	    float   gx_dps = 0.0f;

	    // 1) Read gyro X
	    lsm6_read_raw(&gx);

	    // 2) Convert to dps
	    lsm6_convert(gx, &gx_dps);

	    printf("Gx[dps]: %.2f\r\n", gx_dps);

	    // 3) Update cycle detector (this updates global 'cycles')
	    update_stats_from_gx(gx_dps);

	    // 4) Copy global cycles into the state-machine counter
	    *counter = cycles;




	    HAL_Delay(100);   // ~10 Hz update



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

