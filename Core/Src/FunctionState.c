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
	 uint32_t previousTick = HAL_GetTick();
	 char tx_buffer[100];

	  if (VCNL4020_IsProxReady(&dev) && counter <=10 ){
			  if ( HAL_GetTick() - previousTick >= 100){

				  uint16_t proximity = VCNL4020_ReadProximity(&dev);

				  sprintf(tx_buffer, "%u,%lu\r\n", proximity, counter);

				  HAL_UART_Transmit(&huart3, (uint8_t*)tx_buffer, strlen(tx_buffer), HAL_MAX_DELAY);

				  previousTick = HAL_GetTick();

				  ssd1306_SetCursor(1,1);
				  ssd1306_WriteString("Taps Count:", Font_11x18, White);
				  char text[20] = {0};
				  sprintf(text, "%ld", *counter);
				  ssd1306_SetCursor(1, 30);
				  ssd1306_WriteString(text, Font_11x18, White);
				  ssd1306_UpdateScreen();

			  }
		}

}


void RotH(uint32_t* counter){
   //TO-DO: Implement state
  ssd1306_Fill(0);
  ssd1306_UpdateScreen();
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

