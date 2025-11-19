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
void TapsH(uint32_t* counter){
  ssd1306_SetCursor(1,1);
  ssd1306_WriteString("Taps Count:", Font_11x18, White);
  char text[20] = {0};
  sprintf(text, "%ld", *counter);
  ssd1306_SetCursor(1, 30);
  ssd1306_WriteString(text, Font_11x18, White);
  ssd1306_UpdateScreen();
  (*counter)++;
}


void RotH(uint32_t* counter){
  ssd1306_SetCursor(1,1);
  ssd1306_WriteString("Rotations:", Font_11x18, White);
  char text[20] = {0};
  sprintf(text, "%ld", *counter);
  ssd1306_SetCursor(1, 30);
  ssd1306_WriteString(text, Font_11x18, White);
  ssd1306_UpdateScreen();
  (*counter)++;
}
void EndH(){
  ssd1306_Fill(0);
  ssd1306_UpdateScreen();
  ssd1306_SetCursor(1,1);
  ssd1306_WriteString("Goodbye :D", Font_11x18, White);
  ssd1306_UpdateScreen();
  HAL_Delay(3*1000);
}

