#include "notes.h"

  void tune_init()
  {
	TIM4->ARR = (1000000UL / NOTE_CS5)-1;
	HAL_Delay(50);
	TIM4->CCR4 = 80*923/100;
	HAL_Delay(50);
	TIM4->CCR4 = 0;
	HAL_Delay(50);
	TIM4->CCR4 = 80*923/100;
	HAL_Delay(50);
	TIM4->CCR4 = 0;
	HAL_Delay(50);
	TIM4->CCR4 = 80*923/100;
	HAL_Delay(50);
	TIM4->CCR4 = 0;
  }

  void tune_heartbeat()
  {
	TIM4->ARR = (1000000UL / NOTE_FS3)-1;
	TIM4->CCR4 = 80*923/100;
	HAL_Delay(20);
	TIM4->CCR4 = 0;
	HAL_Delay(100);
	TIM4->ARR = (1000000UL / NOTE_CS3)-1;
	TIM4->CCR4 = 80*923/100;
	HAL_Delay(30);
	TIM4->CCR4 = 0;
  }


  void tune_calibrated()
  {
	TIM4->ARR = (1000000UL / NOTE_C4)-1;
	TIM4->CCR4 = 80*923/100;
	HAL_Delay(100);
	TIM4->CCR4 = 0;
	HAL_Delay(100);
	TIM4->CCR4 = 80*923/100;
	HAL_Delay(20);
	TIM4->CCR4 = 0;
	HAL_Delay(100);
	TIM4->ARR = (1000000UL / NOTE_B4)-1;
	TIM4->CCR4 = 80*923/100;
	HAL_Delay(200);
	TIM4->CCR4 = 0;

  }

  