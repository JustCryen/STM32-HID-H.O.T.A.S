/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #define serial_MCP23S17
// #define serial_MCP23017
#include "tunes.c"
#include "serial.c"
#include "usbd_customhid.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t ADC_buffer[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern USBD_HandleTypeDef hUsbDeviceFS;

int centering(uint16_t probed, uint16_t correction)
{
  uint16_t result;
  if (probed < correction)
  {
    result = (4095 - (((probed - 1056)* 2048) / (correction - 1056)));
  }
  else
  {
    result = (4095 - ((((probed - correction)* 2048) / (4095 - correction)) + 2048));
  }
  return result;
}
int expo(uint16_t probed2, uint16_t correction2)
{
  uint16_t x = centering(probed2, correction2);
  uint16_t result2;
  if (abs(x-2048) < 64)
  {
	result2 = x/16;
  }
  else
  {
	if (x > 2048)
	{
		result2 = ( 64 * exp((( x)-2048) / (510.0) ) + 2048)/16;
		if (result2 > 255) result2 = 255;
	}
	else
	{
		result2 = (-64 * exp(((-x)+2048) / (510.0) ) + 2048)/16;
	}
  }
  return result2;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t Joystick_buffer[6];
  uint8_t Extender_raw[4];
  uint16_t x_correction;
  uint16_t y_correction;
  uint8_t ThrottleBT[6] = {'\r'};
  uint16_t Throttle = 0;
  uint16_t heartbit_delay = 50;
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
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_SPI3_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  Joystick_buffer[0] = 0; // T
  Joystick_buffer[1] = 0; // X
  Joystick_buffer[2] = 0; // Y
  Joystick_buffer[3] = 0; // Extender
  Joystick_buffer[4] = 0; // Extender
  Joystick_buffer[5] = 0; // Extender
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  tune_init();
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_buffer, 2);
  HAL_Delay(1000);
  x_correction = ADC_buffer[0];
  y_correction = ADC_buffer[1];
  setup_MCP23X17();
  if (protocol) {
	MCP23X17_write(IO_DEVICE_1, MCP_OLATB, 0x40);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (heartbit_delay > 0)
	  heartbit_delay -= 1;
	else {
      tune_heartbeat();
	  heartbit_delay = 100;
	}

    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0) {
	  setup_MCP23017();
      x_correction = ADC_buffer[0];
      y_correction = ADC_buffer[1];
	  tune_calibrated();
	}

    HAL_UART_Receive_DMA(&huart2, ThrottleBT, 6);
    if (ThrottleBT[0] == 'x')
    {
      ThrottleBT[0] = '0';
      Throttle = atoi((char*) ThrottleBT);
    }
	
    Extender_raw[0] = MCP23X17_read(IO_DEVICE_1, MCP_GPIOA);
    Extender_raw[1] = MCP23X17_read(IO_DEVICE_1, MCP_GPIOB);
    Extender_raw[2] = MCP23X17_read(IO_DEVICE_2, MCP_GPIOA);
    Extender_raw[3] = MCP23X17_read(IO_DEVICE_2, MCP_GPIOB);
    uint8_t Extender_data[3] = {0};

    for(int index = 0; index < 24; index++)
    {
      InputIndexing *input_loc = input_map + index;
      int Extender_value = (Extender_raw[input_loc->byte_index] >> input_loc->bit_index) & 0x1;
      int curr_byte_index = index / 8;
      int curr_bit_index  = index % 8;
      Extender_data[curr_byte_index] |= Extender_value << curr_bit_index;
    }

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_buffer, 2);
    // Joystick_buffer[0] = Throttle/4;
    Joystick_buffer[0] = centering(ADC_buffer[0], x_correction)/16;
    Joystick_buffer[1] = expo(ADC_buffer[0], x_correction);
    Joystick_buffer[2] = expo(ADC_buffer[1], y_correction);
    Joystick_buffer[3] = Extender_data[0];
    Joystick_buffer[4] = Extender_data[1];
    Joystick_buffer[5] = Extender_data[2];
    USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, Joystick_buffer, 6);
    HAL_Delay(50);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

#ifdef  USE_FULL_ASSERT
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
