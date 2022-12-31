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
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_customhid.h"
#include "MCP23S17.h"
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

int map(uint16_t probed, uint16_t correction)
{
  uint16_t result;
  if (probed < correction)
  {
    result = (((probed - 1056)* 2048) / (correction - 1056)) / 16;
  }
  else
  {
    result = ((((probed - correction)* 2048) / (4095 - correction)) + 2048) / 16;
  }
  return result;
}

void setup_MCP23S17()
{
   MCP23S17_write(IO_DEVICE_1, MCP_IOCONA, 0x38);      //Device Configutation
   MCP23S17_write(IO_DEVICE_1, MCP_IOCONB, 0x38);      //Device Configutation
   MCP23S17_write(IO_DEVICE_1, MCP_IODIRA, 0xFF);      //Set pins as inputs or outputs on side A
   MCP23S17_write(IO_DEVICE_1, MCP_IODIRB, 0xFF);      //Set pins as inputs or outputs on side B
   MCP23S17_write(IO_DEVICE_1, MCP_GPPUA, 0xFF);       //I/O pullup pin state on side A
   MCP23S17_write(IO_DEVICE_1, MCP_GPPUB, 0xFF);       //I/O pullup pin state on side B
   MCP23S17_write(IO_DEVICE_1, MCP_IPOLA, 0xFF);       //Signal polarity on side A
   MCP23S17_write(IO_DEVICE_1, MCP_IPOLB, 0xFF);       //Signal polarity on side B

   MCP23S17_write(IO_DEVICE_2, MCP_IOCONA, 0x38);      //Device Configutation
   MCP23S17_write(IO_DEVICE_2, MCP_IOCONB, 0x38);      //Device Configutation
   MCP23S17_write(IO_DEVICE_2, MCP_IODIRA, 0xFF);      //Set pins as inputs or outputs on side A
   MCP23S17_write(IO_DEVICE_2, MCP_IODIRB, 0xFF);      //Set pins as inputs or outputs on side B
   MCP23S17_write(IO_DEVICE_2, MCP_GPPUA, 0xFF);       //I/O pullup pin state on side A
   MCP23S17_write(IO_DEVICE_2, MCP_GPPUB, 0xFF);       //I/O pullup pin state on side B
   MCP23S17_write(IO_DEVICE_2, MCP_IPOLA, 0xFF);       //Signal polarity on side A
   MCP23S17_write(IO_DEVICE_2, MCP_IPOLB, 0xFF);       //Signal polarity on side B
}

void MCP23S17_write(uint8_t device, uint8_t address, uint8_t value)
{
  uint8_t SPI_TX[3] = {device, address, value};
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi3, SPI_TX, 3, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
}

uint8_t MCP23S17_read(uint8_t device, uint8_t address)
{
  device = device + 1;                                 //change device to read mode 
  //device |= 0x1;                                       //change device to read mode 
  uint8_t received_data = 0;
  uint8_t SPI_TX[2] = {device, address};
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi3, SPI_TX, 2, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi3, &received_data, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
  return received_data;
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
  uint16_t x_correction;
  uint16_t y_correction;
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
  /* USER CODE BEGIN 2 */

  Joystick_buffer[0] = 0; // 1A Extender
  Joystick_buffer[1] = 0; // 1B Extender
  Joystick_buffer[2] = 0; // 2 Extender
  Joystick_buffer[3] = 0; // X
  Joystick_buffer[4] = 0; // Y
  Joystick_buffer[5] = 0; // T
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_buffer, 2);
  HAL_Delay(1000);
  x_correction = ADC_buffer[0];
  y_correction = ADC_buffer[1];
  setup_MCP23S17();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_buffer, 2);
    Joystick_buffer[0] = MCP23S17_read(IO_DEVICE_1, MCP_GPIOA);
    Joystick_buffer[1] = MCP23S17_read(IO_DEVICE_1, MCP_GPIOB);
    //Joystick_buffer[2] = MCP23S17_read(IO_DEVICE_2, MCP_GPIOA);
    Joystick_buffer[2] = MCP23S17_read(IO_DEVICE_2, MCP_GPIOB);
    
    Joystick_buffer[3] = map(ADC_buffer[0], x_correction);
    Joystick_buffer[4] = map(ADC_buffer[1], y_correction);
    Joystick_buffer[5] = map(ADC_buffer[0], x_correction)/2;
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
