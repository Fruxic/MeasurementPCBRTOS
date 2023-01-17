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
#include "cmsis_os.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "definesMeas.h"
#include "variablesMeas.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
HAL_StatusTypeDef ret;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  if (ARM_CM_DWT_CTRL != 0) {        // See if DWT is available
	  ARM_CM_DEMCR      |= 1 << 24;  // Set bit 24
	  ARM_CM_DWT_CYCCNT  = 0;
	  ARM_CM_DWT_CTRL   |= 1 << 0;   // Set bit 0
  }
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /*initialize humidity module (I2C)*/
  //Check if device is connected
  if((ret = HAL_I2C_IsDeviceReady(&hi2c1, SHT31_ADDR, 1, HAL_MAX_DELAY)) != HAL_OK){
	  //error handler
  }
  //Disable heater
  I2C_trans[0] = SHT31_HEATER_First;
  I2C_trans[1] = SHT31_HEATER_Second;
  if((ret = HAL_I2C_Master_Transmit(&hi2c1, SHT31_ADDR, I2C_trans, 2, HAL_MAX_DELAY)) != HAL_OK){
	  //error handler
  }
  /*Initialize accelerometer module (I2C)*/
  //check if device is connected
  if((ret = HAL_I2C_IsDeviceReady(&hi2c1, LIS2_ADDR, 1, HAL_MAX_DELAY)) != HAL_OK){
	  //error handler
  }
  //Write setting to control register 1
  I2C_trans[0] = LIS2_CTRL1_ADDR;
  I2C_trans[1] = LIS2_CTRL1_Write;
  if((ret = HAL_I2C_Master_Transmit(&hi2c1, LIS2_ADDR, I2C_trans, 2, HAL_MAX_DELAY)) != HAL_OK){
	  //error handler
  }
  //Write setting to FIFO register
  I2C_trans[0] = LIS2_FIFO_ADDR;
  I2C_trans[1] = LIS2_FIFO_Write;
  if((ret = HAL_I2C_Master_Transmit(&hi2c1, LIS2_ADDR, I2C_trans, 2, HAL_MAX_DELAY)) != HAL_OK){
	  //error handler
  }
  /*Initialize Temp module (SPI)*/
  //Reset command
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  SPI_trans[0] = MCP_CMD_RESET;
  if((ret = HAL_SPI_Transmit(&hspi1, SPI_trans, 1, HAL_MAX_DELAY)) != HAL_OK){
	  //error handler
	  while(1);
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  //Setup MUX
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  SPI_trans[0] = MCP_ADDR | MCP_MUX | MCP_WRITE;
  SPI_trans[1] = 0x01;
  if((ret = HAL_SPI_Transmit(&hspi1, SPI_trans, 2, HAL_MAX_DELAY)) != HAL_OK){
	  //error handler
	  while(1);
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  SPI_trans[0] = MCP_ADDR | MCP_MUX | MCP_STATICREAD;
  if((ret = HAL_SPI_Transmit(&hspi1, SPI_trans, 1, HAL_MAX_DELAY)) != HAL_OK){
	  //error handler
	  while(1);
  }
  if((ret = HAL_SPI_Receive(&hspi1, SPI_recv, 1, HAL_MAX_DELAY)) != HAL_OK){
	  //error handler
	  while(1);
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  //Write settings to config register 0
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  SPI_trans[0] = MCP_ADDR | MCP_CONF0 | MCP_WRITE;
  SPI_trans[1] = 0xC3;
  if((ret = HAL_SPI_Transmit(&hspi1, SPI_trans, 2, HAL_MAX_DELAY)) != HAL_OK){
	  //error handler
	  while(1);
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  SPI_trans[0] = MCP_ADDR | MCP_CONF0 | MCP_STATICREAD;	//01000101
  if((ret = HAL_SPI_Transmit(&hspi1, SPI_trans, 1, HAL_MAX_DELAY)) != HAL_OK){
	  //error handler
	  while(1);
  }
  if((ret = HAL_SPI_Receive(&hspi1, SPI_recv, 1, HAL_MAX_DELAY)) != HAL_OK){
	  //error handler
	  while(1);
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  //Write settings to config register 1
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  SPI_trans[0] = MCP_ADDR | MCP_CONF1 | MCP_WRITE;
  SPI_trans[1] = 0xCC; //F0 for diff CC for single
  if((ret = HAL_SPI_Transmit(&hspi1, SPI_trans, 2, HAL_MAX_DELAY)) != HAL_OK){
	  //error handler
	  while(1);
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  SPI_trans[0] = MCP_ADDR | MCP_CONF1 | MCP_STATICREAD;
  if((ret = HAL_SPI_Transmit(&hspi1, SPI_trans, 1, HAL_MAX_DELAY)) != HAL_OK){
	  //error handler
	  while(1);
  }
  if((ret = HAL_SPI_Receive(&hspi1, SPI_recv, 1, HAL_MAX_DELAY)) != HAL_OK){
	  //error handler
	  while(1);
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  //Write settings to config register 2
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  SPI_trans[0] = MCP_ADDR | MCP_CONF2 | MCP_WRITE;
  SPI_trans[1] = 0x0B;
  if((ret = HAL_SPI_Transmit(&hspi1, SPI_trans, 2, HAL_MAX_DELAY)) != HAL_OK){
	  //error handler
	  while(1);
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  SPI_trans[0] = MCP_ADDR | MCP_CONF2 | MCP_STATICREAD;
  if((ret = HAL_SPI_Transmit(&hspi1, SPI_trans, 1, HAL_MAX_DELAY)) != HAL_OK){
	  //error handler
	  while(1);
  }
  if((ret = HAL_SPI_Receive(&hspi1, SPI_recv, 1, HAL_MAX_DELAY)) != HAL_OK){
	  //error handler
	  while(1);
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  //Write settings to config register 3
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  SPI_trans[0] = MCP_ADDR | MCP_CONF3 | MCP_WRITE;
  SPI_trans[1] = 0xC0;
  if((ret = HAL_SPI_Transmit(&hspi1, SPI_trans, 2, HAL_MAX_DELAY)) != HAL_OK){
	  //error handler
	  while(1);
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  SPI_trans[0] = MCP_ADDR | MCP_CONF3 | MCP_STATICREAD;
  if((ret = HAL_SPI_Transmit(&hspi1, SPI_trans, 1, HAL_MAX_DELAY)) != HAL_OK){
	  //error handler
	  while(1);
  }
  if((ret = HAL_SPI_Receive(&hspi1, SPI_recv, 1, HAL_MAX_DELAY)) != HAL_OK){
	  //error handler
	  while(1);
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  //Write settings to config IRQ
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  SPI_trans[0] = MCP_ADDR | MCP_IRQ | MCP_WRITE;
  SPI_trans[1] = 0x77;
  if((ret = HAL_SPI_Transmit(&hspi1, SPI_trans, 2, HAL_MAX_DELAY)) != HAL_OK){
	  while(1);
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  SPI_trans[0] = MCP_ADDR | MCP_IRQ | MCP_STATICREAD;
  if((ret = HAL_SPI_Transmit(&hspi1, SPI_trans, 1, HAL_MAX_DELAY)) != HAL_OK){
	  while(1);
  }
  if((ret = HAL_SPI_Receive(&hspi1, SPI_recv, 1, HAL_MAX_DELAY)) != HAL_OK){
	  while(1);
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  //Conversion start thermistor
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  SPI_trans[0] = MCP_CMD_CONV;
  if((ret = HAL_SPI_Transmit(&hspi1, SPI_trans, 1, HAL_MAX_DELAY)) != HAL_OK){
	  while(1);
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_Delay(500);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 256;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
float complexABS(float real, float compl) {
	return sqrtf((real*real)+(compl*compl));
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
