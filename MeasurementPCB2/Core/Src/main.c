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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define	ARM_MATH_CM4
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CLOCK				100000000
#define VOLTAGE				3.3
#define RESISTANCE			10000
#define KELVIN				273.15
#define A					0.001158902845267
#define B					0.00022421325658
#define C					0.000001081046519710
#define D					0.00000004905515304295

#define SHT31_ADDR			0x44<<1
#define SHT31_HEATER_First	0x30
#define SHT31_HEATER_Second	0x66
#define SHT31_MEASURE_First	0x2C

#define LIS2_ADDR			0x1D<<1
//First Control register for accelerometer
#define LIS2_CTRL1_ADDR		0x20
#define LIS2_CTRL1_Write	0x78
//Second Control register for accelerometer
#define LIS2_CTRL2_ADDR		0x21
#define LIS2_CTRL2_Write	0x18
//FIFO Control register for accelerometer
#define LIS2_FIFO_ADDR		0x25
#define LIS2_FIFO_Write		0xC0
//X-axis output
#define LIS2_OUTXL			0x28
#define LIS2_OUTXH			0x29
//Y-axis output
#define LIS2_OUTYL			0x2A
#define LIS2_OUTYH			0x2B
//Z-axis output
#define LIS2_OUTZL			0x2C
#define LIS2_OUTZH			0x2D

//ADC address
#define MCP_ADDR			0x01 << 6
//ADC Command Byte
#define MCP_CMD_CONV		0x68
#define MCP_CMD_RESET		0x7C

#define MCP_STATICREAD		0x01
#define MCP_WRITE			0x02
#define MCP_READ			0x03
//ADC registers
#define MCP_MUX				0x6 << 2
#define MCP_CONF0			0x1 << 2
#define MCP_CONF1			0x2 << 2
#define MCP_CONF2			0x3 << 2
#define MCP_CONF3			0x4 << 2
#define MCP_DATA			0x0 << 2
#define MCP_IRQ				0x5 << 2
#define MCP_OFFS			0x9 << 2
#define MCP_GAINCAL			0xA << 2

#define SAMPLES             2048            /* 1024 real parts and 1024 imaginary parts */
#define FFT_SIZE            SAMPLES / 2     /* FFT size is always the same size as we have samples, so 512 in our case */

/* ticks measurement */
#define ARM_CM_DEMCR      (*(uint32_t *)0xE000EDFC)
#define ARM_CM_DWT_CTRL   (*(uint32_t *)0xE0001000)
#define ARM_CM_DWT_CYCCNT (*(uint32_t *)0xE0001004)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* Definitions for Accelerometer */
osThreadId_t AccelerometerHandle;
const osThreadAttr_t Accelerometer_attributes = {
  .name = "Accelerometer",
  .stack_size = 12288 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Humidity */
osThreadId_t HumidityHandle;
const osThreadAttr_t Humidity_attributes = {
  .name = "Humidity",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Temperature */
osThreadId_t TemperatureHandle;
const osThreadAttr_t Temperature_attributes = {
  .name = "Temperature",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for UART */
osThreadId_t UARTHandle;
const osThreadAttr_t UART_attributes = {
  .name = "UART",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* USER CODE BEGIN PV */
arm_rfft_fast_instance_f32 S;
arm_cfft_radix4_instance_f32 L;    /* ARM CFFT module */

uint8_t SPI_trans[5];
uint8_t SPI_recv[5];

uint8_t I2C_recv[10];
uint8_t I2C_trans[6];

HAL_StatusTypeDef ret;
uint8_t I2C_lock = 0;

uint32_t start;
uint32_t stop;
uint32_t delta;

//Variables to be sent
uint8_t humAlarm = 0;
float freq;
float ampAvg;
float ampMax;
double temp;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
void StartAccelerometer(void *argument);
void StartHumidity(void *argument);
void StartTemperature(void *argument);
void StartUART(void *argument);

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
  //Initialize DSP functions
  arm_rfft_fast_init_f32(&S, FFT_SIZE);
  /* Initialize the CFFT/CIFFT module, intFlag = 0, doBitReverse = 1 */
  arm_cfft_radix4_init_f32(&L, FFT_SIZE, 0, 1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_Delay(500);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Accelerometer */
  AccelerometerHandle = osThreadNew(StartAccelerometer, NULL, &Accelerometer_attributes);

  /* creation of Humidity */
  HumidityHandle = osThreadNew(StartHumidity, NULL, &Humidity_attributes);

  /* creation of Temperature */
  TemperatureHandle = osThreadNew(StartTemperature, NULL, &Temperature_attributes);

  /* creation of UART */
  UARTHandle = osThreadNew(StartUART, NULL, &UART_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_5);
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, thermistorSwitch_Pin|CS_Pin|RTS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : thermistorSwitch_Pin */
  GPIO_InitStruct.Pin = thermistorSwitch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(thermistorSwitch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RTS_Pin */
  GPIO_InitStruct.Pin = RTS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RTS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
float complexABS(float real, float compl) {
	return sqrtf((real*real)+(compl*compl));
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartAccelerometer */
/**
  * @brief  Function implementing the Accelerometer thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartAccelerometer */
void StartAccelerometer(void *argument)
{
  /* USER CODE BEGIN 5 */
  int16_t rawXaxis[FFT_SIZE];
  int16_t rawYaxis[FFT_SIZE];
  int16_t rawZaxis[FFT_SIZE];

  int16_t maxValueAxis;

  float32_t Input[SAMPLES];
  float32_t Output[FFT_SIZE];
  uint16_t x = 0;

  uint32_t maxIndex;
  float32_t maxValue;

  uint32_t start = 0;
  uint32_t delta = 0;
  uint32_t stop = 0;
  /* Infinite loop */
  for(;;)
  {
	if(I2C_lock == 0){
		//X axis
//	  	I2C_trans[0] = LIS2_OUTXH;
//		if((ret = HAL_I2C_Master_Transmit(&hi2c1, LIS2_ADDR, &I2C_trans[0], 1, HAL_MAX_DELAY)) != HAL_OK){
//			//error handler
//			do{
//				vTaskDelay(1000);
//				ret = HAL_I2C_Master_Transmit(&hi2c1, LIS2_ADDR, &I2C_trans[0], 1, HAL_MAX_DELAY);
//			}while(ret != HAL_OK);
//		}
//		if((ret = HAL_I2C_Master_Receive(&hi2c1, LIS2_ADDR, &I2C_recv[0], 1, HAL_MAX_DELAY)) != HAL_OK){
//			//error handler
//			for(;;);
//		}
//		rawXaxis[x] = I2C_recv[0] << 8;
//
//		I2C_trans[0] = LIS2_OUTXL;
//		if((ret = HAL_I2C_Master_Transmit(&hi2c1, LIS2_ADDR, &I2C_trans[0], 1, HAL_MAX_DELAY)) != HAL_OK){
//			for(;;);
//		}
//		if((ret = HAL_I2C_Master_Receive(&hi2c1, LIS2_ADDR, &I2C_recv[0], 1, HAL_MAX_DELAY)) != HAL_OK){
//			for(;;);
//		}
//		rawXaxis[x] = (rawXaxis[x] | I2C_recv[0])/4;

		//Y axis
//		I2C_trans[0] = LIS2_OUTYH;
//		if((ret = HAL_I2C_Master_Transmit(&hi2c1, LIS2_ADDR, &I2C_trans[0], 1, HAL_MAX_DELAY)) != HAL_OK){
//			//error handler
//			for(;;);
//		}
//		if((ret = HAL_I2C_Master_Receive(&hi2c1, LIS2_ADDR, &I2C_recv[0], 1, HAL_MAX_DELAY)) != HAL_OK){
//			//error handler
//			for(;;);
//		}
//		rawYaxis[x] = I2C_recv[0] << 8;
//
//		I2C_trans[0] = LIS2_OUTYL;
//		if((ret = HAL_I2C_Master_Transmit(&hi2c1, LIS2_ADDR, &I2C_trans[0], 1, HAL_MAX_DELAY)) != HAL_OK){
//			//error handler
//			for(;;);
//		}
//		if((ret = HAL_I2C_Master_Receive(&hi2c1, LIS2_ADDR, &I2C_recv[0], 1, HAL_MAX_DELAY)) != HAL_OK){
//			//error handler
//			for(;;);
//		}
//		rawYaxis[x] = (rawYaxis[x] | I2C_recv[0])/4;

		//Z axis
		I2C_trans[0] = LIS2_OUTZH;
		if((ret = HAL_I2C_Master_Transmit(&hi2c1, LIS2_ADDR, &I2C_trans[0], 1, HAL_MAX_DELAY)) != HAL_OK){
			//error handler
			do{
				vTaskDelay(1000);
				ret = HAL_I2C_Master_Transmit(&hi2c1, LIS2_ADDR, &I2C_trans[0], 1, HAL_MAX_DELAY);
			}while(ret != HAL_OK);
		}
		if((ret = HAL_I2C_Master_Receive(&hi2c1, LIS2_ADDR, &I2C_recv[0], 1, HAL_MAX_DELAY)) != HAL_OK){
			//error handler
			for(;;);
		}
		rawZaxis[x] = I2C_recv[0] << 8;

		I2C_trans[0] = LIS2_OUTZL;
		if((ret = HAL_I2C_Master_Transmit(&hi2c1, LIS2_ADDR, &I2C_trans[0], 1, HAL_MAX_DELAY)) != HAL_OK){
			//error handler
			for(;;);
		}
		if((ret = HAL_I2C_Master_Receive(&hi2c1, LIS2_ADDR, &I2C_recv[0], 1, HAL_MAX_DELAY)) != HAL_OK){
			//error handler
			for(;;);
		}
		rawZaxis[x] = (rawZaxis[x] | I2C_recv[0])/4;//14 bit divide by 4, 12 bit divide by 16
		x++;

		start = ARM_CM_DWT_CYCCNT;
		delta = start - stop;
		stop = start;
		if(x > FFT_SIZE-1){
			maxValueAxis = 0;
		    // Storing the largest number
		    for (int i = 1; i < x; i++) {
		    	if(maxValueAxis < rawZaxis[i]){
		    		maxValueAxis = rawZaxis[i];
		    	}
		    }
		    // Calculate amplitude in G with acceleration of z axis with 4G scale and 14 bit data length
		    ampMax = (maxValueAxis * 0.488)/1000;
			x = 0;

			// DSP functionality
	        for(int i = 0; i < SAMPLES; i += 2){
	            /* Real part */
	            Input[(uint16_t)i] = (float32_t)rawZaxis[x];
	            /* Imaginary part */
	            Input[(uint16_t)(i + 1)] = 0;
	            x++;
	        }
	        x = 0;
	        /* Process the data through the CFFT/CIFFT module */
	        arm_cfft_radix4_f32(&L, Input);
	        /* Process the data through the Complex Magnitude Module for calculating the magnitude at each bin */
	        arm_cmplx_mag_f32(Input, Output, FFT_SIZE);
	        Output[0] = 0;
	        /* Calculates maxValue and returns corresponding value */
	        arm_max_f32(Output, FFT_SIZE, &maxValue, &maxIndex);
	        //calculate frequency.
	        if(maxIndex >= 512){
	        	maxIndex = 0;
	        }
	        freq = (500/(float)(FFT_SIZE)) * (float)maxIndex;
	        x = 0;
		}
	}
    osDelay(2);
  }
  // In case we accidentally leave loop
  osThreadTerminate(NULL);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartHumidity */
/**
* @brief Function implementing the Humidity thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHumidity */
void StartHumidity(void *argument)
{
  /* USER CODE BEGIN StartHumidity */
  uint16_t val;

  float hum_rh;
  float temp_c;
  /* Infinite loop */
  for(;;)
  {
	I2C_trans[0] = 0x2C;
	I2C_trans[1] = 0x10;

	I2C_lock = 1;
	if((ret = HAL_I2C_Master_Transmit(&hi2c1, SHT31_ADDR, I2C_trans, 2, HAL_MAX_DELAY)) != HAL_OK){
		//error handler
	}
	if((ret = HAL_I2C_Master_Receive(&hi2c1, SHT31_ADDR, I2C_recv, 10, HAL_MAX_DELAY)) != HAL_OK){
		//error handler
	}
	I2C_lock = 0;
	val = I2C_recv[0] << 8 | I2C_recv[1];
	temp_c = -45+(175*(val/((pow(2,16))-1)));
	val = 0;
	val = I2C_recv[3] << 8 | I2C_recv[4];
	hum_rh = 100*(val/((pow(2,16))-1));
	if(hum_rh >= 75){
		humAlarm = 1;
	}
    osDelay(100);
  }
  // In case we accidentally leave loop
  osThreadTerminate(NULL);
  /* USER CODE END StartHumidity */
}

/* USER CODE BEGIN Header_StartTemperature */
/**
* @brief Function implementing the Temperature thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTemperature */
void StartTemperature(void *argument)
{
  /* USER CODE BEGIN StartTemperature */
  int32_t val = 0;
//  float tempTemp = 0;
//  uint8_t tempCounter = 0;
  float voltage = 0;
  float voltageDif = 0;
  float voltageFix = VOLTAGE/2;
  float resistance = 0;
  /* Infinite loop */
  for(;;)
  {
	val = 0;
	voltage = 0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	vTaskDelay(500);
	//Read ADC
	SPI_trans[0] =  MCP_ADDR | MCP_DATA | MCP_STATICREAD;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	if((ret = HAL_SPI_Transmit(&hspi1, SPI_trans, 1, HAL_MAX_DELAY)) != HAL_OK){
		//error handler
		while(1);
	}
	for(int i = 0; i <= 255; i++);
	if((ret = HAL_SPI_Receive(&hspi1, SPI_recv, 3, HAL_MAX_DELAY)) != HAL_OK){
		//error handler
		while(1);
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	//get voltage and resistance
	val = (SPI_recv[0] << 24 | SPI_recv[1] << 18 | SPI_recv[2] << 8)/256;
	voltageDif = (VOLTAGE*val)/8388608;
	voltage = voltageDif + voltageFix;
	resistance = (voltage*RESISTANCE)/(VOLTAGE-voltage);
	//Steinhart and Hart approach
	temp = (1/(A + (B*log(resistance)) + (C*pow(log(resistance),2)) + (D*pow(log(resistance),3)))) - KELVIN;
	vTaskDelay(300);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    osDelay(500);
  }
  // In case we accidentally leave loop
  osThreadTerminate(NULL);
  /* USER CODE END StartTemperature */
}

/* USER CODE BEGIN Header_StartUART */
/**
* @brief Function implementing the UART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUART */
void StartUART(void *argument)
{
  /* USER CODE BEGIN StartUART */
  char UART_buf[50];
  /* Infinite loop */
  for(;;)
  {
	sprintf((char *)UART_buf, "%6.2f,%6.2f,%6.2f,%d", ampMax, freq, temp, humAlarm);
	HAL_UART_Transmit(&huart1, UART_buf, strlen(UART_buf), HAL_MAX_DELAY);
	humAlarm = 0;
	osDelay(500);
  }
  // In case we accidentally leave loop
  osThreadTerminate(NULL);
  /* USER CODE END StartUART */
}

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
