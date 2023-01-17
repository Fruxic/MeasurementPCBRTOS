/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "definesMeas.h"
#include "variablesMeas.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
arm_rfft_fast_instance_f32 S;
arm_cfft_radix4_instance_f32 L;    /* ARM CFFT module */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartAccelerometer(void *argument);
void StartHumidity(void *argument);
void StartTemperature(void *argument);
void StartUART(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  //Initialize DSP functions
  arm_rfft_fast_init_f32(&S, FFT_SIZE);
  /* Initialize the CFFT/CIFFT module, intFlag = 0, doBitReverse = 1 */
  arm_cfft_radix4_init_f32(&L, FFT_SIZE, 0, 1);
  /* USER CODE END Init */
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/**
  * @}
  */

/**
  * @}
  */
}

/* USER CODE BEGIN Header_StartAccelerometer */
/**
  * @brief  Function implementing the Accelerometer thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartAccelerometer */
void StartAccelerometer(void *argument)
{
  /* USER CODE BEGIN StartAccelerometer */
  int16_t rawXaxis[FFT_SIZE];
  int16_t rawYaxis[FFT_SIZE];
  int16_t rawZaxis[FFT_SIZE];

  int16_t maxValueAxis;

  float32_t Input[SAMPLES];
  float32_t Output[FFT_SIZE];
  uint16_t x = 0;

  uint32_t maxIndex;
  float32_t maxValue;
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
  /* USER CODE END StartAccelerometer */
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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

