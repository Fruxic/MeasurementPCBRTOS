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
#include "usart.h"
#include "adc.h"
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
osThreadId EmptyHandle;
osThreadId AccelerometerHandle;
osThreadId HumidityHandle;
osThreadId TemperatureHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartEmpty(void const * argument);
void startAccelerometer(void const * argument);
void startHumidity(void const * argument);
void startTemperature(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
	*ppxIdleTaskStackBuffer = &xIdleStack[0];
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
	/* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */
	/*initialize humidity module (I2C)*/
	//Check if device is connected
	if((ret = HAL_I2C_IsDeviceReady(&hi2c2, SHT31_ADDR, 1, HAL_MAX_DELAY)) != HAL_OK){
		//error handler
		NVIC_SystemReset();
		for(;;);
	}
	//Disable heater
	I2C_trans[0] = SHT31_HEATER_First;
	I2C_trans[1] = SHT31_HEATER_Second;
	if((ret = HAL_I2C_Master_Transmit(&hi2c2, SHT31_ADDR, I2C_trans, 2, HAL_MAX_DELAY)) != HAL_OK){
		//error handler
		NVIC_SystemReset();
		for(;;);
	}
	/*Initialize accelerometer module (I2C)*/
	//check if device is connected
	if((ret = HAL_I2C_IsDeviceReady(&hi2c1, LIS2_ADDR, 1, HAL_MAX_DELAY)) != HAL_OK){
		//error handler
		NVIC_SystemReset();
		for(;;);
	}
	//Write setting to control register 1
	I2C_trans[0] = LIS2_CTRL1_ADDR;
	I2C_trans[1] = LIS2_CTRL1_Write;
	if((ret = HAL_I2C_Master_Transmit(&hi2c1, LIS2_ADDR, I2C_trans, 2, HAL_MAX_DELAY)) != HAL_OK){
		//error handler
		NVIC_SystemReset();
		for(;;);
	}
	//Write setting to FIFO register
	I2C_trans[0] = LIS2_FIFO_ADDR;
	I2C_trans[1] = LIS2_FIFO_Write;
	if((ret = HAL_I2C_Master_Transmit(&hi2c1, LIS2_ADDR, I2C_trans, 2, HAL_MAX_DELAY)) != HAL_OK){
		//error handler
		NVIC_SystemReset();
		for(;;);
	}

	//Initialize DSP functions
	arm_rfft_fast_init_f32(&S, FFT_SIZE);
	/* Initialize the CFFT/CIFFT module, intFlag = 0, doBitReverse = 1 */
	arm_cfft_radix4_init_f32(&L, FFT_SIZE, 0, 1);
	/* USER CODE END Init */

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
	/* definition and creation of Empty */
	osThreadDef(Empty, StartEmpty, osPriorityIdle, 0, 128);
	EmptyHandle = osThreadCreate(osThread(Empty), NULL);

	/* definition and creation of Accelerometer */
	osThreadDef(Accelerometer, startAccelerometer, osPriorityRealtime, 0, 12800);
	AccelerometerHandle = osThreadCreate(osThread(Accelerometer), NULL);

	/* definition and creation of Humidity */
	osThreadDef(Humidity, startHumidity, osPriorityLow, 0, 512);
	HumidityHandle = osThreadCreate(osThread(Humidity), NULL);

	/* definition and creation of Temperature */
	osThreadDef(Temperature, startTemperature, osPriorityNormal, 0, 512);
	TemperatureHandle = osThreadCreate(osThread(Temperature), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartEmpty */
/**
 * @brief  Function implementing the Empty thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartEmpty */
void StartEmpty(void const * argument)
{
	/* USER CODE BEGIN StartEmpty */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	// In case we accidentally leave loop
	osThreadTerminate(NULL);
	/* USER CODE END StartEmpty */
}

/* USER CODE BEGIN Header_startAccelerometer */
/**
 * @brief Function implementing the Accelerometer thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startAccelerometer */
void startAccelerometer(void const * argument)
{
	/* USER CODE BEGIN startAccelerometer */
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
	/* USER CODE END startAccelerometer */
}

/* USER CODE BEGIN Header_startHumidity */
/**
 * @brief Function implementing the Humidity thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startHumidity */
void startHumidity(void const * argument)
{
	/* USER CODE BEGIN startHumidity */
	uint16_t val;
	float hum_rh;
	float temp_c;
	/* Infinite loop */
	for(;;)
	{
		I2C_trans[0] = 0x2C;
		I2C_trans[1] = 0x10;

		I2C_lock = 1;
		if((ret = HAL_I2C_Master_Transmit(&hi2c2, SHT31_ADDR, I2C_trans, 2, HAL_MAX_DELAY)) != HAL_OK){
			//error handler
			for(;;);
		}
		if((ret = HAL_I2C_Master_Receive(&hi2c2, SHT31_ADDR, I2C_recv, 10, HAL_MAX_DELAY)) != HAL_OK){
			//error handler
			for(;;);
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
		osDelay(1000);
	}
	// In case we accidentally leave loop
	osThreadTerminate(NULL);
	/* USER CODE END startHumidity */
}

/* USER CODE BEGIN Header_startTemperature */
/**
 * @brief Function implementing the Temperature thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startTemperature */
void startTemperature(void const * argument)
{
	/* USER CODE BEGIN startTemperature */
	uint32_t rawValue;
	float voltage = 0;
	float resistance = 0;
	/* Infinite loop */
	for(;;)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
		vTaskDelay(100);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		rawValue = HAL_ADC_GetValue(&hadc1);
		voltage = (VOLTAGE*rawValue)/RESOLUTION;
		resistance = (voltage*RESISTANCE)/(VOLTAGE-voltage);
		temp = (1/(A + (B*log(resistance)) + (C*pow(log(resistance),2)) + (D*pow(log(resistance),3)))) - KELVIN;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
		osDelay(400);
	}
	// In case we accidentally leave loop
	osThreadTerminate(NULL);
	/* USER CODE END startTemperature */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
