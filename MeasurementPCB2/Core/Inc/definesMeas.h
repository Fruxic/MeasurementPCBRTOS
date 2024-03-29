/*
 * definesMeas.h
 *
 *  Created on: Jan 17, 2023
 *      Author: Q6Q
 */

#ifndef INC_DEFINESMEAS_H_
#define INC_DEFINESMEAS_H_

#define	ARM_MATH_CM4

#define CLOCK				100000000

#define RESOLUTION			4096
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

#define SAMPLES             2048            /* 1024 real parts and 1024 imaginary parts */
#define FFT_SIZE            SAMPLES / 2     /* FFT size is always the same size as we have samples, so 512 in our case */

#endif /* INC_DEFINESMEAS_H_ */
