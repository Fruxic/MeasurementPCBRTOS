/*
 * variablesMeas.h
 *
 *  Created on: Jan 17, 2023
 *      Author: Q6Q
 */

#ifndef INC_VARIABLESMEAS_H_
#define INC_VARIABLESMEAS_H_

extern unsigned char SPI_trans[5];
extern unsigned char SPI_recv[5];

extern unsigned char I2C_recv[10];
extern unsigned char I2C_trans[6];

extern unsigned char I2C_lock;

extern unsigned long start;
extern unsigned long stop;
extern unsigned long delta;

//Variables to be sent
extern unsigned char humAlarm;
extern float freq;
extern float ampAvg;
extern float ampMax;
extern float temp;

#endif /* INC_VARIABLESMEAS_H_ */
