/*
 * variablesMeas.c
 *
 *  Created on: Jan 17, 2023
 *      Author: Q6Q
 */

unsigned char SPI_trans[5];
unsigned char SPI_recv[5];

unsigned char I2C_recv[10];
unsigned char I2C_trans[6];

unsigned char I2C_lock = 0;

unsigned long start;
unsigned long stop;
unsigned long delta;

//Variables to be sent
unsigned char humAlarm = 0;
float freq;
float ampAvg;
float ampMax;
float temp;
