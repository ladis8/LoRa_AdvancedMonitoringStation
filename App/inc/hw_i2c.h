/*
 * hw_i2c.h
 *
 *  Created on: Dec 28, 2018
 *      Author: ladislav
 */

#ifndef PROJECTS_PINGPONG_HW_I2C_H_
#define PROJECTS_PINGPONG_HW_I2C_H_



#define I2C_TIMING  0x009080B5
#define I2C_TIMEOUT 100

#include <stdbool.h>



void HW_I2C_Init();
void HW_I2C_DeInit();
void HW_I2C_IoInit ();

bool HW_I2C_MemRead(uint16_t devAdr, uint16_t regAdr, uint8_t *buff, uint16_t size);

bool HW_I2C_MemWrite(uint16_t devAdr, uint16_t regAdr, uint8_t *buff, uint16_t size);

bool scanI2C1();



#endif /* PROJECTS_PINGPONG_HW_I2C_H_ */
