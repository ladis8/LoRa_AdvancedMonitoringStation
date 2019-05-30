/*!
 * \file		hw_i2c.h
 *
 * \brief		I2C API - definition of functions handling with I2c
 *
 * \copyright
 *
 * \author		Ladislav Stefka
 *
 */

#ifndef __HW_I2C_H__
#define __HW_I2C_H__



#define I2C_TIMING  0x009080B5
#define I2C_TIMEOUT 100

#include <stdbool.h>


/* Initialize I2C1 */
void HW_I2C_Init();

/* Deinitialize I2C1 */
void HW_I2C_DeInit();

/* Initialize I2C1 pins */
void HW_I2C_IoInit ();

/* Read memory from I2C slave */
bool HW_I2C_MemRead(uint16_t devAdr, uint16_t regAdr, uint8_t *buff, uint16_t size);

/* Write to memory of I2C slave */
bool HW_I2C_MemWrite(uint16_t devAdr, uint16_t regAdr, uint8_t *buff, uint16_t size);

/* Scan I2C bus */
bool scanI2C1();



#endif
