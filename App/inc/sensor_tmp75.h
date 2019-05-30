/*!
 * \file		sensor_tmp75.h
 *
 * \brief		Sensor API - definition of functions handling with sensor TMP75 and default constants
 *
 * \copyright
 *
 * \author		Ladislav Stefka
 *
 */

#ifndef __SENSOR_TMP75_H__
#define __SENSOR_TMP75_H__



#define TMP75_resolution 0b01100000
#define TMP75_writeaddress 0x90
#define TMP75_readaddress 0x91
#define TMP75_configreg 0x01
#define TMP75_tempreg 0x00



#include <stdint.h>
#include <stdbool.h>
#include "hw_i2c.h"
#include "arm_math.h"

static float32_t calibrationConstant = 0.0625;

/* Initialize sensor TMP75 */
void TMP75_Init();

/* Write to TMP75 register */
bool TMP75_WriteRegister(uint8_t regAddr, uint8_t* value, uint16_t size);

/* Read from TMP75 register */
bool TMP75_ReadRegister(uint8_t regAddr, uint8_t* value, uint16_t size);

/* Get temperature from TMP75 as uint16_t */
uint16_t TMP75_GetTemperature();

/* Get temperature from TMP75 as float32_t */
float32_t TMP75_GetTemperatureFloat();

#endif
