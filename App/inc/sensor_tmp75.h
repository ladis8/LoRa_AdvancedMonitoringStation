/*
 * sensor_tmp75.h
 *
 *  Created on: Dec 28, 2018
 *      Author: ladislav
 */

#ifndef PROJECTS_SENSOR_TMP75_H_
#define PROJECTS__SENSOR_TMP75_H_



#define TMP75_resolution 0b01100000
#define TMP75_writeaddress 0x90
#define TMP75_readaddress 0x91
#define TMP75_configreg 0x01
#define TMP75_tempreg 0x00



#include <stdint.h>
#include <stdbool.h>
#include "hw_i2c.h"
#include "arm_math.h"

//static float32_t calibrationConstant = 0.0625;

void TMP75_Init();
bool TMP75_WriteRegister(uint8_t regAddr, uint8_t* value, uint16_t size);
bool TMP75_ReadRegister(uint8_t regAddr, uint8_t* value, uint16_t size);

uint16_t TMP75_GetTemperature();
float32_t TMP75_GetTemperatureFloat();

#endif
