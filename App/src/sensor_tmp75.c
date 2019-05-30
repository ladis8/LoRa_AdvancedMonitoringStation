/*!
 * \file		sensor_tmp75.c
 *
 * \brief		Sensor API - implementation of functions handling with sensor TMP75
 *
 * \copyright
 *
 * \author		Ladislav Stefka
 *
 */



#include "sensor_tmp75.h"

static uint8_t tempBuff[2];

void TMP75_Init(){

    uint8_t value = TMP75_resolution;
    TMP75_WriteRegister (TMP75_configreg, &value, 1);
}


bool TMP75_WriteRegister(uint8_t regAddr, uint8_t *value, uint16_t size){
	return HW_I2C_MemWrite(TMP75_writeaddress, regAddr, value, size);
}

bool TMP75_ReadRegister(uint8_t regAddr, uint8_t *value, uint16_t size){
	return HW_I2C_MemRead(TMP75_readaddress, regAddr, value, size);
}

uint16_t TMP75_GetTemperature(){

	bool status = TMP75_ReadRegister (TMP75_tempreg, tempBuff, 2);
	if (!status){
		//What return if fail
		//return ;
	}
	// From Datasheet the TMP75 has a quantisation value of 0.0625 degreesC per bit
	uint16_t temp_shifted = (((tempBuff[0] << 8) | tempBuff[1]) >> 4);
	return temp_shifted;
}


float32_t TMP75_GetTemperatureFloat(){
	//return TMP75_GetTemperature() * calibrationConstant;
	return TMP75_GetTemperature() * 0.0625;
}
