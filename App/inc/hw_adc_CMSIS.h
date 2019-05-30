/*!
 * \file		hw_adc_CMSIS.h
 *
 * \brief		ADC API - definition of functions handling with ADC using CMSIS only
 *
 * \copyright
 *
 * \author		Ladislav Stefka
 *
 */

#ifndef __HW_ADC_CMSIS_H__
#define __PINGPONG_HW_ADC_CMSIS_H__

#include "stm32l073xx.h"

void HW_ADC_Init();
void HW_ADC_IoInit();
uint16_t readADCValue();


#endif
