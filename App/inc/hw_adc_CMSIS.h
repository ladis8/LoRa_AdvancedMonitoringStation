/*
 * hw_adc_CMSIS.h
 *
 *  Created on: Dec 29, 2018
 *      Author: ladislav
 */

#ifndef PROJECTS_PINGPONG_HW_ADC_CMSIS_H_
#define PROJECTS_PINGPONG_HW_ADC_CMSIS_H_

#include "stm32l073xx.h"

void HW_ADC_Init();
void HW_ADC_IoInit();
uint16_t readADCValue();


#endif /* PROJECTS_PINGPONG_HW_ADC_CMSIS_H_ */
