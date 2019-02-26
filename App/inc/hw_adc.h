/*
 * hw_adc.h
 *
 *  Created on: Dec 28, 2018
 *      Author: ladislav
 */

#ifndef _HW_ADC_H_
#define _HW_ADC_H_

#include "hw.h"
#include "utilities.h"

#define VREFINT_CAL       ((uint16_t*) ((uint32_t) 0x1FF80078))
#define LORAWAN_MAX_BAT   254

void ADC_DMA_IRQHandler(void);

void HW_ADC_Init( void );
void HW_ADC_Init_Con( void );

void HW_ADC_DeInit( void );

uint16_t HW_ADC_ReadChannel( uint32_t channel );
void HW_ADC_Read_Continuous(uint16_t* ADCReadings, uint32_t ADCReadingsLength);

/*!
 * \brief Get the current battery level
 *
 * \retval value  battery level ( 0: very low, 254: fully charged )
 */
uint8_t HW_GetBatteryLevel(void);


#endif
