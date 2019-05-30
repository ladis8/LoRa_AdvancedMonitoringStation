/*!
 * \file		hw_adc.h
 *
 * \brief		ADC API - definition of functions handling with ADC
 *
 * \copyright
 *
 * \author		Ladislav Stefka
 *
 */

#ifndef __HW_ADC_H__
#define __HW_ADC_H__

#include <stdbool.h>
#include <stdint.h>



#define VREFINT_CAL       ((uint16_t*) ((uint32_t) 0x1FF80078))
#define LORAWAN_MAX_BAT   254


/* Handler for DMA ADC Interrupt */
void ADC_DMA_IRQHandler(void);

/* Initialize ADC in normal single conversion mode */
void HW_ADC_Init();

/* Initialize ADC in normal multiple (DMA) conversion mode */
void HW_ADC_Init_Con();

/* Deinitialize ADC */
void HW_ADC_DeInit( void );

/* Read ADC channel in single conversion mode */
uint16_t HW_ADC_ReadChannel( uint32_t channel);

/* Read ADC channel in multiple conversion mode */
void HW_ADC_Read_Continuous(uint16_t* ADCReadings, uint32_t ADCReadingsLength);

/* Read ADC channel for battery */
uint8_t HW_GetBatteryLevel(void);

bool isConversionFinished();

#endif
