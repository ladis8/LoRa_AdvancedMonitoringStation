/*
 * hw_adc_CMSIS.c
 *
 *  Created on: Dec 29, 2018
 *      Author: ladislav
 */

#include "hw_adc_CMSIS.h"
void HW_ADC_Init(){

/* Switch on the clock for the ADC1 peripheral */
	//RCC->AHBENR |= RCC_AHBENR_ADC12EN;
	RCC-> APB2ENR |= RCC_APB2ENR_ADCEN;
	/* Select the APB prescaled by 2 as the clock source for the ADC */
	//ADC->CCR |= ADC12_CCR_CKMODE_0;
	ADC1-> CFGR2 |= ADC_CFGR2_CKMODE_0;
	/* Set auto-off mode --> reduce power consumption */


	//ADC1->CFGR1 |= ADC_CFGR1_AUTOFF;

	/* First of all, we have to enable the ADC internal voltage regulator */
	ADC1->CR |= ADC_CR_ADVREGEN;
	ADC1->CHSELR = ADC_CHSELR_CHSEL0;

	/* Set the sampling time for channel 1 to 160.5 ADC clock cycles */
	ADC1->SMPR |= (ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2);
	/* Set single conversion mode */
	ADC1->CFGR1 &= ~ADC_CFGR1_CONT;

	/* Wait approximately 10 us before enabling the ADC or starting calibration */
	delay_us(10);


	/* Run the calibration of ADC in single-ended mode to remove chip to chip
	 * varying ofsets.
	 * The output of calibration is stored in DR register and is added by HW */
	ADC1->CR &= ~ADC_CR_ADEN;
	ADC1->CR |= ADC_CR_ADCAL;
	/* Wait until the calibration is completed */
	while (ADC1->CR & ADC_CR_ADCAL);

	/* Now we can proceed to enable the ADC */
	ADC1->CR |= ADC_CR_ADEN;

	/* Wait until the ADC is ready */
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0);

}

void HW_ADC_IoInit(){
	//Define analog input pin
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	GPIOA->MODER &= ~GPIO_MODER_MODE0;
	GPIOA->MODER |= (3 << GPIO_MODER_MODE0_Pos);

}

uint16_t readADCValue (){

	ADC1->CR |= ADC_CR_ADSTART;
	/* wait until the sequence of conversions is done */
	while ((ADC1->ISR & ADC_ISR_EOC) == 0);

	ADC1->ISR &= ~ADC_ISR_EOC;

	/* Store the converted value into tho TIM_CCR1 register to adjust the PWM */
	return ADC1->DR;
}
