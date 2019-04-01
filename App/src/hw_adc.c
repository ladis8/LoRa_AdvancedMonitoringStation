/*
 * hw_adc.c
 *
 *  Created on: Feb 11, 2019
 *      Author: ladislav
 */


#include "hw_adc.h"

static bool adcInit = false;
static ADC_HandleTypeDef hadc;
static DMA_HandleTypeDef hdma_adc;


 __IO ITStatus ADCDMAFinished = RESET;

void HAL_ADC_ConvCpltCallback( ADC_HandleTypeDef *hadc){
	ADCDMAFinished = SET;
}


void ADC_DMA_IRQHandler(){
	HAL_DMA_IRQHandler(hadc.DMA_Handle);
}

void HW_ADC_Init( void )
{
  if( adcInit == false )
  {
    adcInit = true;

    hadc.Instance  = ADCx;

    hadc.Init.OversamplingMode      = DISABLE;

    hadc.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc.Init.LowPowerAutoPowerOff  = DISABLE;
    hadc.Init.LowPowerFrequencyMode = ENABLE;
    hadc.Init.LowPowerAutoWait      = DISABLE;

    hadc.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc.Init.SamplingTime          = ADC_SAMPLETIME_160CYCLES_5;
    hadc.Init.ScanConvMode          = ADC_SCAN_DIRECTION_FORWARD;
    hadc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc.Init.ContinuousConvMode    = DISABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc.Init.DMAContinuousRequests = DISABLE;

    ADC_CLK_ENABLE();

    HAL_ADC_Init( &hadc );

  }
}

void HW_ADC_Init_Con (void)
{
	if( adcInit == false )
	{
		adcInit = true;
		//sample time == [12.5 + 160.5] * 1/8 *10^(-6) == 21.625us
		//fs === 46.242 kHz
		//45.1582 Hz

		hadc.Instance = ADCx;
		hadc.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;		//8 Mhz clock
		hadc.Init.Resolution 			= ADC_RESOLUTION_12B;
		hadc.Init.SamplingTime          = ADC_SAMPLETIME_160CYCLES_5;
		//hadc.Init.ScanConvMode          = ADC_SCAN_ENABLE;
		hadc.Init.ScanConvMode 			= ADC_SCAN_DIRECTION_FORWARD; //Sequencer disabled (ADC conversion on only 1 channel
		hadc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;        //EOC flag picked-up to indicate conversion end
		hadc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
		hadc.Init.ContinuousConvMode    = ENABLE;
		hadc.Init.DiscontinuousConvMode = DISABLE;
		hadc.Init.ExternalTrigConvEdge  = ADC_SOFTWARE_START;
		hadc.Init.DMAContinuousRequests = DISABLE;

		//hadc.Init.NbrOfDiscConversion = 0;
		//hadc.Init.NbrOfConversion = 	ADC_SAMPLES;

		if (HAL_ADC_Init(&hadc) != HAL_OK)
			Error_Handler();


		ADC_ChannelConfTypeDef adcConf;
		/* Deselects all channels*/
		adcConf.Channel = ADC_CHANNEL_MASK;
		adcConf.Rank = ADC_RANK_NONE;
		HAL_ADC_ConfigChannel( &hadc, &adcConf);

		/* configure adc channel */
		adcConf.Channel = ADC_READ_CHANNEL;
		adcConf.Rank = ADC_RANK_CHANNEL_NUMBER;
		HAL_ADC_ConfigChannel( &hadc, &adcConf);



	}

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
	 GPIO_InitTypeDef GPIO_InitStruct;

	 //1. allow gpio, adc, dma clock
	 ADC_GPIO_CLK_ENABLE();
	 ADC_CLK_ENABLE();
	 DMAx_CLK_ENABLE();

	 //2. set gpio pin
	 GPIO_InitStruct.Pin = ADC_PIN;
	 GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	 GPIO_InitStruct.Pull = GPIO_NOPULL;
	 HAL_GPIO_Init(ADC_GPIOPORT, &GPIO_InitStruct);


	//3. set DMA init

	hdma_adc.Instance = ADC_DMA_CHANNEL;
	hdma_adc.Init.Request = ADC_DMA_REQUEST;
	hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
	hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma_adc.Init.Mode = DMA_CIRCULAR;
	hdma_adc.Init.Priority = DMA_PRIORITY_LOW;

	if (HAL_DMA_Init(&hdma_adc) != HAL_OK)
		Error_Handler();

	__HAL_LINKDMA(hadc, DMA_Handle, hdma_adc);

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(ADC_DMA_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ADC_DMA_IRQn);
}

void HW_ADC_Read_Continuous(uint16_t* ADCReadings, uint32_t ADCReadingsLength)
{
	if(adcInit){

	    /* wait the the Vrefint used by adc is set */
	    while (__HAL_PWR_GET_FLAG(PWR_FLAG_VREFINTRDY) == RESET) {};

	    ADC_CLK_ENABLE();

	    /*calibrate ADC if any calibraiton hardware*/
	    if (HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK)
	    	Error_Handler();

	    ADCDMAFinished = RESET;
	    HAL_ADC_Start_DMA(&hadc, (uint32_t*) ADCReadings, ADCReadingsLength);

	}
	 else {
		  PRINTF("ERROR: ADC has not been initialized yet!\r\n");
	  }
}

bool isConversionFinished(){
	return (ADCDMAFinished == SET)? true: false;
}


void HW_ADC_DeInit( void )
{
	 HAL_ADC_DeInit(&hadc);
	 ADC_CLK_DISABLE();
	 adcInit = false;
}


uint16_t HW_ADC_ReadChannel( uint32_t channel )
{

  ADC_ChannelConfTypeDef adcConf;
  uint16_t adcData = 0;

  if(adcInit){

    /* wait the the Vrefint used by adc is set */
    while (__HAL_PWR_GET_FLAG(PWR_FLAG_VREFINTRDY) == RESET) {};

    ADC_CLK_ENABLE();

    /*calibrate ADC if any calibraiton hardware*/
    HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED );

    /* Deselects all channels*/
    adcConf.Channel = ADC_CHANNEL_MASK;
    adcConf.Rank = ADC_RANK_NONE;
    HAL_ADC_ConfigChannel( &hadc, &adcConf);

    /* configure adc channel */
    adcConf.Channel = channel;
    adcConf.Rank = ADC_RANK_CHANNEL_NUMBER;
    HAL_ADC_ConfigChannel( &hadc, &adcConf);

    /* Start the conversion process */
    HAL_ADC_Start( &hadc);

    /* Wait for the end of conversion */
    HAL_ADC_PollForConversion( &hadc, HAL_MAX_DELAY );

    /* Get the converted value of regular channel */
    adcData = HAL_ADC_GetValue ( &hadc);

    __HAL_ADC_DISABLE( &hadc) ;

    ADC_CLK_DISABLE();
  }
  else {
	  PRINTF("ERROR: ADC has not been initialized yet!\r\n");
  }
  return adcData;
}


/**
  * @brief This function return the battery level
  * @param none
  * @retval the battery level  1 (very low) to 254 (fully charged)
  */
uint8_t HW_GetBatteryLevel( void )
{
  uint8_t batteryLevel = 0;
  uint16_t measuredLevel = 0;
  uint32_t batteryLevelmV;

  measuredLevel = HW_ADC_ReadChannel( ADC_CHANNEL_VREFINT );

  if (measuredLevel == 0)
  {
    batteryLevelmV = 0;
  }
  else
  {
    batteryLevelmV= (( (uint32_t) VDDA_VREFINT_CAL * (*VREFINT_CAL ) )/ measuredLevel);
  }

  if (batteryLevelmV > VDD_BAT)
  {
    batteryLevel = LORAWAN_MAX_BAT;
  }
  else if (batteryLevelmV < VDD_MIN)
  {
    batteryLevel = 0;
  }
  else
  {
    batteryLevel = (( (uint32_t) (batteryLevelmV - VDD_MIN)*LORAWAN_MAX_BAT) /(VDD_BAT-VDD_MIN) );
  }
  return batteryLevel;
}
