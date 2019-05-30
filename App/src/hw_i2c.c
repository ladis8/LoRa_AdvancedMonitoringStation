/*!
 * \file		hw_i2c.h
 *
 * \brief		I2C API - implementation of functions handling with I2C
 *
 * \copyright
 *
 * \author		Ladislav Stefka
 *
 */

#include "hw.h"
#include "utilities.h"


static I2C_HandleTypeDef hi2c;

void HW_I2C_Init()
{

	hi2c.Instance              = I2Cx;
	hi2c.Init.Timing           = I2C_TIMING;
	hi2c.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
	hi2c.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
	hi2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
	hi2c.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
	hi2c.Init.OwnAddress1      = 0;
	hi2c.Init.OwnAddress2      = 0;

	/*##-1- Configure the I2C clock source. The clock is derived from the SYSCLK #*/

	RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;
	RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
	RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
	HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);


	if(HAL_I2C_Init(&hi2c) != HAL_OK)
		Error_Handler();

	//The function call can not be here!!!
	//__HAL_RCC_I2C1_CLK_ENABLE();

}

void HW_I2C_DeInit()
{
  HAL_I2C_DeInit(&hi2c);
  __HAL_RCC_I2C1_FORCE_RESET();
  __HAL_RCC_I2C1_RELEASE_RESET();

}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;

	/*##-1- Configure the I2C clock source. The clock is derived from the SYSCLK #*/
	RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
	RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
	HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

	/*##-2- Enable peripherals and GPIO Clocks #################################*/
	//__GPIOA_CLK_ENABLE();
	I2Cx_GPIO_CLK_ENABLE();


	GPIO_InitStruct.Pin       = I2Cx_SCL_PIN | I2Cx_SDA_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = I2Cx_AF;

	HAL_GPIO_Init(I2Cx_GPIOPORT, &GPIO_InitStruct);

	//the function call must be here!!!
	I2Cx_CLK_ENABLE();




	/*##-6- Configure the NVIC for I2C ########################################*/
	/* NVIC for I2Cx */
	//HAL_NVIC_SetPriority(I2C1_IRQn, 0, 1);
	//HAL_NVIC_EnableIRQ(I2C1_IRQn);
}



bool HW_I2C_MemRead(uint16_t devAdr, uint16_t regAdr, uint8_t *buff, uint16_t size)
{
	HAL_StatusTypeDef status =  HAL_I2C_Mem_Read(&hi2c, devAdr, regAdr, 1, buff, size,  I2C_TIMEOUT);

	if (status != HAL_OK){
		//error
		return false;
	}
	return true;
}


bool HW_I2C_MemWrite(uint16_t devAdr, uint16_t regAdr, uint8_t *buff, uint16_t size){
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c, devAdr, regAdr, 1, buff, size, I2C_TIMEOUT);
	if (status != HAL_OK){
		//error
		return false;
	}
	return true;
}




bool scanI2C1()
{

	bool someDevice = false;

	PRINTF("Scanning I2C bus...\r\n");
	HAL_StatusTypeDef result;
	for (int i=1; i<128; i++){
		//the HAL wants a left aligned i2c address
		result = HAL_I2C_IsDeviceReady(&hi2c, (uint16_t)(i<<1), 2, 2);
		if (result != HAL_OK) // HAL_ERROR or HAL_BUSY or HAL_TIMEOUT{
			PRINTF(".");
		if (result == HAL_OK){
			PRINTF("\r\nDevice on address: 0x%X\r\n", i);
			someDevice = true;
		}

	}
	PRINTF("\r\n");
	return someDevice;

}
