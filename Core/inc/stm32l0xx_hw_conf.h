#ifndef __HW_CONF_L0_H__
#define __HW_CONF_L0_H__

#ifdef __cplusplus
 extern "C" {
#endif


//LORA I/O definition
#define RADIO_RESET_PORT                          GPIOA
#define RADIO_RESET_PIN                           GPIO_PIN_0

#define RADIO_MOSI_PORT                           GPIOA
#define RADIO_MOSI_PIN                            GPIO_PIN_7

#define RADIO_MISO_PORT                           GPIOA
#define RADIO_MISO_PIN                            GPIO_PIN_6

#define RADIO_SCLK_PORT                           GPIOA
#define RADIO_SCLK_PIN                            GPIO_PIN_5

#define RADIO_NSS_PORT                            GPIOB
#define RADIO_NSS_PIN                             GPIO_PIN_6

#define RADIO_DIO_0_PORT                          GPIOA
#define RADIO_DIO_0_PIN                           GPIO_PIN_10

#define RADIO_DIO_1_PORT                          GPIOB
#define RADIO_DIO_1_PIN                           GPIO_PIN_3

#define RADIO_DIO_2_PORT                          GPIOB
#define RADIO_DIO_2_PIN                           GPIO_PIN_5

#define RADIO_DIO_3_PORT                          GPIOB
#define RADIO_DIO_3_PIN                           GPIO_PIN_4


#define RADIO_ANT_SWITCH_PORT                     GPIOC
#define RADIO_ANT_SWITCH_PIN                      GPIO_PIN_1


 //I2C definitions
#define I2Cx									  I2C1
#define I2Cx_GPIOPORT							  GPIOB
#define I2Cx_GPIO_CLK_ENABLE()					  __GPIOB_CLK_ENABLE()

#define I2Cx_SCL_PIN							  GPIO_PIN_8
#define I2Cx_SDA_PIN 							  GPIO_PIN_9

#define I2Cx_CLK_ENABLE()                		  __HAL_RCC_I2C1_CLK_ENABLE()
#define I2Cx_AF									  GPIO_AF4_I2C1


 /*  SPI MACRO redefinition */

#define SPI_CLK_ENABLE()                __HAL_RCC_SPI1_CLK_ENABLE()

#define SPI1_AF                          GPIO_AF0_SPI1  

/* ADC MACRO redefinition */

#ifdef USE_STM32L0XX_NUCLEO
#define ADC_READ_CHANNEL                 ADC_CHANNEL_4
#define ADCCLK_ENABLE()                 __HAL_RCC_ADC1_CLK_ENABLE() ;
#define ADCCLK_DISABLE()                __HAL_RCC_ADC1_CLK_DISABLE() ;
#endif


/* --------------------------- RTC HW definition -------------------------------- */

#define RTC_OUTPUT       DBG_RTC_OUTPUT

#define RTC_Alarm_IRQn              RTC_IRQn
/* --------------------------- USART HW definition -------------------------------*/
#define USARTx                           USART2
#define USARTx_CLK_ENABLE()              __USART2_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE() 
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __USART2_RELEASE_RESET()


#define USARTx_TX_PIN                  GPIO_PIN_2
#define USARTx_TX_GPIO_PORT            GPIOA  
#define USARTx_TX_AF                   GPIO_AF4_USART2
#define USARTx_RX_PIN                  GPIO_PIN_3
#define USARTx_RX_GPIO_PORT            GPIOA 
#define USARTx_RX_AF                   GPIO_AF4_USART2

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART2_IRQn
#define USARTx_IRQHandler                USART2_IRQHandler

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_CHANNEL             DMA1_Channel7

/* Definition for USARTx's DMA Request */
#define USARTx_TX_DMA_REQUEST             DMA_REQUEST_4

/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn                DMA1_Channel4_5_6_7_IRQn
#define USARTx_DMA_TX_IRQHandler          DMA1_Channel4_5_6_7_IRQHandler

#define USARTx_Priority 0
#define USARTx_DMA_Priority 0

#define LED_Toggle( x )
#define LED_On( x )
#define LED_Off( x )

#ifdef __cplusplus
}
#endif

#endif /* __HW_CONF_L0_H__ */

