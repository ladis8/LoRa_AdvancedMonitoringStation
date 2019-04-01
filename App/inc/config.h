#ifndef __CONFIG_H__
#define __CONFIG_H__

#ifdef __cplusplus
 extern "C" {
#endif


#include <stdbool.h>
#include <stdint.h>
#include "arm_math.h"


 /* DEFAULT RADIONET SETTINGS*/
#define CFG_INITIAL_SID 0x00						//0x00 is for new join
#define CFG_JOIN_INTERVAL 10000


/* DEFAULT SETTING FOR LORA MODULE */
#define CFG_TX_OUTPUT_POWER                             14        // dBm


#define CFG_LORA_FREQUENCY                              868500000 // Hz
#define CFG_LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define CFG_LORA_SPREADING_FACTOR                       12         // [SF7..SF12]
#define CFG_LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define CFG_LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define CFG_LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define CFG_LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define CFG_LORA_IQ_INVERSION_ON                        false


#define CFG_RX_TIMEOUT          						5000
#define CFG_TX_TIMEOUT									10000 //not used

/* DEFAULT SETTING FOR LORA MODULE */


 /* DEFAULT SETTING FOR FFT ADC */
 //TODO: check bit reverse and ifft
 //TODO: find more logical order
#define CFG_FFT_SAMPLES 1024

#define CFG_ADC_CLOCK_DIVIDER 2
#define CFG_ADC_SAMPLING_TIME 160




 typedef enum{
     LOWPOWER,
     RX,
     RX_TIMEOUT,
     RX_ERROR,
     TX,
     TX_TIMEOUT,
 } radioState_t;


 typedef struct{

    struct{
       uint8_t sId; // Session ID
       bool joinPending;
       bool nodeJoined;
       radioState_t state;
       uint16_t joinInterval;
       uint16_t statusinfoInterval;

    } radionet;

    struct{
    	uint8_t bw;
    	uint8_t sf;
    	uint8_t cr;
    	uint32_t frequency;
    	uint32_t rxTimeout;
    	uint32_t txTimeout;

	} lora;

	struct{
		uint8_t ifftFlag;
		uint8_t doBitReverse;

		uint16_t fftSamples;
		uint16_t adcClockDivider;
		uint8_t	adcTimings;
		uint8_t fftPeaksNum;
		uint8_t tempAveragingNum;
		uint8_t rmsAveragingNum;

	} adc_fft;


    struct{
       uint16_t tempInterval;                // Temperature measure interval in ms
       uint16_t tempBufferSize;
       uint16_t loadcellInterval;
       uint16_t loadcellBufferSize;

       uint16_t rmsInterval;
       uint16_t fftCompressInterval;


    } monitoring;


 } loraNode_cfg_t;



#define MEASURMENTS_BUFFER_SIZE 128

 typedef struct{

	 uint16_t tempBuffer[MEASURMENTS_BUFFER_SIZE];
	 uint16_t loadcellBuffer [MEASURMENTS_BUFFER_SIZE];
	 float32_t rmsBuffer [MEASURMENTS_BUFFER_SIZE];

	 uint16_t *adcReadings;
	 float32_t *testInput;


 }loraNode_msr_t;




void printConfig();


#endif
