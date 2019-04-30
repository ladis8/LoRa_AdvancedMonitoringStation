#ifndef __CONFIG_H__
#define __CONFIG_H__

#ifdef __cplusplus
 extern "C" {
#endif


#include <stdbool.h>
#include <stdint.h>
#include "arm_math.h"



#define CFG_FWVER 0x01

 /* DEFAULT RADIONET SETTINGS*/
#define CFG_INITIAL_SID 								0x00	//0x00 is for new join
#define CFG_UID											(*(uint32_t *)0x1FF80050U)
#define CFG_JOIN_INTERVAL 								15000
#define CFG_STATUSINFO_INTERVAL 						35000
#define CFG_STATUSINFO_LISTEN_INTERVAL 					5000



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


#define CFG_RX_TIMEOUT          						10000
#define CFG_TX_TIMEOUT									10000 //not used



/* DEFAULT SETTING FOR LORA MODULE */


 /* DEFAULT SETTING FOR FFT ADC */
 //TODO: check bit reverse and ifft
 //TODO: find more logical order
#define CFG_ADC_CLOCK_DIVIDER 	ASYNC_DIV2
#define CFG_ADC_SAMPLING_TIME 	S_160CYCLE5
#define CFG_FFT_SAMPLES_NUM   	N_1024
#define CFG_FFT_PEAKS_NUM 		5
#define CFG_FFT_PEAKS_DELTA     2

 /* DEFAULT SETTING FOR DSP */
#define CFG_DSP_THRESHOLD_VOLTAGE 2.0
#define CFG_DSP_KURTOSIS_TRIMMED_SAMPLES 10
#define CFG_DSP_RMS_AVERAGING_NUM 1
#define CFG_DSP_RMS_AC true





/* RADIO STATES for state machine*/
 typedef enum{
     LOWPOWER,
     RX,
	 RX_DONE,
     RX_TIMEOUT,
     RX_ERROR,
     TX,
     TX_TIMEOUT,

 } radioState_t;


 /* FFT && CONFIGURABLE PROPERTIES */
 typedef enum{
	 N_64,
	 N_128,
	 N_512,
	 N_1024,
	 N_2048

 }fftSamplesNum_t;



 typedef enum{
	 S_1CYCLE5,
	 S_3CYCLE5,
	 S_7CYCLE5,
	 S_12CYCLE5,
	 S_19CYCLE5,
	 S_39CYCLE5,
	 S_79CYCLE5,
	 S_160CYCLE5,

 }adcSamplingTime_t;



 typedef enum{
	 ASYNC_DIV1,
	 ASYNC_DIV2,
	 ASYNC_DIV4,
	 ASYNC_DIV8,
	 ASYNC_DIV16,
	 ASYNC_DIV32,
	 ASYNC_DIV64,
	 ASYNC_DIV128,
	 ASYNC_DIV256

 }adcDivider_t;





 /* ALL CONFIGURATIONS */
 typedef struct{

	 uint8_t fwver;

    struct{
       uint8_t sId; 		    // Session ID
       uint32_t uId; 		    //Unique ID of device == MAC (but only first 32 bit)
       bool joinPending;
       bool nodeJoined;
       bool configPending;
       bool nodeConfigured;
       radioState_t state;
       uint16_t joinInterval;
       uint16_t statusinfoInterval;
       uint16_t statusinfoListenInterval;

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

		uint8_t adcClockDivider;
		uint8_t	adcSamplingTime;
		uint8_t fftSamplesNum;
		uint8_t fftPeaksNum;
		uint8_t fftPeaksDelta;
		uint8_t tempAveragingNum;

	} adc_fft;

	struct{
		uint16_t thresholdVoltage;
		uint8_t kurtosisTrimmedSamples;
		uint8_t rmsAc;
		uint8_t rmsAveragingNum;
	} dsp;


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
