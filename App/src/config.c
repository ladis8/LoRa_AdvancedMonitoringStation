/*
 * config.c
 *
 *  Created on: Mar 4, 2019
 *      Author: ladislav
 */

#include "config.h"

#include "string.h"
#include "util_console.h"

//WISDOM: variables definied in config.h are multiply defined by includion


const char * adcSamplingTime_names[] = {
		 "S_1CYCLE5","S_3CYCLE5","S_7CYCLE5","S_12CYCLE5",
		 "S_19CYCLE5","S_39CYCLE5","S_79CYCLE5","S_160CYCLE5"
 };

const char * adcDivider_names[] = {
	 "ASYNC_DIV1","ASYNC_DIV2","ASYNC_DIV4","ASYNC_DIV8","ASYNC_DIV16",
	 "ASYNC_DIV32","ASYNC_DIV64","ASYNC_DIV128","ASYNC_DIV256"
 };

const char * fftSamplesNum_names[] = {
	"N_64","N_128","N_512","N_1024","N_2048"
 };



loraNode_cfg_t loraNode_cfg = {

		 .fwver = CFG_FWVER,

		.radionet = {
				.sId = CFG_INITIAL_SID,
				.state = LOWPOWER,
				.nodeJoined = false,
				.joinPending = false,
				.joinInterval = 10000,
		},

		.lora = {
				.bw = CFG_LORA_BANDWIDTH,
		    	.sf = CFG_LORA_SPREADING_FACTOR,
		    	.cr = CFG_LORA_CODINGRATE,
		    	.frequency = CFG_LORA_FREQUENCY,
				.rxTimeout = CFG_RX_TIMEOUT,
				.txTimeout = CFG_TX_TIMEOUT,
		},

		.adc_fft = {
			.ifftFlag 	  = 0,
			.doBitReverse = 1,
			.fftSamplesNum = CFG_FFT_SAMPLES_NUM,
			.fftPeaksNum = CFG_FFT_PEAKS_NUM,
			.adcClockDivider = CFG_ADC_CLOCK_DIVIDER,
			.adcSamplingTime = CFG_ADC_SAMPLING_TIME,
		},
};


void printConfig(){

	PRINTF("Config params: \r\n");
	PRINTF("Radionet params: \r\n");
	PRINTF("    SID          : 0x%02x\r\n", loraNode_cfg.radionet.sId);
	PRINTF("    UID      	 : 0x%08x\r\n", loraNode_cfg.radionet.uId);
	PRINTF("    joinInterval : %u\r\n" , loraNode_cfg.radionet.joinInterval);
	PRINTF("    state        : %u\r\n" , loraNode_cfg.radionet.state);
	PRINTF("LoRa radio params: \r\n");
	PRINTF("    BW : %u\r\n", loraNode_cfg.lora.bw);
	PRINTF("    CR : %u\r\n", loraNode_cfg.lora.sf);
	PRINTF("    SF : %u\r\n", loraNode_cfg.lora.cr);
	PRINTF("ADC && FFT arams: \r\n");
	PRINTF("    FFT SAMPLES   : %s\r\n", fftSamplesNum_names[loraNode_cfg.adc_fft.fftSamplesNum]);
	PRINTF("    FFT NUM PEAKS : %u\r\n", loraNode_cfg.adc_fft.fftPeaksNum);
	PRINTF("    ADC sampling T: %s\r\n", adcSamplingTime_names[loraNode_cfg.adc_fft.adcSamplingTime]);
	PRINTF("    ADC divider   : %s\r\n", adcDivider_names[loraNode_cfg.adc_fft.adcClockDivider]);
	PRINTF("    TEMP samples  : %u\r\n", loraNode_cfg.adc_fft.tempAveragingNum);
	PRINTF("    RMS samples   : %u\r\n", loraNode_cfg.adc_fft.rmsAveragingNum);
	PRINTF("    Status int.   : %u\r\n", loraNode_cfg.radionet.statusinfoInterval);

}


