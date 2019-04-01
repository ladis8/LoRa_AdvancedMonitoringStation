/*
 * config.c
 *
 *  Created on: Mar 4, 2019
 *      Author: ladislav
 */

#include "config.h"
#include "util_console.h"

loraNode_cfg_t loraNode_cfg = {

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
			.fftSamples = CFG_FFT_SAMPLES,
			.adcClockDivider = CFG_ADC_CLOCK_DIVIDER,
			.adcTimings = CFG_ADC_SAMPLING_TIME,
		},


};

void printConfig(){

	PRINTF("Config params: \r\n");
	PRINTF("Radionet params: \r\n");
	PRINTF("    SID          : 0x%d\r\n", loraNode_cfg.radionet.sId);
	PRINTF("    joinInterval : %u\r\n" , loraNode_cfg.radionet.joinInterval);
	PRINTF("    state        : %u\r\n" , loraNode_cfg.radionet.state);
	PRINTF("LoRa radio params: \r\n");
	PRINTF("    BW : %u\r\n", loraNode_cfg.lora.bw);
	PRINTF("    CR : %u\r\n", loraNode_cfg.lora.sf);
	PRINTF("    SF : %u\r\n", loraNode_cfg.lora.cr);
	PRINTF("ADC && FFT arams: \r\n");
	PRINTF("    FFT SAMPLES   : %u\r\n", loraNode_cfg.adc_fft.fftSamples);
	PRINTF("    FFT NUM PEAKS : %u\r\n", loraNode_cfg.adc_fft.fftPeaksNum);
	PRINTF("    ADC timings   : %u\r\n", loraNode_cfg.adc_fft.adcTimings);
	PRINTF("    TEMP samples  : %u\r\n", loraNode_cfg.adc_fft.tempAveragingNum);
	PRINTF("    RMS samples   : %u\r\n", loraNode_cfg.adc_fft.rmsAveragingNum);
	PRINTF("    Status int.   : %u\r\n", loraNode_cfg.radionet.statusinfoInterval);

}


