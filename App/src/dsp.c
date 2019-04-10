

#include "dsp.h"

#include <stdbool.h>
#include <stdlib.h>	//malloc

#include "arm_const_structs.h"


void calculateMeanofSignal(uint16_t *signal, uint16_t fftSamplesNum, uint16_t *mean){
	uint32_t sum = 0;
	for (int i = 0; i < fftSamplesNum; i++){
		sum += signal[i];
	}
	*mean = (uint16_t)(sum/fftSamplesNum);
}


void calculateRMSofSignal(uint16_t *signal, uint16_t N, float32_t *rms, float32_t *rms_meanRemoved){

	float32_t *voltageBuffer = (float32_t*) malloc(N * sizeof(float32_t));

	uint16_t mean;
	calculateMeanofSignal(signal,N, &mean);


	for (int i=0; i < N; i++){
		voltageBuffer[i] = ((float32_t)signal[i])/4095 * 3.3;
	}
	arm_rms_f32(voltageBuffer, N, rms);

	for (int i=0; i < N; i++){
		voltageBuffer[i]  = ((float32_t)signal[i] - mean)/4095 * 3.3;
	}
	arm_rms_f32(voltageBuffer, N, rms_meanRemoved);
	free(voltageBuffer);
}


void calculateFFT(uint16_t *signal, uint16_t N, float32_t *fftBuffer, uint8_t ifftFlag, uint8_t doBitReverse){
    /* Process the data through the CFFT/CIFFT module */
   //arm_status status = ARM_MATH_SUCCESS;
   //arm_cfft_f32(&arm_cfft_sR_f32_len1024, testInput_f32_10khz, ifftFlag, doBitReverse);
     /* Process the data through the Complex Magnitude Module for
     calculating the magnitude at each bin */
   //arm_cmplx_mag_f32(testInput_f32_10khz, testOutput, fftSize);
     /* Calculates maxValue and returns corresponding BIN value */
   //arm_max_f32(testOutput, fftSize, &maxValue, &testIndex);
   //if(testIndex !=  refIndex)
       //status = ARM_MATH_TEST_FAILURE;
   //PRINTF("Test index is %d\n", testIndex);


	//prepare buffer to hold complex signal

	for (int i=0; i < N; i++){
   		fftBuffer[2*i] = ((float32_t)signal[i])/4095 * 3.3;
   		fftBuffer[2*i +1] = 0;
	}

	const arm_cfft_instance_f32 *cfftInstance;

	switch (N){
		case 64: 	cfftInstance = &arm_cfft_sR_f32_len64; break;
		case 128: 	cfftInstance = &arm_cfft_sR_f32_len128; break;
		case 512: 	cfftInstance = &arm_cfft_sR_f32_len512; break;
		case 1024: 	cfftInstance = &arm_cfft_sR_f32_len1024; break;
		case 2048: 	cfftInstance = &arm_cfft_sR_f32_len2048; break;
		default: 	cfftInstance = &arm_cfft_sR_f32_len1024;
	}
	arm_cfft_f32(cfftInstance, fftBuffer, ifftFlag, doBitReverse);
	//arm_rfft_f32(&arm_cfft_sR_f32_len1024, testInput, loraNode_cfg.adc_fft.ifftFlag, loraNode_cfg.adc_fft.doBitReverse);
	arm_cmplx_mag_f32(fftBuffer, fftBuffer, N);

}


uint16_t  _min(float32_t *buff, uint16_t length){
	uint16_t index = 0;
	for (int i = 1; i < length; i++){
		if (buff[index] > buff[i])
			index = i;
	}
	return index;
}

uint16_t  _max(float32_t *buff, uint16_t length){
	uint16_t index = 0;
	for (int i = 1; i < length; i++){
		if (buff[index] > buff[i])
			index = i;
	}
	return index;
}


void findFFTPeaks(float32_t *fftBuffer, uint16_t N,
		float32_t *fftPeaksValues, uint16_t *fftPeaksIndexes, uint8_t fftPeaksNum,
		float32_t delta, uint8_t lookAhead){

	memset(fftPeaksValues, 0, fftPeaksNum);
	memset(fftPeaksIndexes, UINT16_MAX, fftPeaksNum);

	float32_t mx = 0;
	uint16_t mxPos=0;
	float32_t mn=2000.0;

	bool looksForMax = true;

	for (int i = 0; i < N/2; i++){

		//looks for max
		if (looksForMax){
			if (fftBuffer[i] > mx){
				mx = fftBuffer[i]; mxPos = i;
			}

			else if (mx - delta > fftBuffer[i]){

				//check lookahead
				bool jitter = false;
				for (int j = 0; j < lookAhead; j++){
					if (i + j < N && fftBuffer[i+j] > mx)
						jitter = true;
				}

				//save peak instead of minimum peak
				if (!jitter){
					uint16_t currentMinPos = _min(fftPeaksValues, fftPeaksNum);
					if (fftPeaksValues[currentMinPos] < mx){
						fftPeaksValues[currentMinPos] = mx; fftPeaksIndexes[currentMinPos] = mxPos;
					}
					mn = fftBuffer[i];
					looksForMax = false;
				}

			}

		}
		//looks for min
		else{
			if (fftBuffer[i] < mn)
				mn = fftBuffer[i];

			else if (mn + delta < fftBuffer[i]){
				mx = fftBuffer[i];
				mxPos = i;
				looksForMax = true;
			}
		}

	}

}

/*void convertFFTPeaks(float32_t *fftPeaksValuesFloat, uint16_t *fftPeaksIndexes, uint8_t *fftPeaksStructure, uint16_t fftPeaksNum){

	for (int i = 0; i < fftPeaksNum; i++){
		//select biggest peak
		uint16_t maxValueIndex = _max(fftPeaksValuesFloat, fftPeaksNum);
		float32_t maxValue = fftPeaksValuesFloat[maxValueIndex];

		//convert to uint8_t scale


		//mark as used
		fftPeaksValuesFloat[maxValueIndex] = -1.0;

		memcpy(fftPeaksStructure[3 * i], (fftPeaksIndexes + maxValueIndex), sizeof(uint16_t));
		fftPeaksStructure[3 * i + sizeof(uint16_t)] =
	}
}*/




