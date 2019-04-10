/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DSP_H__
#define __DSP_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include "arm_math.h"


void calculateMeanofSignal(uint16_t *signal, uint16_t fftSamplesNum, uint16_t *mean);

void calculateRMSofSignal(uint16_t *signal, uint16_t fftSamplesNum, float32_t *rms, float32_t *rms_meanRemoved);

void calculateFFT(uint16_t *signal, uint16_t fftSamplesNum, float32_t *fftBuffer, uint8_t ifftFlag, uint8_t doBitReverse);


void findFFTPeaks(float32_t *fftBuffer, uint16_t fftSamplesNum, float32_t *fftPeaksValues, uint16_t *fftPeaksIndexes, uint8_t fftPeaksNum, float32_t delta, uint8_t lookAhead);

void convertFFTPeaks(float32_t *fftPeaksValuesFloat, uint8_t *fftPeaksValuesInt, uint16_t fftPeaksNum);



#endif/* __DSP_H__*/

