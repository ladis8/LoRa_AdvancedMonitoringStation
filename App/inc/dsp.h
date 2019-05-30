/*!
 * \file		dsp.h
 *
 * \brief		Application DSP API - definition of functions for signal processing
 *
 * \copyright
 *
 * \author		Ladislav Stefka
 *
 */

#ifndef __DSP_H__
#define __DSP_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include "arm_math.h"





void calculateRMSofSignal(uint16_t *signal, uint16_t fftSamplesNum, float32_t *rms, float32_t *rms_meanRemoved);

void calculatePeakofSignal(uint16_t *signal, uint16_t N, float32_t *vpp, uint16_t *vppIndex);

void calculateTimeDomainAnalysis(uint16_t *signal, uint16_t N, uint16_t threshold,
		uint16_t *firstTresholdCrossing, uint8_t *ringDownCount,  uint16_t *thresholdDuration);

void calculateRiseTimeofSignal(uint16_t vppIndex, uint16_t firstThresholdCrossing, uint16_t *riseTime);

void calculateKurtosisRatioofSignal(uint16_t *signal, uint16_t N, uint8_t percentageToDrop, float32_t *kurtosisRatio);


void calculateFFT(uint16_t *signal, uint16_t fftSamplesNum, float32_t *fftBuffer, uint8_t ifftFlag, uint8_t doBitReverse);

void findFFTPeaks(float32_t *fftBuffer, uint16_t fftSamplesNum, float32_t *fftPeaksValues, uint16_t *fftPeaksIndexes, uint8_t fftPeaksNum, float32_t delta, uint8_t lookAhead);

void convertFFTPeaks(float32_t *fftPeaksValuesFloat, uint8_t *fftPeaksValuesInt, uint16_t fftPeaksNum);



#endif

