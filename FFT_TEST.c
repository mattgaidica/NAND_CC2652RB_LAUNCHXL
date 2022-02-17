// https://gaidi.ca/weblog/implementing-a-biquad-cascade-iir-filter-on-a-cortex-m4?rq=CMSIS
// https://www.keil.com/pack/doc/CMSIS/DSP/html/group__BiquadCascadeDF2T.html#gafd8b4068de567e9012e444f1c2320e1c
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h> // rand()
#include <string.h>
#include <math.h> // atan2(x,y)

// CMSIS Math
#include "arm_math.h"
#include "arm_const_structs.h"

//#include "FFTsignal.h"
#include <ESLO.h>
//#include "R0003_00231.h" // perfect SWA
//#include "R0003_00359.h" // crappy SWA
//#include "R0003_00362.h" // good detection
//#include "R0003_00363.h" // 2.72Hz
#include "R0003_00220.h" // non-SWA
//int32_t swaBuffer[SWA_LEN * 2] = { 0 }; // filled by include
static float32_t complexFFT[FFT_LEN], realFFT[FFT_HALF_LEN],
		imagFFT[FFT_HALF_LEN], angleFFT[FFT_HALF_LEN], powerFFT[FFT_HALF_LEN];
float32_t swaFFT[FFT_LEN] = { 0 };
float32_t filtInput[SWA_LEN], filtOutput[SWA_LEN];

uint32_t fftSize = FFT_LEN;
uint32_t ifftFlag = 0;
arm_rfft_fast_instance_f32 S;
uint32_t maxIndex = 0;
arm_status armStatus;
float32_t maxValue;
uint16_t iArm;
uint32_t timeElapsed, iStep;

// Filter
#define IIR_ORDER 4
#define IIR_NUMSTAGES (IIR_ORDER/2)
static float32_t m_biquad_state[IIR_ORDER];
static float32_t m_biquad_coeffs[5 * IIR_NUMSTAGES] = { 1.0000000000f,
		-1.9999576322f, 1.0000000000f, 1.9489902175f, -0.9503655611f,
		1.0000000000f, -1.9997900711f, 1.0000000000f, 1.9929892321f,
		-0.9935824259f };
arm_biquad_cascade_df2T_instance_f32 const filtInst = {
IIR_ORDER / 2, m_biquad_state, m_biquad_coeffs };

/* Driver Header files */

#include <ti/drivers/GPIO.h>
#include <ti/sysbios/knl/Clock.h>

/* Driver configuration */
#include <ti_drivers_config.h>

void* mainThread(void *arg0) {
	GPIO_init();
	GPIO_write(LED_0, CONFIG_GPIO_LED_ON);

	while (1) {
		GPIO_toggle(LED_0);

		timeElapsed = Clock_getTicks();

		for (iArm = 0; iArm < SWA_LEN; iArm++) {
			filtInput[iArm] = (float32_t) swaBuffer[iArm];
		}
		arm_biquad_cascade_df2T_f32(&filtInst, filtInput, filtOutput,
		SWA_LEN);

		// subsample to reduce Fs and make FFT more accurate for SWA freqs
		memset(swaFFT, 0x00, sizeof(float32_t) * FFT_LEN);
		for (iArm = 0; iArm < SWA_LEN / FFT_SWA_DIV; iArm++) {
			swaFFT[iArm] = (float32_t) swaBuffer[iArm * FFT_SWA_DIV];
		}

		// remove DC offset
//		float32_t meanInput;
//		arm_mean_f32(swaFFT, SWA_LEN / FFT_SWA_DIV, &meanInput);
//		for (iArm = 0; iArm < SWA_LEN / FFT_SWA_DIV; iArm++) {
//			swaFFT[iArm] = swaFFT[iArm] - meanInput;
//		}

		armStatus = ARM_MATH_SUCCESS;
		armStatus = arm_rfft_fast_init_f32(&S, fftSize);
		// if (status != ARM_MATH_SUCCESS) <- use this to skip if failure (ARM_MATH_TEST_FAILURE)
		// input is real, output is interleaved real and complex
		arm_rfft_fast_f32(&S, swaFFT, complexFFT, ifftFlag);

		// de-interleave real and complex values, used in atan() for phase
		for (iArm = 0; iArm <= (FFT_LEN / 2) - 1; iArm++) {
			realFFT[iArm] = complexFFT[iArm * 2];
			imagFFT[iArm] = complexFFT[(iArm * 2) + 1];
		}

		// compute power
		arm_cmplx_mag_squared_f32(complexFFT, powerFFT,
		FFT_HALF_LEN);
		arm_max_f32(powerFFT, FFT_HALF_LEN, &maxValue, &maxIndex);

		float32_t Fs = 250 / EEG_SAMPLING_DIV / FFT_SWA_DIV; // effective Fs
		float32_t stepSize = (Fs / 2) / FFT_HALF_LEN;
		float32_t Fc = stepSize * maxIndex;

		// SWA_FFT_THRESH == 0 skips all filters for baseline recordings
		float32_t SWA_mean = 0;
		float32_t nSWA_mean = 0;
		uint16_t SWA_count = 0;
		uint16_t nSWA_count = 0;
		for (iStep = 0; iStep < FFT_HALF_LEN; iStep++) {
			if (stepSize * iStep > SWA_F_MIN && stepSize * iStep <= SWA_F_MAX) {
				SWA_mean = SWA_mean + powerFFT[iStep];
				SWA_count++;
			} else {
				nSWA_mean = nSWA_mean + powerFFT[iStep];
				nSWA_count++;
			}
		}
		SWA_mean = SWA_mean / (float32_t) SWA_count;
		nSWA_mean = nSWA_mean / (float32_t) nSWA_count;
		float swaRatio = SWA_mean / nSWA_mean;

		// find angle of FFT
		for (iArm = 0; iArm <= FFT_LEN / 2; iArm++) {
			angleFFT[iArm] = atan2f(imagFFT[iArm], realFFT[iArm]);
		}

		float32_t degSec = 360 * Fc;
		float32_t windowLength = (float32_t) SWA_LEN
				/ (250 / (float32_t) EEG_SAMPLING_DIV);
		timeElapsed = (Clock_getTicks() - timeElapsed) * Clock_tickPeriod;
		float32_t computeDegrees = degSec * (float32_t) timeElapsed / 1000000;
		float32_t endAngle = degSec * windowLength
				+ (angleFFT[maxIndex] * 180 / M_PI) + computeDegrees;

		int32_t phaseAngle = (int32_t) (1000 * endAngle) % (360 * 1000);
		int32_t dominantFreq = (int32_t) (Fc * 1000);

		sleep(1);
	}
}
