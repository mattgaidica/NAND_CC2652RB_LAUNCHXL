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

#include <ESLO.h>

// swaBuffer:
//#include "R0003_00231.h" // perfect SWA
//#include "R0003_00359.h" // crappy SWA
//#include "R0003_00362.h" // good detection
#include "R0003_00363.h" // 2.72Hz
//#include "R0003_00220.h" // non-SWA
//#include "R0003_00659.h" // LF test

static float32_t complexFFT[FFT_LEN], realFFT[FFT_HALF_LEN],
		imagFFT[FFT_HALF_LEN], angleFFT[FFT_HALF_LEN], powerFFT[FFT_HALF_LEN];
float32_t swaFFT[FFT_LEN] = { 0 };

float32_t filtInput[SWA_LEN], filtOutput[SWA_LEN];
float32_t *InputValuesf32_ptr = &filtInput[0];  // declare Input pointer
float32_t *OutputValuesf32_ptr = &filtOutput[0]; // declare Output pointer

uint32_t fftSize = FFT_LEN;
uint32_t ifftFlag = 0;
arm_rfft_fast_instance_f32 S;
uint32_t maxIndex = 0;
arm_status armStatus;
float32_t maxValue;
uint32_t timeElapsed, iStep;

float32_t Fs, stepSize, Fc; // moved these

// Filter
#define NUM_SECTIONS_IIR 2

// IIR Coefficients
float32_t iirCoeffsf32[NUM_SECTIONS_IIR * 5] = { // b0, b1, b2, a1, a2
		0.05871944653317, 0.11743889306635, 0.05871944653317, 1.23859224269646,
				-0.47527907203419, 0.96505525434840, -1.93011050869680,
				0.96505525434840, 1.94430253415389, -0.94604808142077 };

#define BLOCKSIZE 32
#define NUMBLOCKS  (SWA_LEN/BLOCKSIZE)
float32_t iirStatesf32[NUM_SECTIONS_IIR * 2];

arm_biquad_cascade_df2T_instance_f32 Sf;

/* Driver Header files */

#include <ti/drivers/GPIO.h>
#include <ti/sysbios/knl/Clock.h>

/* Driver configuration */
#include <ti_drivers_config.h>

static float32_t ESLO_ADSgain_uV(int32_t rawValue) {
	float32_t rawFloat = (float32_t) rawValue;
	float32_t refV = (VREF / ADS_GAIN) / 8388607;
	float32_t empiricMult = 1.6667;
	float32_t uV = rawFloat * (refV) * 1000000 * empiricMult; // 5/3 is empiric, see MATLAB
	return uV;
}

void* mainThread(void *arg0) {
	GPIO_init();
	GPIO_write(LED_0, CONFIG_GPIO_LED_ON);

	while (1) {
		GPIO_toggle(LED_0);

		timeElapsed = Clock_getTicks();
		uint32_t k, iStep;
		uint32_t swaDiv = 2;

		// convert to float for CMSIS functions
		float32_t filtSum = 0;
		for (k = 0; k < SWA_LEN; k++) {
			//			filtInput[k] = (float32_t) swaBuffer[k];
			filtInput[k] = ESLO_ADSgain_uV(swaBuffer[k]);
			filtSum += filtInput[k];
		}
		// remove DC component
		float32_t mean_uV = filtSum / SWA_LEN;
		for (k = 0; k < SWA_LEN; k++) {
			filtInput[k] = filtInput[k] - mean_uV;
		}

		// Initialise Biquads
		arm_biquad_cascade_df2T_init_f32(&Sf, NUM_SECTIONS_IIR,
				&(iirCoeffsf32[0]), &(iirStatesf32[0]));

		// Perform IIR filtering operation
		for (k = 0; k < NUMBLOCKS; k++)
			arm_biquad_cascade_df2T_f32(&Sf,
					InputValuesf32_ptr + (k * BLOCKSIZE),
					OutputValuesf32_ptr + (k * BLOCKSIZE),
					BLOCKSIZE); // perform filtering

		// find max uV of filtered signal
		float32_t max_uV = 0;
		for (k = 0; k < SWA_LEN; k++) {
			if (fabs(filtOutput[k]) > max_uV) {
				max_uV = fabs(filtOutput[k]);
			}
		}

		// is the filtered max amplitude above user thresh?
//		if (max_uV < (float32_t) esloSettings[Set_SWAThresh]) {
//			return;
//		}

		// only init once
		arm_rfft_fast_init_f32(&S, fftSize);

		// subsample to reduce Fs and make FFT more accurate for SWA freqs
		while (1) {
			memset(swaFFT, 0x00, sizeof(float32_t) * FFT_LEN); // swaFFT is manipulated in place, always reset to zero
			if (swaDiv == 2) {
				for (k = 0; k < SWA_LEN / swaDiv; k++) {
					swaFFT[k] = filtOutput[k * swaDiv]; // subsample
				}
			} else {
				for (k = 0; k < SWA_LEN / 2; k++) {
					swaFFT[k] = filtOutput[k + (SWA_LEN / 2)]; // memcpy from tail
				}
			}

			// input is real, output is interleaved real and complex
			arm_rfft_fast_f32(&S, swaFFT, complexFFT, ifftFlag);

			// compute power
			arm_cmplx_mag_squared_f32(complexFFT, powerFFT,
			FFT_HALF_LEN);
			arm_max_f32(powerFFT, FFT_HALF_LEN, &maxValue, &maxIndex);

			Fs = EEG_FS / EEG_SAMPLING_DIV / swaDiv; // effective Fs
			stepSize = (Fs / 2) / FFT_HALF_LEN;
			Fc = stepSize * maxIndex;

			if (Fc < SWA_F_MIN || Fc >= SWA_F_MAX) { // do ratio here?
//				return;
			}
			if (Fc < 2 || swaDiv == 1) {
				break;
			} // retry swaDiv=1
			swaDiv--;
		}

		float32_t SWA_mean = 0;
		float32_t THETA_mean = 0;
		uint16_t SWA_count = 0;
		uint16_t THETA_count = 0;
		for (iStep = 0; iStep < FFT_HALF_LEN; iStep++) {
			if (stepSize * iStep >= SWA_F_MIN && stepSize * iStep < SWA_F_MAX) {
				SWA_mean = SWA_mean + powerFFT[iStep];
				SWA_count++;
			}
			if (stepSize * iStep >= THETA_F_MIN
					&& stepSize * iStep < THETA_F_MAX) {
				THETA_mean = THETA_mean + powerFFT[iStep];
				THETA_count++;
			}
		}
		SWA_mean = SWA_mean / (float32_t) SWA_count;
		THETA_mean = THETA_mean / (float32_t) THETA_count;
		float32_t swaRatio = SWA_mean / THETA_mean;

		// is the SWA/THETA ratio above user thresh?
//		if (swaRatio < (float32_t) esloSettings[Set_SWARatio]) {
//			return;
//		}

		// redo FFT on raw data to get true phase
		memset(swaFFT, 0x00, sizeof(float32_t) * FFT_LEN);
		if (swaDiv == 2) {
			for (k = 0; k < SWA_LEN / swaDiv; k++) {
				swaFFT[k] = (float32_t) swaBuffer[k * swaDiv]; // subsample
			}
		} else {
			for (k = 0; k < SWA_LEN / 2; k++) {
				swaFFT[k] = (float32_t) swaBuffer[k + (SWA_LEN / 2)]; // memcpy from tail
			}
		}
		arm_rfft_fast_f32(&S, swaFFT, complexFFT, ifftFlag);

		// de-interleave real and complex values, used in atan() for phase
		for (k = 0; k <= (FFT_LEN / 2) - 1; k++) {
			realFFT[k] = complexFFT[k * 2];
			imagFFT[k] = complexFFT[(k * 2) + 1];
		}
		// find angle of FFT
		for (k = 0; k <= FFT_LEN / 2; k++) {
			angleFFT[k] = atan2f(imagFFT[k], realFFT[k]);
		}

		float32_t degSec = 360 * Fc;
		float32_t windowLength = SWA_LEN / (Fs * EEG_SAMPLING_DIV);
		timeElapsed = (Clock_getTicks() - timeElapsed) * Clock_tickPeriod;
		float32_t computeDegrees = degSec * (float32_t) timeElapsed / 1000000;
		float32_t endAngle = degSec * windowLength
				+ (angleFFT[maxIndex] * 180 / M_PI) + computeDegrees;

		int32_t phaseA ngle = (int32_t) (1000 * endAngle) % (360 * 1000);
		int32_t dominantFreq = (int32_t) (Fc * 1000);

		sleep(1);
	}
}
