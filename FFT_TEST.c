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

uint32_t fftSize = FFT_LEN;
uint32_t ifftFlag = 0;
arm_rfft_fast_instance_f32 S;
uint32_t maxIndex = 0;
arm_status armStatus;
float32_t maxValue;
uint16_t iArm;
uint32_t timeElapsed, iStep;

float32_t Fs, stepSize, Fc; // moved these

// Filter
#define IIR_ORDER 16
#define IIR_NUMSTAGES (IIR_ORDER/2)
static float32_t m_biquad_state[IIR_ORDER];
static float32_t m_biquad_coeffs[5 * IIR_NUMSTAGES] = { 0.8438558985f,
		-1.6351879750f, 0.8438558985f, 1.9287731453f, -0.9654257558f,
		0.8438558985f, -1.6873686793f, 0.8438558985f, 1.9945479756f,
		-0.9952291527f, 0.7188136140f, -1.3634894168f, 0.7188136140f,
		1.8997354675f, -0.9259286369f, 0.7188136140f, -1.4374526701f,
		0.7188136140f, 1.9848343985f, -0.9857663068f, 0.4658449997f,
		-0.6668070904f, 0.4658449997f, 1.9531940082f, -0.9553548338f,
		0.4658449997f, -0.9316719297f, 0.4658449997f, 1.8897069251f,
		-0.9007244148f, 0.1138709021f, -0.2216813213f, 0.1138709021f,
		1.9493721928f, -0.9905892999f, 0.1138709021f, -0.2276875334f,
		0.1138709021f, 1.9982291468f, -0.9988431274f };
arm_biquad_cascade_df2T_instance_f32 const filtInst = {
IIR_ORDER / 2, m_biquad_state, m_biquad_coeffs };

#define BLOCK_SIZE	32

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

		uint32_t blockSize = BLOCK_SIZE;
		uint32_t numBlocks = SWA_LEN / BLOCK_SIZE;
		uint32_t i;
		uint32_t swaDiv = 2;

		// convert to float for CMSIS functions
		for (iArm = 0; iArm < SWA_LEN; iArm++) {
			filtInput[iArm] = (float32_t) swaBuffer[iArm];
		}
		// !! what is the added delay for more precision?
		// !! NEED TO GET FILTER RIGHT FIRST, not sure I need block loop
		armStatus = ARM_MATH_SUCCESS;
		armStatus = arm_rfft_fast_init_f32(&S, fftSize);

		for (i = 0; i < numBlocks; i++) {
//			arm_fir_f32(&S, inputF32 + (i * blockSize),
//					outputF32 + (i * blockSize), blockSize);
			arm_biquad_cascade_df2T_f32(&filtInst, filtInput + (i * blockSize),
					filtOutput + (i * blockSize), blockSize);
		}

		// subsample to reduce Fs and make FFT more accurate for SWA freqs
		for (swaDiv = 2; swaDiv > 0; swaDiv--) {
			memset(swaFFT, 0x00, sizeof(float32_t) * FFT_LEN); // swaFFT is manipulated in place, always reset to zero
			if (swaDiv == 2) {
				for (iArm = 0; iArm < SWA_LEN / swaDiv; iArm++) {
					swaFFT[iArm] = filtOutput[iArm * swaDiv]; // subsample
				}
			} else {
				for (iArm = 0; iArm < SWA_LEN / 2; iArm++) {
					swaFFT[iArm] = filtOutput[iArm + (SWA_LEN / 2)]; // memcpy from tail
				}
			}

//			if (armStatus != ARM_MATH_SUCCESS) {
//				return NULL; // !! return?
//			}
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
				return NULL; // !! return?
			}
//			if (Fc < 2) { // !! hardcode this?
//				break;
//			} // retry swaDiv=1
		}

		// !! if there is any phase distortion, need to FFT on original (float_32t) swaBuffer data to get phase at Fc

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

		// de-interleave real and complex values, used in atan() for phase
		for (iArm = 0; iArm <= (FFT_LEN / 2) - 1; iArm++) {
			realFFT[iArm] = complexFFT[iArm * 2];
			imagFFT[iArm] = complexFFT[(iArm * 2) + 1];
		}
		// find angle of FFT
		for (iArm = 0; iArm <= FFT_LEN / 2; iArm++) {
			angleFFT[iArm] = atan2f(imagFFT[iArm], realFFT[iArm]);
		}

		float32_t degSec = 360 * Fc;
		float32_t windowLength = (float32_t) SWA_LEN
				/ (EEG_FS / (float32_t) EEG_SAMPLING_DIV);
		timeElapsed = (Clock_getTicks() - timeElapsed) * Clock_tickPeriod;
		float32_t computeDegrees = degSec * (float32_t) timeElapsed / 1000000;
		float32_t endAngle = degSec * windowLength
				+ (angleFFT[maxIndex] * 180 / M_PI) + computeDegrees;

		int32_t phaseAngle = (int32_t) (1000 * endAngle) % (360 * 1000);
		int32_t dominantFreq = (int32_t) (Fc * 1000);

		sleep(1);
	}
}
