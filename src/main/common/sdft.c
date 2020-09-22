/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>
#include <stdbool.h>

#include "platform.h"

#include "common/maths.h"
#include "common/sdft.h"

static FAST_DATA_ZERO_INIT bool      constsInitialized;
static FAST_DATA_ZERO_INIT float     dampingFactor;
static FAST_DATA_ZERO_INIT float     r_to_n;
static FAST_DATA_ZERO_INIT complex_t twiddle[SDFT_BIN_COUNT];


void sdftInit(sdft_t *sdft, const uint8_t startBin, const uint8_t endBin)
{
	if (!constsInitialized) {
		dampingFactor = nexttowardf(1, 0);
		r_to_n = powf(dampingFactor, SDFT_SAMPLE_SIZE);
		const float c = 2.0f * M_PIf / (float)SDFT_SAMPLE_SIZE;
		float phi = 0.0f;
		for (uint8_t i = 0; i < SDFT_BIN_COUNT; i++) {
			phi = c*i;
			twiddle[i] = cos_approx(phi) + _Complex_I * sin_approx(phi);
		}
		constsInitialized = true;
	}

	sdft->idx = 0;
	sdft->startBin = startBin;
	sdft->endBin = endBin;

	for (uint8_t i = 0; i < SDFT_SAMPLE_SIZE; i++) {
		sdft->samples[i] = 0.0f;
	}

	for (uint8_t i = 0; i < SDFT_BIN_COUNT; i++) {
		sdft->data[i] = 0.0f;
	}
}

// Add new sample to frequency spectrum
FAST_CODE void sdftPush(sdft_t *sdft, const float sample)
{
	const float delta = sample - r_to_n * sdft->samples[sdft->idx];
	sdft->samples[sdft->idx] = sample;

	for (uint8_t i = sdft->startBin; i <= sdft->endBin; i++) {
		sdft->data[i] = twiddle[i] * (dampingFactor * sdft->data[i] + delta);
	}

	sdft->idx = (sdft->idx + 1) % SDFT_SAMPLE_SIZE;
}


// Get squared magnitude of frequency spectrum
FAST_CODE void sdftMagSq(const sdft_t *sdft, float *output)
{
	float re;
	float im;
	for (uint8_t i = 0; i < SDFT_BIN_COUNT; i++) {
		re = crealf(sdft->data[i]);
		im = cimagf(sdft->data[i]);
		output[i] = re * re + im * im;
	}
}


// Apply Hann window
FAST_CODE void sdftWindow(const sdft_t *sdft, complex_t *output)
{
	for (uint8_t i = sdft->startBin; i < sdft->endBin; i++) {
		output[i] = 0.5f * sdft->data[i] - 0.25f * sdft->data[i - 1] + sdft->data[i + 1];
	}
	output[sdft->endBin] = 0.5f * sdft->data[sdft->endBin] - 0.25f * sdft->data[sdft->endBin - 1] + sdft->data[(sdft->endBin + 1) % SDFT_BIN_COUNT];
}


// Apply Hann window and calculate squared magnitude
FAST_CODE void sdftWindowf(const sdft_t *sdft, float *output)
{
	complex_t temp;
	float re;
	float im;
	for (uint8_t i = sdft->startBin; i < sdft->endBin; i++) {
		temp = 0.5f * sdft->data[i] - 0.25f * sdft->data[i - 1] + sdft->data[i + 1];
		re = crealf(temp);
		im = cimagf(temp);
		output[i] = re * re + im * im;
	}
	temp = 0.5f * sdft->data[sdft->endBin] - 0.25f * sdft->data[sdft->endBin - 1] + sdft->data[(sdft->endBin + 1) % SDFT_BIN_COUNT];
	re = crealf(temp);
	im = cimagf(temp);
	output[sdft->endBin] = re * re + im * im;
}