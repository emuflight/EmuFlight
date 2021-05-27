/*
 * This file is part of Cleanflight and Betaflight and EmuFlight.
 *
 * Cleanflight and Betaflight and EmuFlight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight and EmuFlight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"
#include "adaptive_filter.h"

static float SquaredNorm(adaptiveFilter_t *pData) {
	float output = 0;

	for (int i = 0; i < pData->length; i++) {
		output += pData->pBuffer[i] * pData->pBuffer[i];
	}

	return output;
}

static void adaptWeights(adaptiveFilter_t *pData) {
	float normStepSize;

	normStepSize = (pData->stepSize) / (pData->regularization + SquaredNorm(pData)); // normalize step size

	for (int i = pData->length - 1; i >= 0; i--) {
        // wrap index
        if (pData->BufferIdx >= pData->length) {
            pData->BufferIdx = 0;
        }

    // Normalized Least Mean Square update equation
		pData->pWeights[i] += normStepSize * (pData->error) * (pData->pBuffer[pData->BufferIdx++]);
	}
}

static float filter(float input, adaptiveFilter_t *pData) {
	float output = 0;
	// wrap index
  if (pData->BufferIdx >= pData->length) {
      pData->BufferIdx = 0;
  }

	// overwrite oldest input with new input
	pData->pBuffer[pData->BufferIdx++] = input;

	for (int i = pData->length - 1; i >= 0; i--) {
        // wrap index
        if (pData->BufferIdx >= pData->length) {
            pData->BufferIdx = 0;
        }

		// compute inner product of weight vector and buffer
		output += (pData->pWeights[i]) * (pData->pBuffer[pData->BufferIdx++]);
	}

	return output;
}

static float delayInput(adaptiveFilter_t *pData, float rawInput) {
    pData->data[pData->idx] = rawInput;

    pData->idx++;
    if (pData->idx > pData->samples) {
        pData->idx = 0;
    }

    // filter the delayedInput to help reduce the overall noise this prediction adds
    float delayedInput = pData->data[pData->idx];

    return pData->delayedCrossFeed * rawInput + (1 - pData->delayedCrossFeed) * delayedInput;
}

float adaptiveFilterApply(float input, adaptiveFilter_t *pData) {
	float output;
  float delayedInput;
  if (pData->length > 1) {

    delayedInput = delayInput(pData, input);

	  output = filter(input, pData); // filter the input
	  pData->error = delayedInput - output; // update the error
	  adaptWeights(pData); // update adaptive filter weights

	  return output;
  } else {
    return input;
  }
}
