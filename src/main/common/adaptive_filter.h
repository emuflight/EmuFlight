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

#pragma once

#define MAX_ADAPTIVE_FILTER_TAPS 200

typedef struct adaptiveFilter_s{
	float stepSize; // adaptive filter step size
  float regularization; // regularization constant
	uint8_t length; // length of filter
	float pBuffer[MAX_ADAPTIVE_FILTER_TAPS]; // pointer to input buffer
	uint8_t BufferIdx; // circular index into input buffer
	float pWeights[MAX_ADAPTIVE_FILTER_TAPS]; // pointer to adaptive filter weights
	float error; // pointer to output error (desired - output) state
	float squaredNorm; // pBuffer added and squared

  // structure for the delayed input data
  uint8_t samples;
  uint8_t idx;
  float data[MAX_ADAPTIVE_FILTER_TAPS + 1]; // This is gonna be a ring buffer
  float delayedCrossFeed;
} adaptiveFilter_t;

float adaptiveFilterApply(float input, adaptiveFilter_t *pData);
