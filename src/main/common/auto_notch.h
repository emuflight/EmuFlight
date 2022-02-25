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

#include "filter.h"

typedef struct autoNotch_s {
    float preVariance;
    pt1Filter_t preVarianceFilter; // used as an exponential average, setup k to act like exponential average
    biquadFilter_t preVarianceBandpass;

    float noiseLimit; // default of 50 allows 70 amplitude noise to be totally notched
    float weight;
    float invWeight;

    biquadFilter_t notchFilter; // the notch filter we apply to the data
} autoNotch_t;

void initAutoNotch(autoNotch_t *autoNotch, float initial_frequency, int q, int noiseLimit, float looptimeUs);
float applyAutoNotch(autoNotch_t *autoNotch, float input);
void updateAutoNotch(autoNotch_t *autoNotch, float frequency, float q, float weightMultiplier, float looptimeUs);
