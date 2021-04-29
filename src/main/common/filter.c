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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "fc/fc_rc.h"

#define M_LN2_FLOAT     0.69314718055994530942f
#define M_PI_FLOAT      3.14159265358979323846f
#define BIQUAD_Q        (1.0f / sqrtf(2.0f))     /* quality factor - 2nd order butterworth*/

// NULL filter

FAST_CODE float nullFilterApply(filter_t *filter, float input) {
    UNUSED(filter);
    return input;
}


// PT1 Low Pass filter

float pt1FilterGain(uint16_t f_cut, float dT) {
    const float RC = 0.5f / (M_PI_FLOAT * f_cut);
    return dT / (RC + dT);
}

void pt1FilterInit(pt1Filter_t *filter, float k) {
    filter->state = 0.0f;
    filter->k = k;
}

void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k) {
    filter->k = k;
}

FAST_CODE float pt1FilterApply(pt1Filter_t *filter, float input) {
    filter->state = filter->state + filter->k * (input - filter->state);
    return filter->state;
}

// Slew filter with limit

void slewFilterInit(slewFilter_t *filter, float slewLimit, float threshold) {
    filter->state = 0.0f;
    filter->slewLimit = slewLimit;
    filter->threshold = threshold;
}

FAST_CODE float slewFilterApply(slewFilter_t *filter, float input) {
    if (filter->state >= filter->threshold) {
        if (input >= filter->state - filter->slewLimit) {
            filter->state = input;
        }
    } else if (filter->state <= -filter->threshold) {
        if (input <= filter->state + filter->slewLimit) {
            filter->state = input;
        }
    } else {
        filter->state = input;
    }
    return filter->state;
}

// get notch filter Q given center frequency (f0) and lower cutoff frequency (f1)
// Q = f0 / (f2 - f1) ; f2 = f0^2 / f1
float filterGetNotchQ(float centerFreq, float cutoffFreq) {
    return centerFreq * cutoffFreq / (centerFreq * centerFreq - cutoffFreq * cutoffFreq);
}

/* sets up a biquad Filter */
void biquadFilterInitLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate) {
    biquadFilterInit(filter, filterFreq, refreshRate, BIQUAD_Q, FILTER_LPF);
}

void biquadFilterInit(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType) {
    // setup variables
    const float omega = 2.0f * M_PI_FLOAT * filterFreq * refreshRate * 0.000001f;
    const float sn = sin_approx(omega);
    const float cs = cos_approx(omega);
    const float alpha = sn / (2.0f * Q);
    float b0 = 0, b1 = 0, b2 = 0, a0 = 0, a1 = 0, a2 = 0;
    switch (filterType) {
    case FILTER_LPF:
        // 2nd order Butterworth (with Q=1/sqrt(2)) / Butterworth biquad section with Q
        // described in http://www.ti.com/lit/an/slaa447/slaa447.pdf
        b0 = (1 - cs) * 0.5f;
        b1 = 1 - cs;
        b2 = (1 - cs) * 0.5f;
        a0 = 1 + alpha;
        a1 = -2 * cs;
        a2 = 1 - alpha;
        break;
    case FILTER_NOTCH:
        b0 =  1;
        b1 = -2 * cs;
        b2 =  1;
        a0 =  1 + alpha;
        a1 = -2 * cs;
        a2 =  1 - alpha;
        break;
    case FILTER_BPF:
        b0 = alpha;
        b1 = 0;
        b2 = -alpha;
        a0 = 1 + alpha;
        a1 = -2 * cs;
        a2 = 1 - alpha;
        break;
    }
    // precompute the coefficients
    filter->b0 = b0 / a0;
    filter->b1 = b1 / a0;
    filter->b2 = b2 / a0;
    filter->a1 = a1 / a0;
    filter->a2 = a2 / a0;
    // zero initial samples
    filter->x1 = filter->x2 = 0;
    filter->y1 = filter->y2 = 0;
}

FAST_CODE void biquadFilterUpdate(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType) {
    // backup state
    float x1 = filter->x1;
    float x2 = filter->x2;
    float y1 = filter->y1;
    float y2 = filter->y2;
    biquadFilterInit(filter, filterFreq, refreshRate, Q, filterType);
    // restore state
    filter->x1 = x1;
    filter->x2 = x2;
    filter->y1 = y1;
    filter->y2 = y2;
}

FAST_CODE void biquadFilterUpdateLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate) {
    biquadFilterUpdate(filter, filterFreq, refreshRate, BIQUAD_Q, FILTER_LPF);
}

/* Computes a biquadFilter_t filter on a sample (slightly less precise than df2 but works in dynamic mode) */
FAST_CODE float biquadFilterApplyDF1(biquadFilter_t *filter, float input) {
    /* compute result */
    const float result = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2 - filter->a1 * filter->y1 - filter->a2 * filter->y2;
    /* shift x1 to x2, input to x1 */
    filter->x2 = filter->x1;
    filter->x1 = input;
    /* shift y1 to y2, result to y1 */
    filter->y2 = filter->y1;
    filter->y1 = result;
    return result;
}

/* Computes a biquadFilter_t filter in direct form 2 on a sample (higher precision but can't handle changes in coefficients */
FAST_CODE float biquadFilterApply(biquadFilter_t *filter, float input) {
    const float result = filter->b0 * input + filter->x1;
    filter->x1 = filter->b1 * input - filter->a1 * result + filter->x2;
    filter->x2 = filter->b2 * input - filter->a2 * result;
    return result;
}

// Robert Bouwens AlphaBetaGamma

void ABGInit(alphaBetaGammaFilter_t *filter, float alpha, int boostGain, int halfLife, float dT) {
	const float Alpha = alpha * 0.001f;
  // beta, gamma, and eta gains all derived from
  // http://yadda.icm.edu.pl/yadda/element/bwmeta1.element.baztech-922ff6cb-e991-417f-93f0-77448f1ef4ec/c/A_Study_Jeong_1_2017.pdf

  const float xi = powf(-Alpha + 1.0f, 0.25); // fourth rool of -a + 1
  filter->xk = 0.0f;
  filter->vk = 0.0f;
  filter->ak = 0.0f;
  filter->jk = 0.0f;
  filter->a = Alpha;
  filter->b = (1.0f / 6.0f) * powf(1.0f - xi, 2) * (11.0f + 14.0f * xi + 11 * xi * xi);
  filter->g = 2 * powf(1.0f - xi, 3) * (1 + xi);
  filter->e = (1.0f / 6.0f) * powf(1 - xi, 4);
	filter->dT = dT;
	filter->dT2 = dT * dT;
  filter->dT3 = dT * dT * dT;

  pt1FilterInit(&filter->boostFilter, pt1FilterGain(100, dT));

  filter->boost = (boostGain * boostGain / 1000000) * 0.003;
  filter->halfLife = halfLife != 0 ?
            powf(0.5f, dT / (halfLife / 100.0f)): 1.0f;

} // ABGInit

FAST_CODE float alphaBetaGammaApply(alphaBetaGammaFilter_t *filter, float input) {
	// float xk;   // current system state (ie: position)
	// float vk;   // derivative of system state (ie: velocity)
  // float ak;   // derivative of system velociy (ie: acceleration)
  // float jk;   // derivative of system acceleration (ie: jerk)
  float rk;   // residual error

  // give the filter limited history
  filter->xk *= filter->halfLife;
  filter->vk *= filter->halfLife;
  filter->ak *= filter->halfLife;
  filter->jk *= filter->halfLife;

  // update our (estimated) state 'x' from the system (ie pos = pos + vel (last).dT)
  filter->xk += filter->dT * filter->vk + (1.0f / 2.0f) * filter->dT2 * filter->ak + (1.0f / 6.0f) * filter->dT3 * filter->jk;
  // update (estimated) velocity (also estimated dterm from measurement)
  filter->vk += filter->dT * filter->ak + 0.5f * filter->dT2 * filter->jk;
  filter->ak += filter->dT * filter->jk;
  // what is our residual error (measured - estimated)
  rk = input - filter->xk;
  // artificially boost the error to increase the response of the filter
  rk += (fabsf(rk) * rk * filter->boost);
  filter->rk = rk; // for logging
  // update our estimates given the residual error.
  filter->xk += filter->a * rk;
  filter->vk += filter->b / filter->dT * rk;
  filter->ak += filter->g / (2.0f * filter->dT2) * rk;
  filter->jk += filter->e / (6.0f * filter->dT3) * rk;

	return filter->xk;
} // ABGUpdate
