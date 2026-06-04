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

#pragma once
#include <stdbool.h>

struct filter_s;
typedef struct filter_s filter_t;

typedef struct pt1Filter_s {
    float state;
    float k;
} pt1Filter_t;

typedef struct slewFilter_s {
    float state;
    float slewLimit;
    float threshold;
} slewFilter_t;

/* this holds the data required to update samples thru a filter */
typedef struct biquadFilter_s {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} biquadFilter_t;

// SVF filter in TPT form — lowpass (Q fixed to Butterworth = 1/sqrt(2))
typedef struct svfLowpassFilter_s {
    float f;
    float a1;
    float a2;
    float ic1;
    float ic2;
} svfLowpassFilter_t;

// SVF filter in TPT form — notch (Q must remain constant across updates)
typedef struct svfNotchFilter_s {
    float a1;
    float a2q;  // a2 * q
    float fq;   // f * Q
    float ic1q; // state 1 scaled by q
    float ic2;
} svfNotchFilter_t;

typedef struct alphaBetaGammaFilter_s {
    float a, b, g, e;
    float ak, vk, xk, jk, rk;
    float dT, dT2, dT3;
    float halfLife, boost;
    pt1Filter_t boostFilter, velFilter, accFilter, jerkFilter;
} alphaBetaGammaFilter_t;

typedef struct ptnFilter_s {
    float state[5];
    float k;
    uint8_t order;
} ptnFilter_t;

typedef enum {
    FILTER_PT1 = 0,
    FILTER_SVF,      // replaces FILTER_BIQUAD at index 1 — same EEPROM value, CLI string stays "BIQUAD"
    FILTER_PT2,
    FILTER_PT3,
    FILTER_PT4,
} lowpassFilterType_e;
#define FILTER_BIQUAD FILTER_SVF  // backwards-compat alias for EF-specific code

typedef enum {
    FILTER_LPF,    // 2nd order Butterworth section
    FILTER_NOTCH,
    FILTER_BPF,
} biquadFilterType_e;


typedef float (*filterApplyFnPtr)(filter_t *filter, float input);

float nullFilterApply(filter_t *filter, float input);

void biquadFilterInitLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate);
void biquadFilterInit(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType);
void biquadFilterUpdate(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType);
void biquadFilterUpdateLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate);

float biquadFilterApplyDF1(biquadFilter_t *filter, float input);
float biquadFilterApply(biquadFilter_t *filter, float input);
float filterGetNotchQ(float centerFreq, float cutoffFreq);

void svfLowpassFilterInit(svfLowpassFilter_t *filter, float filterFreq, float dt);
void svfLowpassFilterUpdate(svfLowpassFilter_t *filter, float filterFreq, float dt);
float svfLowpassFilterApply(svfLowpassFilter_t *filter, float input);

void svfNotchInit(svfNotchFilter_t *filter, float filterFreq, float dt, float Q);
void svfNotchUpdate(svfNotchFilter_t *filter, float filterFreq, float dt, float Q);
float svfNotchApply(svfNotchFilter_t *filter, float input);

float pt1FilterGain(uint16_t f_cut, float dT);
void pt1FilterInit(pt1Filter_t *filter, float k);
void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k);
float pt1FilterApply(pt1Filter_t *filter, float input);

void slewFilterInit(slewFilter_t *filter, float slewLimit, float threshold);
float slewFilterApply(slewFilter_t *filter, float input);

void ABGInit(alphaBetaGammaFilter_t *filter, float alpha, int boostGain, int halfLife, float dT);
float alphaBetaGammaApply(alphaBetaGammaFilter_t *filter, float input);

void ptnFilterInit(ptnFilter_t *filter, uint8_t order, uint16_t f_cut, float dT);
void ptnFilterUpdate(ptnFilter_t *filter, float f_cut, float ScaleF, float dt);
float ptnFilterApply(ptnFilter_t *filter, float input);
