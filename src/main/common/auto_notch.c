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

#include <string.h>
#include "arm_math.h"

#include "auto_notch.h"
#include "build/debug.h"
#include "maths.h"
/*
 * The idea is simple, run a passband at the notch frequency to isolate noise
 * at the notch frequency. Then look at the averaged squared rate of change over
 * a period of time great enough to cover at least 1 full wave of noise at notch
 * frequency. This way we can measure noise and compare noise of pre/post notch.
 * this allows us to crossfade based on noise.
 */


void init(autoNotch_t *autoNotch, float initial_frequency, int q, int noiseLimit, float looptimeUs) {
    // creates an exponential moving average that will cover the freq we are nothing/bandpassing
    float adjustedQ = q / 100.0f;

    autoNotch->noiseLimit = noiseLimit;
    autoNotch->weight = 1.0;
    autoNotch->invWeight = 0.0;

    autoNotch->preVariance = 0.0;
    pt1FilterInit(&autoNotch->preVarianceFilter, pt1FilterGain(10.0, looptimeUs * 1e-6f));
    biquadFilterInit(&autoNotch->preVarianceBandpass, initial_frequency, looptimeUs, adjustedQ, FILTER_BPF, 1.0f);

    biquadFilterInit(&autoNotch->notchFilter, initial_frequency, looptimeUs, adjustedQ, FILTER_NOTCH, 1.0f);
}

float apply(autoNotch_t *autoNotch, float input) {
    float preNotchNoise = biquadFilterApplyDF1(&autoNotch->preVarianceBandpass, input);
    // variance is approximately the noise squared and averaged
    autoNotch->preVariance = pt1FilterApply(&autoNotch->preVarianceFilter, preNotchNoise * preNotchNoise);

    float notchFilteredNoise = biquadFilterApplyDF1(&autoNotch->notchFilter, input);

    return autoNotch->weight * notchFilteredNoise + autoNotch->invWeight * input;
}

void updateAutoNotch(autoNotch_t *autoNotch, float frequency, float q, float weightMultiplier, float looptimeUs) {
    biquadFilterInit(&autoNotch->preVarianceBandpass, frequency, looptimeUs, q, FILTER_BPF, 1.0f);
    biquadFilterUpdate(&autoNotch->notchFilter, frequency, looptimeUs, q, FILTER_NOTCH, 1.0f);

    float deviation;
    arm_sqrt_f32(autoNotch->preVariance, &deviation);
    float weight = deviation / autoNotch->noiseLimit;

    autoNotch->weight = MIN(weight * weightMultiplier, 1.0);
    autoNotch->invWeight = 1.0 - autoNotch->weight;
}
