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

/* original work by Rav
 * 2018_07 updated by ctzsnooze to post filter, wider Q, different peak detection
 * coding assistance and advice from DieHertz, Rav, eTracer
 * test pilots icr4sh, UAV Tech, Flint723
 */
#include <stdint.h>

#include "platform.h"

#define USE_GYRO_DATA_ANALYSE

#ifdef USE_GYRO_DATA_ANALYSE
#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"
#include "common/sdft.h"
#include "common/time.h"
#include "common/utils.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/time.h"

#include "sensors/gyro.h"

#include "fc/core.h"

#include "gyroanalyse.h"

// SDFT_SAMPLE_SIZE defaults to 72 (common/sdft.h).
// We get 36 frequency bins from 72 consecutive data values, called SDFT_BIN_COUNT (common/sdft.h)
// Bin 0 is DC and can't be used.
// Only bins 1 to 35 are usable.

// A gyro sample is collected every PID loop.
// maxSampleCount recent gyro values are accumulated and averaged
// to ensure that 72 samples are collected at the right rate for the required SDFT bandwidth.

// For an 8k gyro loop, at default 600hz max, 6 sequential gyro data points are averaged, SDFT runs 1333Hz.
// Upper limit of SDFT is half that frequency, eg 666Hz by default.
// At 8k, if user sets a max of 300Hz, int(8000/600) = 13, sdftSampleRateHz = 615Hz, range 307Hz.
// Note that lower max requires more samples to be averaged, increasing precision but taking longer to get enough samples.
// For Bosch at 3200Hz gyro, max of 600, int(3200/1200) = 2, sdftSampleRateHz = 1600, range to 800hz.
// For Bosch on XClass, better to set a max of 300, int(3200/600) = 5, sdftSampleRateHz = 640, range to 320Hz.

// When sampleCount reaches maxSampleCount, the averaged gyro value is put into the corresponding SDFT.
// At 8k, with 600Hz max, maxSampleCount = 6, this happens every 6 * 0.125us, or every 0.75ms.
// Hence to completely replace all 72 samples of the SDFT input buffer with clean new data takes 54ms.

// The SDFT code is split into steps. It takes 4 gyro loops to calculate the SDFT, track peaks and update the filters for one axis.
// Since there are three axes, it takes 12 gyro loops to completely update all axes.
// At 8k, any one axis gets updated at 8000 / 12 or 666hz or every 1.5ms
// In this time, 2 points in the SDFT buffer will have changed.
// At 4k, it takes twice as long to update an axis, i.e. each axis updates only every 3ms.
// Four points in the buffer will have changed in that time, and each point will be the average of three samples.
// Hence output jitter at 4k is about four times worse than at 8k. At 2k output jitter is quite bad.

// Each SDFT output bin has width sdftSampleRateHz/72, ie 18.5Hz per bin at 1333Hz.
// Usable bandwidth is half this, ie 666Hz if sdftSampleRateHz is 1333Hz, i.e. bin 1 is 18.5Hz, bin 2 is 37.0Hz etc.

#define DYN_NOTCH_SMOOTH_HZ        4
#define DYN_NOTCH_CALC_TICKS       (XYZ_AXIS_COUNT * 4) // 4 steps per axis
#define DYN_NOTCH_OSD_MIN_THROTTLE 20

static sdft_t FAST_DATA_ZERO_INIT     sdft[XYZ_AXIS_COUNT];
static float FAST_DATA_ZERO_INIT      sdftData[SDFT_BIN_COUNT];
static uint16_t FAST_DATA_ZERO_INIT   sdftSampleRateHz;
static float FAST_DATA_ZERO_INIT      sdftResolution;
static uint8_t FAST_DATA_ZERO_INIT    sdftStartBin;
static uint8_t FAST_DATA_ZERO_INIT    sdftEndBin;
static float FAST_DATA_ZERO_INIT      dynNotchQ;
static float FAST_DATA_ZERO_INIT      dynNotch1Ctr;
static float FAST_DATA_ZERO_INIT      dynNotch2Ctr;
static uint16_t FAST_DATA_ZERO_INIT   dynNotchMinHz;
static uint16_t FAST_DATA_ZERO_INIT   dynNotchMaxHz;
static uint16_t FAST_DATA_ZERO_INIT   dynNotchMaxFFT;
static float FAST_DATA_ZERO_INIT      smoothFactor;
static uint8_t FAST_DATA_ZERO_INIT    numSamples;

void gyroDataAnalyseInit(uint32_t targetLooptimeUs)
{
#ifdef USE_MULTI_GYRO
    static bool gyroAnalyseInitialized;
    if (gyroAnalyseInitialized) {
        return;
    }
    gyroAnalyseInitialized = true;
#endif

    dynNotchMinHz = gyroConfig()->dyn_notch_min_hz;
    dynNotchMaxHz = MAX(2 * dynNotchMinHz, gyroConfig()->dyn_notch_max_hz);

    const int gyroLoopRateHz = lrintf((1.0f / targetLooptimeUs) * 1e6f);
    numSamples = MAX(1, gyroLoopRateHz / (2 * dynNotchMaxHz)); //600hz, 8k looptime, 13.333

    sdftSampleRateHz = gyroLoopRateHz / numSamples;
    // eg 8k, user max 600hz, int(8000/1200) = 6 (6.666), sdftSampleRateHz = 1333hz, range 666Hz
    // eg 4k, user max 600hz, int(4000/1200) = 3 (3.333), sdftSampleRateHz = 1333hz, range 666Hz
    // eg 2k, user max 600hz, int(2000/1200) = 1 (1.666) sdftSampleRateHz = 2000hz, range 1000Hz
    // eg 2k, user max 400hz, int(2000/800) = 2 (2.5) sdftSampleRateHz = 1000hz, range 500Hz
    // eg 1k, user max 600hz, int(1000/1200) = 1 (max(1,0.8333)) sdftSampleRateHz = 1000hz, range 500Hz
    // the upper limit of DN is always going to be Nyquist

    sdftResolution = (float)sdftSampleRateHz / SDFT_SAMPLE_SIZE; // 13.3hz per bin at 8k
    sdftStartBin = MAX(2, lrintf(dynNotchMinHz / sdftResolution + 0.5f)); // can't use bin 0 because it is DC.
    sdftEndBin = MIN(SDFT_BIN_COUNT - 1, lrintf(dynNotchMaxHz / sdftResolution + 0.5f)); // can't use more than SDFT_BIN_COUNT bins.
    smoothFactor = 2 * M_PIf * DYN_NOTCH_SMOOTH_HZ / (gyroLoopRateHz / 12); // minimum PT1 k value

    for (uint8_t axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        sdftInit(&sdft[axis], sdftStartBin, sdftEndBin);
    }
}

void gyroDataAnalyseStateInit(gyroAnalyseState_t *state, uint32_t targetLooptimeUs)
{
    // initialise even if FEATURE_DYNAMIC_FILTER not set, since it may be set later
    gyroDataAnalyseInit(targetLooptimeUs);
    state->maxSampleCount = numSamples;
    state->maxSampleCountRcp = 1.0f / state->maxSampleCount;
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // any init value
        state->centerFreq[axis] = dynNotchMaxHz;
    }
}

void gyroDataAnalysePush(gyroAnalyseState_t *state, const int axis, const float sample)
{
    state->oversampledGyroAccumulator[axis] += sample;
}

static void gyroDataAnalyseUpdate(gyroAnalyseState_t *state);

/*
 * Collect gyro data, to be analysed in gyroDataAnalyseUpdate function
 */
FAST_CODE void gyroDataAnalyse(gyroAnalyseState_t *state, biquadFilter_t *notchFilterDyn, biquadFilter_t *notchFilterDyn2)
{
    // samples should have been pushed by `gyroDataAnalysePush`
    // if gyro sampling is > 1kHz, accumulate and average multiple gyro samples
    state->sampleCount++;

    if (state->sampleCount == state->maxSampleCount) {
        state->sampleCount = 0;

        // calculate mean value of accumulated samples
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            const float sample = state->oversampledGyroAccumulator[axis] * state->maxSampleCountRcp;
            state->downsampledGyroData[axis] = sample;
            if (axis == 0) {
                DEBUG_SET(DEBUG_FFT, 2, lrintf(sample));
            }

            state->oversampledGyroAccumulator[axis] = 0;
        }

        // We need DYN_NOTCH_CALC_TICKS tick to update all axis with newly sampled value
        // recalculation of filters takes 4 calls per axis => each filter gets updated every DYN_NOTCH_CALC_TICKS calls
        // at 8kHz gyro loop rate this means 8kHz / 4 / 3 = 666Hz => update every 1.5ms
        // at 4kHz gyro loop rate this means 4kHz / 4 / 3 = 333Hz => update every 3ms
        state->updateTicks = DYN_NOTCH_CALC_TICKS;
    }

    // calculate SDFT and update filters
    if (state->updateTicks > 0) {
      gyroDataAnalyseUpdate(state);
        --state->updateTicks;
    }
}

/*
 * Analyse gyro data
 */
 static void gyroDataAnalyseUpdate(gyroAnalyseState_t *state)
{
    enum {
        STEP_SDFT,
        STEP_WINDOW,
        STEP_CALC_FREQUENCIES,
        STEP_UPDATE_FILTERS,
        STEP_COUNT
    };

    uint32_t startTime = 0;
    if (debugMode == (DEBUG_FFT_TIME)) {
        startTime = micros();
    }

    DEBUG_SET(DEBUG_FFT_TIME, 0, state->updateStep);
    switch (state->updateStep) {
        case STEP_SDFT:
        {
            sdftPush(&sdft[state->updateAxis], state->downsampledGyroData[state->updateAxis]);

            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);

            break;
        }
        case STEP_WINDOW:
        {
            sdftWinSq(&sdft[state->updateAxis], sdftData);

            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);

            break;
        }
        case STEP_CALC_FREQUENCIES:
        {
            // identify max bin and max/min heights
            float dataMax = 0.0f;
            float dataMin = 1.0f;
            uint8_t binMax = 0;
            float dataMinHi = 1.0f;

            // Search for biggest peak in frequency spectrum
            for (uint8_t bin = (sdftStartBin + 1); bin < sdftEndBin; bin++) {
                // Check if bin is peak
                if ((sdftData[bin] > sdftData[bin - 1]) && (sdftData[bin] > sdftData[bin + 1])) {
                    // Check if peak is biggest peak so far
                    if (sdftData[bin] > dataMax) {
                        dataMax = sdftData[bin];
                        binMax = bin;
                    }
                    bin++; // If bin is peak, next bin can't be peak => jump it
                }
            }
            // Search for pits on both sides of biggest peak
            if (binMax == 0) { // no peak found, hold prev max bin, dataMin = 1 dataMax = 0, ie move slow
                binMax = lrintf(state->centerFreq[state->updateAxis] / sdftResolution + 0.5f);
            } else { // there was a peak, find pits
                for (uint8_t bin = binMax - 1; bin > 1; bin--) { // look for min below max
                    if (sdftData[bin] < sdftData[bin - 1]) {
                        dataMin = sdftData[bin];
                        break;
                    }
                }
                for (uint8_t bin = binMax + 1; bin < SDFT_BIN_COUNT - 1; bin++) { // look for min above max
                    if (sdftData[bin] < sdftData[bin + 1]) {
                        dataMinHi = sdftData[bin];
                        break;
                    }
                }
            }
            dataMin = fminf(dataMin, dataMinHi);

            // accumulate sdftSum and sdftWeightedSum from peak bin, and shoulder bins either side of peak
            float squaredData = sdftData[binMax]; // sdftData already squared (see sdftWinSq)
            float sdftSum = squaredData;
            float sdftWeightedSum = squaredData * binMax;

            // accumulate upper shoulder unless it would be sdftEndBin
            uint8_t shoulderBin = binMax + 1;
            if (shoulderBin < sdftEndBin) {
                squaredData = sdftData[shoulderBin]; // sdftData already squared (see sdftWinSq)
                sdftSum += squaredData;
                sdftWeightedSum += squaredData * shoulderBin;
            }

            // accumulate lower shoulder unless lower shoulder would be bin 0 (DC)
            if (binMax > 1) {
                shoulderBin = binMax - 1;
                squaredData = sdftData[shoulderBin]; // sdftData already squared (see sdftWinSq)
                sdftSum += squaredData;
                sdftWeightedSum += squaredData * shoulderBin;
            }

            // get centerFreq in Hz from weighted bins
            float centerFreq = dynNotchMaxHz;
            float sdftMeanBin = 0;

            if (sdftSum > 0) {
                sdftMeanBin = (sdftWeightedSum / sdftSum);
                centerFreq = sdftMeanBin * sdftResolution;
                // In theory, the index points to the centre frequency of the bin.
                // at 1333hz, bin widths are 13.3Hz, so bin 2 (26.7Hz) has the range 20Hz to 33.3Hz
                // Rav feels that maybe centerFreq = (sdftMeanBin + 0.5) * sdftResolution is better
                // empirical checking shows that not adding 0.5 works better
            } else {
                centerFreq = state->centerFreq[state->updateAxis];
            }
            centerFreq = constrainf(centerFreq, dynNotchMinHz, dynNotchMaxHz);

            // PT1 style dynamic smoothing moves rapidly towards big peaks and slowly away, up to 8x faster
            const float dynamicFactor = constrainf(sqrt_approx(dataMax / dataMin), 1.0f, 8.0f);
            state->centerFreq[state->updateAxis] += smoothFactor * dynamicFactor * (centerFreq - state->centerFreq[state->updateAxis]);

            if(calculateThrottlePercentAbs() > DYN_NOTCH_OSD_MIN_THROTTLE) {
                dynNotchMaxFFT = MAX(dynNotchMaxFFT, state->centerFreq[state->updateAxis]);
            }

            if (state->updateAxis == 0) {
                DEBUG_SET(DEBUG_FFT, 3, lrintf(sdftMeanBin * 100));
                DEBUG_SET(DEBUG_FFT_FREQ, 0, state->centerFreq[state->updateAxis]);
                DEBUG_SET(DEBUG_FFT_FREQ, 1, lrintf(dynamicFactor * 100));
            }

            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);

            break;
        }
        case STEP_UPDATE_FILTERS:
        {
            // 7us
            state->filterUpdateExecute = true;
            state->filterUpdateAxis = state->updateAxis;
            state->filterUpdateFrequency = state->centerFreq[state->updateAxis];

            DEBUG_SET(DEBUG_FFT_TIME, 1, micros() - startTime);

            state->updateAxis = (state->updateAxis + 1) % XYZ_AXIS_COUNT;
        }
    }

    state->updateStep = (state->updateStep + 1) % STEP_COUNT;
}


uint16_t getMaxFFT(void) {
    return dynNotchMaxFFT;
}

void resetMaxFFT(void) {
    dynNotchMaxFFT = 0;
}

#endif // USE_GYRO_DATA_ANALYSE
