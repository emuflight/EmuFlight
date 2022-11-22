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
#include <stdint.h>

#include "platform.h"

#if defined(USE_RPM_FILTER)

#include "build/debug.h"

#include "common/filter.h"
#include "common/maths.h"
#include "common/time.h"
#include "common/auto_notch.h"

#include "drivers/dshot.h"

#include "flight/mixer.h"
#include "flight/pid.h"

#include "pg/motor.h"

#include "scheduler/scheduler.h"

#include "sensors/gyro.h"

#include "rpm_filter.h"

#define RPM_FILTER_MAXHARMONICS 3
#define SECONDS_PER_MINUTE      60.0f
#define ERPM_PER_LSB            100.0f
#define MIN_UPDATE_T            0.001f


static pt1Filter_t rpmFilters[MAX_SUPPORTED_MOTORS];

typedef struct rpmNotchFilter_s {

    uint8_t  harmonics;
    float    minHz;
    float    maxHz;
    float    fadeRangeHz;
    float    q;
    timeUs_t looptimeUs;

    autoNotch_t notch[XYZ_AXIS_COUNT][MAX_SUPPORTED_MOTORS][RPM_FILTER_MAXHARMONICS];

} rpmNotchFilter_t;

FAST_DATA_ZERO_INIT static float   erpmToHz;
FAST_DATA_ZERO_INIT static float   filteredMotorErpm[MAX_SUPPORTED_MOTORS];
FAST_DATA_ZERO_INIT static float   motorFrequency[MAX_SUPPORTED_MOTORS];
FAST_DATA_ZERO_INIT static float   minMotorFrequency;
FAST_DATA_ZERO_INIT static uint8_t numberFilters;
FAST_DATA_ZERO_INIT static uint8_t numberRpmNotchFilters;
FAST_DATA_ZERO_INIT static uint8_t filterUpdatesPerIteration;
FAST_DATA_ZERO_INIT static float   pidLooptime;
FAST_DATA_ZERO_INIT static rpmNotchFilter_t filters[2];
FAST_DATA_ZERO_INIT static rpmNotchFilter_t *gyroFilter;

FAST_DATA_ZERO_INIT static uint8_t currentMotor;
FAST_DATA_ZERO_INIT static uint8_t currentHarmonic;
FAST_DATA_ZERO_INIT static uint8_t currentFilterNumber;
FAST_DATA static rpmNotchFilter_t *currentFilter = &filters[0];



PG_REGISTER_WITH_RESET_FN(rpmFilterConfig_t, rpmFilterConfig, PG_RPM_FILTER_CONFIG, 5);

void pgResetFn_rpmFilterConfig(rpmFilterConfig_t *config)
{
    config->rpm_filter_harmonics = 3;
    config->rpm_filter_min_hz = 100;
    config->rpm_filter_fade_range_hz = 50;
    config->rpm_filter_q = 500;

    config->rpm_filter_lpf_hz = 150;

    config->noise_limit = 30;
}

static void rpmNotchFilterInit(rpmNotchFilter_t *filter, const rpmFilterConfig_t *config, const timeUs_t looptimeUs)
{
    filter->harmonics = config->rpm_filter_harmonics;
    filter->minHz = config->rpm_filter_min_hz;
    filter->maxHz = 0.48f * 1e6f / looptimeUs; // don't go quite to nyquist to avoid oscillations
    filter->fadeRangeHz = config->rpm_filter_fade_range_hz;
    filter->q = config->rpm_filter_q / 100.0f;
    filter->looptimeUs = looptimeUs;

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        for (int motor = 0; motor < getMotorCount(); motor++) {
            for (int i = 0; i < filter->harmonics; i++) {
                initAutoNotch(
                    &filter->notch[axis][motor][i], filter->minHz * i, filter->q, config->noise_limit, filter->looptimeUs);
            }
        }
    }
}

void rpmFilterInit(const rpmFilterConfig_t *config)
{
    currentFilter = &filters[0];
    currentMotor = currentHarmonic = currentFilterNumber = 0;

    numberRpmNotchFilters = 0;
    if (!motorConfig()->dev.useDshotTelemetry) {
        gyroFilter = NULL;
        return;
    }

    pidLooptime = gyro.targetLooptime;
    if (config->rpm_filter_harmonics) {
        gyroFilter = &filters[numberRpmNotchFilters++];
        rpmNotchFilterInit(gyroFilter, config, pidLooptime);
    } else {
        gyroFilter = NULL;
    }

    for (int i = 0; i < getMotorCount(); i++) {
        pt1FilterInit(&rpmFilters[i], pt1FilterGain(config->rpm_filter_lpf_hz, pidLooptime * 1e-6f));
    }

    erpmToHz = ERPM_PER_LSB / SECONDS_PER_MINUTE  / (motorConfig()->motorPoleCount / 2.0f);

    const float loopIterationsPerUpdate = MIN_UPDATE_T / (pidLooptime * 1e-6f);
    numberFilters = getMotorCount() * (filters[0].harmonics + filters[1].harmonics);
    const float filtersPerLoopIteration = numberFilters / loopIterationsPerUpdate;
    filterUpdatesPerIteration = rintf(filtersPerLoopIteration + 0.49f);
}

static float applyFilter(rpmNotchFilter_t *filter, const int axis, float value)
{
    if (filter == NULL) {
        return value;
    }
    for (int motor = 0; motor < getMotorCount(); motor++) {
        for (int i = 0; i < filter->harmonics; i++) {
            value = applyAutoNotch(&filter->notch[axis][motor][i], value);
        }
    }
    return value;
}

float rpmFilterGyro(const int axis, float value)
{
    return applyFilter(gyroFilter, axis, value);
}

FAST_CODE_NOINLINE void rpmFilterUpdate(void)
{
    for (int motor = 0; motor < getMotorCount(); motor++) {
        filteredMotorErpm[motor] = pt1FilterApply(&rpmFilters[motor], getDshotTelemetry(motor));
        if (motor < 4) {
            DEBUG_SET(DEBUG_RPM_FILTER, motor, motorFrequency[motor]);
        }
        motorFrequency[motor] = erpmToHz * filteredMotorErpm[motor];
    }

    if (gyroFilter == NULL) {
        minMotorFrequency = 0.0f;
        return;
    }

    for (int i = 0; i < filterUpdatesPerIteration; i++) {

        float frequency = constrainf(
            (currentHarmonic + 1) * motorFrequency[currentMotor], currentFilter->minHz, currentFilter->maxHz);
        autoNotch_t *template = &currentFilter->notch[0][currentMotor][currentHarmonic];
        // uncomment below to debug filter stepping. Need to also comment out motor rpm DEBUG_SET above
        /* DEBUG_SET(DEBUG_RPM_FILTER, 0, harmonic); */
        /* DEBUG_SET(DEBUG_RPM_FILTER, 1, motor); */
        /* DEBUG_SET(DEBUG_RPM_FILTER, 2, currentFilter == &gyroFilter); */
        /* DEBUG_SET(DEBUG_RPM_FILTER, 3, frequency) */

        // fade out notch when approaching minHz (turn it off)
        float weight = 1.0f;
        if (frequency < currentFilter->minHz + currentFilter->fadeRangeHz) {
            weight = (frequency - currentFilter->minHz) / currentFilter->fadeRangeHz;
        }

        updateAutoNotch(
            template, frequency, currentFilter->q, weight, currentFilter->looptimeUs);

        for (int axis = 1; axis < XYZ_AXIS_COUNT; axis++) {
            autoNotch_t *clone = &currentFilter->notch[axis][currentMotor][currentHarmonic];
            clone->notchFilter.b0 = template->notchFilter.b0;
            clone->notchFilter.b1 = template->notchFilter.b1;
            clone->notchFilter.b2 = template->notchFilter.b2;
            clone->notchFilter.a1 = template->notchFilter.a1;
            clone->notchFilter.a2 = template->notchFilter.a2;

            updateWeight(clone, frequency, weight);
            }

        if (++currentHarmonic == currentFilter->harmonics) {
            currentHarmonic = 0;
            if (++currentFilterNumber == numberRpmNotchFilters) {
                currentFilterNumber = 0;
                if (++currentMotor == getMotorCount()) {
                    currentMotor = 0;
                }
                minMotorFrequency = 0.0f;
            }
            currentFilter = &filters[currentFilterNumber];
        }
    }
}

bool isRpmFilterEnabled(void)
{
    return (motorConfig()->dev.useDshotTelemetry && rpmFilterConfig()->rpm_filter_harmonics);
}

float rpmMinMotorFrequency(void)
{
    if (minMotorFrequency == 0.0f) {
        minMotorFrequency = 10000.0f; // max RPM reported in Hz = 600,000RPM
        for (int i = getMotorCount(); i--;) {
            if (motorFrequency[i] < minMotorFrequency) {
                minMotorFrequency = motorFrequency[i];
            }
        }
    }
    return minMotorFrequency;
}

#endif
