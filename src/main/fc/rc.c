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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/feature.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/feedforward.h"
#include "flight/gps_rescue.h"
#include "flight/pid_init.h"

#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "rc.h"


typedef float (applyRatesFn)(const int axis, float rcCommandf, const float rcCommandfAbs);

#ifdef USE_FEEDFORWARD
static float oldRcCommand[XYZ_AXIS_COUNT];
static bool isDuplicate[XYZ_AXIS_COUNT];
float rcCommandDelta[XYZ_AXIS_COUNT];
#endif
static float rawSetpoint[XYZ_AXIS_COUNT];
static float setpointRate[3], rcDeflection[3], rcDeflectionAbs[3];
static float throttlePAttenuation, throttleIAttenuation, throttleDAttenuation;
static bool reverseMotors = false;
static applyRatesFn *applyRates;
static uint16_t currentRxRefreshRate;
static bool isRxDataNew = false;
static float rcCommandDivider = 500.0f;
static float rcCommandYawDivider = 500.0f;

static FAST_DATA_ZERO_INIT bool newRxDataForFF;

enum {
    ROLL_FLAG = 1 << ROLL,
    PITCH_FLAG = 1 << PITCH,
    YAW_FLAG = 1 << YAW,
    THROTTLE_FLAG = 1 << THROTTLE,
};

#ifdef USE_RC_SMOOTHING_FILTER
#define RC_SMOOTHING_FILTER_STARTUP_DELAY_MS    5000  // Time to wait after power to let the PID loop stabilize before starting average frame rate calculation
#define RC_SMOOTHING_FILTER_TRAINING_SAMPLES    50    // Number of rx frame rate samples to average during initial training
#define RC_SMOOTHING_FILTER_RETRAINING_SAMPLES  20    // Number of rx frame rate samples to average during frame rate changes
#define RC_SMOOTHING_FILTER_TRAINING_DELAY_MS   1000  // Additional time to wait after receiving first valid rx frame before initial training starts
#define RC_SMOOTHING_FILTER_RETRAINING_DELAY_MS 2000  // Guard time to wait after retraining to prevent retraining again too quickly
#define RC_SMOOTHING_RX_RATE_CHANGE_PERCENT     20    // Look for samples varying this much from the current detected frame rate to initiate retraining
#define RC_SMOOTHING_RX_RATE_MIN_US             1000  // 1ms
#define RC_SMOOTHING_RX_RATE_MAX_US             50000 // 50ms or 20hz
#define RC_SMOOTHING_FEEDFORWARD_INITIAL_HZ     100 // The value to use for "auto" when interpolated feedforward is enabled

static FAST_DATA_ZERO_INIT rcSmoothingFilter_t rcSmoothingData;
#endif // USE_RC_SMOOTHING_FILTER

bool getShouldUpdateFeedforward()
// only used in pid.c, when feedforward is enabled, to initiate a new FF value
{
    const bool updateFf = newRxDataForFF;
    if (newRxDataForFF == true){
        newRxDataForFF = false;
    }
    return updateFf;
}

float getSetpointRate(int axis)
// only used in pid.c to provide setpointRate for the crash recovery function
{
    return setpointRate[axis];
}

float getRcDeflection(int axis)
{
    return rcDeflection[axis];
}

float getRcDeflectionAbs(int axis)
{
    return rcDeflectionAbs[axis];
}

float getThrottlePAttenuation(void)
{
    return throttlePAttenuation;
}

float getThrottleIAttenuation(void)
{
    return throttleIAttenuation;
}

float getThrottleDAttenuation(void)
{
    return throttleDAttenuation;
}

#ifdef USE_FEEDFORWARD
float getRawSetpoint(int axis)
{
    return rawSetpoint[axis];
}

float getRcCommandDelta(int axis)
{
    return rcCommandDelta[axis];
}
#endif

#define THROTTLE_LOOKUP_LENGTH 12
static int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];    // lookup table for expo & mid THROTTLE

static int16_t rcLookupThrottle(int32_t tmp)
{
    const int32_t tmp2 = tmp / 100;
    // [0;1000] -> expo -> [MINTHROTTLE;MAXTHROTTLE]
    return lookupThrottleRC[tmp2] + (tmp - tmp2 * 100) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 100;
}

#define SETPOINT_RATE_LIMIT 1998

#define RC_RATE_INCREMENTAL 14.54f

float applyBetaflightRates(const int axis, float rcCommandf, const float rcCommandfAbs)
{
    if (currentControlRateProfile->rcExpo[axis]) {
        const float expof = currentControlRateProfile->rcExpo[axis] / 100.0f;
        rcCommandf = rcCommandf * power3(rcCommandfAbs) * expof + rcCommandf * (1 - expof);
    }

    float rcRate = currentControlRateProfile->rcRates[axis] / 100.0f;
    if (rcRate > 2.0f) {
        rcRate += RC_RATE_INCREMENTAL * (rcRate - 2.0f);
    }
    float angleRate = 200.0f * rcRate * rcCommandf;
    if (currentControlRateProfile->rates[axis]) {
        const float rcSuperfactor = 1.0f / (constrainf(1.0f - (rcCommandfAbs * (currentControlRateProfile->rates[axis] / 100.0f)), 0.01f, 1.00f));
        angleRate *= rcSuperfactor;
    }

    return angleRate;
}

float applyRaceFlightRates(const int axis, float rcCommandf, const float rcCommandfAbs)
{
    float expo = 0.01f * currentControlRateProfile->rcExpo[axis];
    float rcRate = 10.0f * currentControlRateProfile->rcRates[axis];
    float acroPlus = currentControlRateProfile->rates[axis] * 0.01f;
    // -1.0 to 1.0 ranged and curved
    rcCommandf = ((1.0f + expo * (rcCommandf * rcCommandf - 1.0f)) * rcCommandf);
    // convert to -2000 to 2000 range using acro+ modifier
    float angleRate = rcCommandf * (rcRate + (rcCommandfAbs * rcRate * acroPlus));

    return angleRate;
}

float applyKissRates(const int axis, float rcCommandf, const float rcCommandfAbs)
{
    const float rcCurvef = currentControlRateProfile->rcExpo[axis] / 100.0f;

    float kissRpyUseRates = 1.0f / (constrainf(1.0f - (rcCommandfAbs * (currentControlRateProfile->rates[axis] / 100.0f)), 0.01f, 1.00f));
    float kissRcCommandf = (power3(rcCommandf) * rcCurvef + rcCommandf * (1 - rcCurvef)) * (currentControlRateProfile->rcRates[axis] / 1000.0f);
    float kissAngle = ((2000.0f * kissRpyUseRates) * kissRcCommandf);

    return kissAngle;
}

float applyActualRates(const int axis, float rcCommandf, const float rcCommandfAbs)
{
    float expof = currentControlRateProfile->rcExpo[axis] / 100.0f;
    expof = rcCommandfAbs * (powf(rcCommandf, 5) * expof + rcCommandf * (1 - expof));

    const float centerSensitivity = currentControlRateProfile->rcRates[axis] * 10.0f;
    const float stickMovement = MAX(0, currentControlRateProfile->rates[axis] * 10.0f - centerSensitivity);
    const float angleRate = rcCommandf * centerSensitivity + stickMovement * expof;

    return angleRate;
}

float applyCurve(int axis, float deflection)
{
    return applyRates(axis, deflection, fabsf(deflection));
}

static void scaleSetpointToFpvCamAngle(void)
{
    float currentPitchAngle = constrainf(attitude.raw[FD_PITCH] * 0.1f,-rxConfig()->fpvCamAngleDegrees,rxConfig()->fpvCamAngleDegrees);
    //recalculate sin/cos only when rxConfig()->fpvCamAngleDegrees changed
    static uint8_t lastFpvCamAngleDegrees = 0;
    static float cosFactor = 1.0;
    static float sinFactor = 0.0;

    if (rxConfig()->yawAroundGravity) {
        cosFactor = cos_approx(currentPitchAngle * RAD);
        sinFactor = sin_approx(currentPitchAngle * RAD);
    } else if (lastFpvCamAngleDegrees != rxConfig()->fpvCamAngleDegrees) {
        lastFpvCamAngleDegrees = rxConfig()->fpvCamAngleDegrees;
        cosFactor = cos_approx(rxConfig()->fpvCamAngleDegrees * RAD);
        sinFactor = sin_approx(rxConfig()->fpvCamAngleDegrees * RAD);
    }

    float roll = setpointRate[ROLL];
    float yaw = setpointRate[YAW];
    setpointRate[ROLL] = constrainf(roll * cosFactor -  yaw * sinFactor, -SETPOINT_RATE_LIMIT, SETPOINT_RATE_LIMIT);
    setpointRate[YAW]  = constrainf(yaw  * cosFactor + roll * sinFactor, -SETPOINT_RATE_LIMIT, SETPOINT_RATE_LIMIT);
}

#define THROTTLE_BUFFER_MAX 20
#define THROTTLE_DELTA_MS 100

static void checkForThrottleErrorResetState(uint16_t rxRefreshRate)
{
    static int index;
    static int16_t rcCommandThrottlePrevious[THROTTLE_BUFFER_MAX];

    const int rxRefreshRateMs = rxRefreshRate / 1000;
    const int indexMax = constrain(THROTTLE_DELTA_MS / rxRefreshRateMs, 1, THROTTLE_BUFFER_MAX);
    const int16_t throttleVelocityThreshold = (featureIsEnabled(FEATURE_3D)) ? currentPidProfile->itermThrottleThreshold / 2 : currentPidProfile->itermThrottleThreshold;

    rcCommandThrottlePrevious[index++] = rcCommand[THROTTLE];
    if (index >= indexMax) {
        index = 0;
    }

    const int16_t rcCommandSpeed = rcCommand[THROTTLE] - rcCommandThrottlePrevious[index];

    if (currentPidProfile->antiGravityMode == ANTI_GRAVITY_STEP) {
        if (ABS(rcCommandSpeed) > throttleVelocityThreshold) {
            pidSetItermAccelerator(CONVERT_PARAMETER_TO_FLOAT(currentPidProfile->itermAcceleratorGain));
        } else {
            pidSetItermAccelerator(0.0f);
        }
    }
}

void updateRcRefreshRate(timeUs_t currentTimeUs)
{
    static timeUs_t lastRxTimeUs;

    timeDelta_t frameAgeUs;
    timeDelta_t refreshRateUs = rxGetFrameDelta(&frameAgeUs);
    if (!refreshRateUs || cmpTimeUs(currentTimeUs, lastRxTimeUs) <= frameAgeUs) {
        refreshRateUs = cmpTimeUs(currentTimeUs, lastRxTimeUs); // calculate a delta here if not supplied by the protocol
    }
    lastRxTimeUs = currentTimeUs;
    currentRxRefreshRate = constrain(refreshRateUs, 1000, 30000);
}

uint16_t getCurrentRxRefreshRate(void)
{
    return currentRxRefreshRate;
}

#ifdef USE_RC_SMOOTHING_FILTER
// Determine a cutoff frequency based on smoothness factor and calculated average rx frame time
FAST_CODE_NOINLINE int calcAutoSmoothingCutoff(int avgRxFrameTimeUs, uint8_t autoSmoothnessFactor)
{
    if (avgRxFrameTimeUs > 0) {
        const float cutoffFactor = 1.5f / (1.0f + (autoSmoothnessFactor / 10.0f));
        float cutoff = (1 / (avgRxFrameTimeUs * 1e-6f));  // link frequency
        cutoff = cutoff * cutoffFactor;
        return lrintf(cutoff);
    } else {
        return 0;
    }
}

// Preforms a reasonableness check on the rx frame time to avoid bad data
// skewing the average.
static FAST_CODE bool rcSmoothingRxRateValid(int currentRxRefreshRate)
{
    return (currentRxRefreshRate >= RC_SMOOTHING_RX_RATE_MIN_US && currentRxRefreshRate <= RC_SMOOTHING_RX_RATE_MAX_US);
}

// Initialize or update the filters base on either the manually selected cutoff, or
// the auto-calculated cutoff frequency based on detected rx frame rate.
FAST_CODE_NOINLINE void rcSmoothingSetFilterCutoffs(rcSmoothingFilter_t *smoothingData)
{
    const float dT = targetPidLooptime * 1e-6f;
    uint16_t oldCutoff = smoothingData->setpointCutoffFrequency;

    if (smoothingData->setpointCutoffSetting == 0) {
        smoothingData->setpointCutoffFrequency = calcAutoSmoothingCutoff(smoothingData->averageFrameTimeUs, smoothingData->autoSmoothnessFactorSetpoint);
    }
    if (smoothingData->throttleCutoffSetting == 0) {
        smoothingData->throttleCutoffFrequency = calcAutoSmoothingCutoff(smoothingData->averageFrameTimeUs, smoothingData->autoSmoothnessFactorThrottle);
    }


    // initialize or update the Setpoint filter
    if ((smoothingData->setpointCutoffFrequency != oldCutoff) || !smoothingData->filterInitialized) {
        for (int i = 0; i < PRIMARY_CHANNEL_COUNT; i++) {
            if (i < THROTTLE) { // Throttle handled by smoothing rcCommand
                if (!smoothingData->filterInitialized) {
                    ptnFilterInit(&smoothingData->filter[i], 3, smoothingData->setpointCutoffFrequency, dT);
                } else {
                    ptnFilterUpdate(&smoothingData->filter[i], smoothingData->setpointCutoffFrequency, dT);
                }
            } else {
                if (!smoothingData->filterInitialized) {
                    ptnFilterInit((ptnFilter_t*) &smoothingData->filter[i], 3, smoothingData->setpointCutoffFrequency, dT);
                } else {
                    ptnFilterUpdate((ptnFilter_t*) &smoothingData->filter[i], smoothingData->setpointCutoffFrequency, dT);
                }
            }
        }
    }

    // update or initialize the FF filter
    oldCutoff = smoothingData->feedforwardCutoffFrequency;
    if (rcSmoothingData.ffCutoffSetting == 0) {
        smoothingData->feedforwardCutoffFrequency = calcAutoSmoothingCutoff(smoothingData->averageFrameTimeUs, smoothingData->autoSmoothnessFactorSetpoint);
    }
    if (!smoothingData->filterInitialized) {
        pidInitFeedforwardLpf(smoothingData->feedforwardCutoffFrequency, smoothingData->debugAxis);
    } else if (smoothingData->feedforwardCutoffFrequency != oldCutoff) {
        pidUpdateFeedforwardLpf(smoothingData->feedforwardCutoffFrequency);
    }
}

FAST_CODE_NOINLINE void rcSmoothingResetAccumulation(rcSmoothingFilter_t *smoothingData)
{
    smoothingData->training.sum = 0;
    smoothingData->training.count = 0;
    smoothingData->training.min = UINT16_MAX;
    smoothingData->training.max = 0;
}

// Accumulate the rx frame time samples. Once we've collected enough samples calculate the
// average and return true.
static FAST_CODE bool rcSmoothingAccumulateSample(rcSmoothingFilter_t *smoothingData, int rxFrameTimeUs)
{
    smoothingData->training.sum += rxFrameTimeUs;
    smoothingData->training.count++;
    smoothingData->training.max = MAX(smoothingData->training.max, rxFrameTimeUs);
    smoothingData->training.min = MIN(smoothingData->training.min, rxFrameTimeUs);

    // if we've collected enough samples then calculate the average and reset the accumulation
    const int sampleLimit = (rcSmoothingData.filterInitialized) ? RC_SMOOTHING_FILTER_RETRAINING_SAMPLES : RC_SMOOTHING_FILTER_TRAINING_SAMPLES;
    if (smoothingData->training.count >= sampleLimit) {
        smoothingData->training.sum = smoothingData->training.sum - smoothingData->training.min - smoothingData->training.max; // Throw out high and low samples
        smoothingData->averageFrameTimeUs = lrintf(smoothingData->training.sum / (smoothingData->training.count - 2));
        rcSmoothingResetAccumulation(smoothingData);
        return true;
    }
    return false;
}

// Determine if we need to caclulate filter cutoffs. If not then we can avoid
// examining the rx frame times completely
FAST_CODE_NOINLINE bool rcSmoothingAutoCalculate(void)
{
    // if any rc smoothing cutoff is 0 (auto) then we need to calculate cutoffs
    if ((rcSmoothingData.setpointCutoffSetting == 0) || (rcSmoothingData.ffCutoffSetting == 0) || (rcSmoothingData.throttleCutoffSetting == 0)) {
        return true;
    }
    return false;
}

static FAST_CODE void processRcSmoothingFilter(void)
{
    static FAST_DATA_ZERO_INIT float rxDataToSmooth[4];
    static FAST_DATA_ZERO_INIT bool initialized;
    static FAST_DATA_ZERO_INIT timeMs_t validRxFrameTimeMs;
    static FAST_DATA_ZERO_INIT bool calculateCutoffs;

    // first call initialization
    if (!initialized) {
        initialized = true;
        rcSmoothingData.filterInitialized = false;
        rcSmoothingData.averageFrameTimeUs = 0;
        rcSmoothingData.autoSmoothnessFactorSetpoint = rxConfig()->rc_smoothing_auto_factor_rpy;
        rcSmoothingData.autoSmoothnessFactorThrottle = rxConfig()->rc_smoothing_auto_factor_throttle;
        rcSmoothingData.debugAxis = rxConfig()->rc_smoothing_debug_axis;
        rcSmoothingData.setpointCutoffSetting = rxConfig()->rc_smoothing_setpoint_cutoff;
        rcSmoothingData.throttleCutoffSetting = rxConfig()->rc_smoothing_throttle_cutoff;
        rcSmoothingData.ffCutoffSetting = rxConfig()->rc_smoothing_feedforward_cutoff;
        rcSmoothingResetAccumulation(&rcSmoothingData);
        rcSmoothingData.setpointCutoffFrequency = rcSmoothingData.setpointCutoffSetting;
        rcSmoothingData.throttleCutoffFrequency = rcSmoothingData.throttleCutoffSetting;
        if (rcSmoothingData.ffCutoffSetting == 0) {
            // calculate and use an initial derivative cutoff until the RC interval is known
            const float cutoffFactor = 1.5f / (1.0f + (rcSmoothingData.autoSmoothnessFactorSetpoint / 10.0f));
            float ffCutoff = RC_SMOOTHING_FEEDFORWARD_INITIAL_HZ * cutoffFactor;
            rcSmoothingData.feedforwardCutoffFrequency = lrintf(ffCutoff);
        } else {
            rcSmoothingData.feedforwardCutoffFrequency = rcSmoothingData.ffCutoffSetting;
        }

        if (rxConfig()->rc_smoothing_mode) {
            calculateCutoffs = rcSmoothingAutoCalculate();

            // if we don't need to calculate cutoffs dynamically then the filters can be initialized now
            if (!calculateCutoffs) {
                rcSmoothingSetFilterCutoffs(&rcSmoothingData);
                rcSmoothingData.filterInitialized = true;
            }
        }
    }

    if (isRxDataNew) {
        // for auto calculated filters we need to examine each rx frame interval
        if (calculateCutoffs) {
            const timeMs_t currentTimeMs = millis();
            int sampleState = 0;

            // If the filter cutoffs in auto mode, and we have good rx data, then determine the average rx frame rate
            // and use that to calculate the filter cutoff frequencies
            if ((currentTimeMs > RC_SMOOTHING_FILTER_STARTUP_DELAY_MS) && (targetPidLooptime > 0)) { // skip during FC initialization
                if (rxIsReceivingSignal()  && rcSmoothingRxRateValid(currentRxRefreshRate)) {

                    // set the guard time expiration if it's not set
                    if (validRxFrameTimeMs == 0) {
                        validRxFrameTimeMs = currentTimeMs + (rcSmoothingData.filterInitialized ? RC_SMOOTHING_FILTER_RETRAINING_DELAY_MS : RC_SMOOTHING_FILTER_TRAINING_DELAY_MS);
                    } else {
                        sampleState = 1;
                    }

                    // if the guard time has expired then process the rx frame time
                    if (currentTimeMs > validRxFrameTimeMs) {
                        sampleState = 2;
                        bool accumulateSample = true;

                        // During initial training process all samples.
                        // During retraining check samples to determine if they vary by more than the limit percentage.
                        if (rcSmoothingData.filterInitialized) {
                            const float percentChange = (ABS(currentRxRefreshRate - rcSmoothingData.averageFrameTimeUs) / (float)rcSmoothingData.averageFrameTimeUs) * 100;
                            if (percentChange < RC_SMOOTHING_RX_RATE_CHANGE_PERCENT) {
                                // We received a sample that wasn't more than the limit percent so reset the accumulation
                                // During retraining we need a contiguous block of samples that are all significantly different than the current average
                                rcSmoothingResetAccumulation(&rcSmoothingData);
                                accumulateSample = false;
                            }
                        }

                        // accumlate the sample into the average
                        if (accumulateSample) {
                            if (rcSmoothingAccumulateSample(&rcSmoothingData, currentRxRefreshRate)) {
                                // the required number of samples were collected so set the filter cutoffs, but only if smoothing is active
                                if (rxConfig()->rc_smoothing_mode) {
                                    rcSmoothingSetFilterCutoffs(&rcSmoothingData);
                                    rcSmoothingData.filterInitialized = true;
                                }
                                validRxFrameTimeMs = 0;
                            }
                        }

                    }
                } else {
                    // we have either stopped receiving rx samples (failsafe?) or the sample time is unreasonable so reset the accumulation
                    rcSmoothingResetAccumulation(&rcSmoothingData);
                }
            }

            // rx frame rate training blackbox debugging
            if (debugMode == DEBUG_RC_SMOOTHING_RATE) {
                DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 0, currentRxRefreshRate);              // log each rx frame interval
                DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 1, rcSmoothingData.training.count);    // log the training step count
                DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 2, rcSmoothingData.averageFrameTimeUs);// the current calculated average
                DEBUG_SET(DEBUG_RC_SMOOTHING_RATE, 3, sampleState);                       // indicates whether guard time is active
            }
        }
        // Get new values to be smoothed
        for (int i = 0; i < PRIMARY_CHANNEL_COUNT; i++) {
            rxDataToSmooth[i] = i == THROTTLE ? rcCommand[i] : rawSetpoint[i];
            if (i < THROTTLE) {
                DEBUG_SET(DEBUG_RC_INTERPOLATION, i, lrintf(rxDataToSmooth[i]));
            } else {
                DEBUG_SET(DEBUG_RC_INTERPOLATION, i, ((lrintf(rxDataToSmooth[i])) - 1000));
            }
        }
    }

    if (rcSmoothingData.filterInitialized && (debugMode == DEBUG_RC_SMOOTHING)) {
        // after training has completed then log the raw rc channel and the calculated
        // average rx frame rate that was used to calculate the automatic filter cutoffs
        DEBUG_SET(DEBUG_RC_SMOOTHING, 3, rcSmoothingData.averageFrameTimeUs);
    }

    // each pid loop, apply the last received channel value to the filter, if initialised - thanks @klutvott
    for (int i = 0; i < PRIMARY_CHANNEL_COUNT; i++) {
        float *dst = i == THROTTLE ? &rcCommand[i] : &setpointRate[i];
        if (rcSmoothingData.filterInitialized) {
            *dst = ptnFilterApply(&rcSmoothingData.filter[i], rxDataToSmooth[i]);
        } else {
            // If filter isn't initialized yet, as in smoothing off, use the actual unsmoothed rx channel data
            *dst = rxDataToSmooth[i];
        }
    }
}
#endif // USE_RC_SMOOTHING_FILTER

FAST_CODE void processRcCommand(void)
{
    if (isRxDataNew) {
        newRxDataForFF = true;
    }

    if (isRxDataNew && pidAntiGravityEnabled()) {
        checkForThrottleErrorResetState(currentRxRefreshRate);
    }

    if (isRxDataNew) {
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {

            isDuplicate[axis] = (oldRcCommand[axis] == rcCommand[axis]);
            rcCommandDelta[axis] = fabsf(rcCommand[axis] - oldRcCommand[axis]);
            oldRcCommand[axis] = rcCommand[axis];

            float angleRate;

#ifdef USE_GPS_RESCUE
            if ((axis == FD_YAW) && FLIGHT_MODE(GPS_RESCUE_MODE)) {
                // If GPS Rescue is active then override the setpointRate used in the
                // pid controller with the value calculated from the desired heading logic.
                angleRate = gpsRescueGetYawRate();
                // Treat the stick input as centered to avoid any stick deflection base modifications (like acceleration limit)
                rcDeflection[axis] = 0;
                rcDeflectionAbs[axis] = 0;
            } else
#endif
            {
                // scale rcCommandf to range [-1.0, 1.0]
                float rcCommandf;
                if (axis == FD_YAW) {
                    rcCommandf = rcCommand[axis] / rcCommandYawDivider;
                } else {
                    rcCommandf = rcCommand[axis] / rcCommandDivider;
                }

                rcDeflection[axis] = rcCommandf;
                const float rcCommandfAbs = fabsf(rcCommandf);
                rcDeflectionAbs[axis] = rcCommandfAbs;

                angleRate = applyRates(axis, rcCommandf, rcCommandfAbs);

            }
            rawSetpoint[axis] = constrainf(angleRate, -1.0f * SETPOINT_RATE_LIMIT, 1.0f * SETPOINT_RATE_LIMIT);
            DEBUG_SET(DEBUG_ANGLERATE, axis, angleRate);
        }

        // adjust un-filtered setpoint steps to camera angle (mixing Roll and Yaw)
        if (rxConfig()->fpvCamAngleDegrees && IS_RC_MODE_ACTIVE(BOXFPVANGLEMIX) && !FLIGHT_MODE(HEADFREE_MODE)) {
            scaleSetpointToFpvCamAngle();
        }
    }

#ifdef USE_RC_SMOOTHING_FILTER
    processRcSmoothingFilter();
#endif

    isRxDataNew = false;
}

static void applyRollYawMix(void) {
    float rollAddition, yawAddition, unchangedRoll;

    unchangedRoll = rcCommand[FD_ROLL];
    yawAddition = rcCommand[FD_YAW] * (currentControlRateProfile->addYawToRollRc / 100.0f) * -GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);
    rcCommand[FD_ROLL] = constrainf((rcCommand[FD_ROLL] + yawAddition), -500.0f, 500.0f);

    rollAddition = unchangedRoll * (currentControlRateProfile->addRollToYawRc / 100.0f) * -GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);
    rcCommand[FD_YAW] = constrainf((rcCommand[FD_YAW] + rollAddition), -500.0f, 500.0f);
}

static void applyPolarExpo(void) {
    const float roll_pitch_mag = sqrtf((rcCommand[FD_ROLL] * rcCommand[FD_ROLL] / 250000.0f) + (rcCommand[FD_PITCH] * rcCommand[FD_PITCH] / 250000.0f));

    float roll_pitch_scale;
    const float rollPitchMagExpo = currentControlRateProfile->rollPitchMagExpo / 100.0f;
    if (roll_pitch_mag > 1.0f) {
        roll_pitch_scale = (1.0f / roll_pitch_mag);
        roll_pitch_scale = ((roll_pitch_scale - 1.0f) * rollPitchMagExpo) + 1.0f;
    } else {
        roll_pitch_scale = 1.0f;
    }

    rcCommand[FD_ROLL] *= roll_pitch_scale;
    rcCommand[FD_PITCH] *= roll_pitch_scale;
}

FAST_CODE_NOINLINE void updateRcCommands(void)
{
    isRxDataNew = true;

    // PITCH & ROLL only dynamic PID adjustment,  depending on throttle value
    int32_t propP, propI, propD;
    if (rcData[THROTTLE] < pidRuntime.tpaBreakpoint) {
        throttlePAttenuation = 1.0f;
        throttleIAttenuation = 1.0f;
        throttleDAttenuation = 1.0f;
    } else {
        float tpaMultiplier = (rcData[THROTTLE] - pidRuntime.tpaBreakpoint) / (2000 - pidRuntime.tpaBreakpoint);
        propP = 100 + ((uint16_t)pidRuntime.dynThr[0] - 100) * tpaMultiplier;
        propI = 100 + ((uint16_t)pidRuntime.dynThr[1] - 100) * tpaMultiplier;
        propD = 100 + ((uint16_t)pidRuntime.dynThr[2] - 100) * tpaMultiplier;
        throttlePAttenuation = propP / 100.0f;
        throttleIAttenuation = propI / 100.0f;
        throttleDAttenuation = propD / 100.0f;
    }

    for (int axis = 0; axis < 3; axis++) {
        // non coupled PID reduction scaler used in PID controller 1 and PID controller 2.

        float tmp = MIN(ABS(rcData[axis] - rxConfig()->midrc), 500);
        if (axis == ROLL || axis == PITCH) {
            if (tmp > rcControlsConfig()->deadband) {
                tmp -= rcControlsConfig()->deadband;
            } else {
                tmp = 0;
            }
            rcCommand[axis] = tmp;
        } else {
            if (tmp > rcControlsConfig()->yaw_deadband) {
                tmp -= rcControlsConfig()->yaw_deadband;
            } else {
                tmp = 0;
            }
            rcCommand[axis] = tmp * -GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);
        }
        if (rcData[axis] < rxConfig()->midrc) {
            rcCommand[axis] = -rcCommand[axis];
        }
      rcCommand[axis] = rateDynamics(rcCommand[axis], axis, currentRxRefreshRate);
    }

    applyPolarExpo();
    applyRollYawMix();

    if (rxConfig()->showAlteredRc != 0) {
        for (int axis = 0; axis < 3; axis++) {
            if (axis == ROLL || axis == PITCH) {
                rcData[axis] = rcCommand[axis] + rxConfig()->midrc;
            } else {
                rcData[axis] = (rcCommand[axis] * -GET_DIRECTION(rcControlsConfig()->yaw_control_reversed)) + rxConfig()->midrc;
            }
        }
    }

    int32_t tmp;
    if (featureIsEnabled(FEATURE_3D)) {
        tmp = constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX);
        tmp = (uint32_t)(tmp - PWM_RANGE_MIN);
    } else {
        tmp = constrain(rcData[THROTTLE], rxConfig()->mincheck, PWM_RANGE_MAX);
        tmp = (uint32_t)(tmp - rxConfig()->mincheck) * PWM_RANGE_MIN / (PWM_RANGE_MAX - rxConfig()->mincheck);
    }

    if (getLowVoltageCutoff()->enabled) {
        tmp = tmp * getLowVoltageCutoff()->percentage / 100;
    }

    rcCommand[THROTTLE] = rcLookupThrottle(tmp);

    if (featureIsEnabled(FEATURE_3D) && !failsafeIsActive()) {
        if (!flight3DConfig()->switched_mode3d) {
            if (IS_RC_MODE_ACTIVE(BOX3D)) {
                fix12_t throttleScaler = qConstruct(rcCommand[THROTTLE] - 1000, 1000);
                rcCommand[THROTTLE] = rxConfig()->midrc + qMultiply(throttleScaler, PWM_RANGE_MAX - rxConfig()->midrc);
            }
        } else {
            if (IS_RC_MODE_ACTIVE(BOX3D)) {
                reverseMotors = true;
                fix12_t throttleScaler = qConstruct(rcCommand[THROTTLE] - 1000, 1000);
                rcCommand[THROTTLE] = rxConfig()->midrc + qMultiply(throttleScaler, PWM_RANGE_MIN - rxConfig()->midrc);
            } else {
                reverseMotors = false;
                fix12_t throttleScaler = qConstruct(rcCommand[THROTTLE] - 1000, 1000);
                rcCommand[THROTTLE] = rxConfig()->midrc + qMultiply(throttleScaler, PWM_RANGE_MAX - rxConfig()->midrc);
            }
        }
    }
    if (FLIGHT_MODE(HEADFREE_MODE)) {
        static t_fp_vector_def  rcCommandBuff;

        rcCommandBuff.X = rcCommand[ROLL];
        rcCommandBuff.Y = rcCommand[PITCH];
        if ((!FLIGHT_MODE(ANGLE_MODE) && (!FLIGHT_MODE(HORIZON_MODE)) && (!FLIGHT_MODE(NFE_RACE_MODE)) && (!FLIGHT_MODE(GPS_RESCUE_MODE)))) {
            rcCommandBuff.Z = rcCommand[YAW];
        } else {
            rcCommandBuff.Z = 0;
        }
        imuQuaternionHeadfreeTransformVectorEarthToBody(&rcCommandBuff);
        rcCommand[ROLL] = rcCommandBuff.X;
        rcCommand[PITCH] = rcCommandBuff.Y;
        if ((!FLIGHT_MODE(ANGLE_MODE) && (!FLIGHT_MODE(HORIZON_MODE)) && (!FLIGHT_MODE(NFE_RACE_MODE)) && (!FLIGHT_MODE(GPS_RESCUE_MODE)))) {
            rcCommand[YAW] = rcCommandBuff.Z;
        }
    }
}

void resetYawAxis(void)
{
    rcCommand[YAW] = 0;
    setpointRate[YAW] = 0;
}

bool isMotorsReversed(void)
{
    return reverseMotors;
}

void initRcProcessing(void)
{
    rcCommandDivider = 500.0f - rcControlsConfig()->deadband;
    rcCommandYawDivider = 500.0f - rcControlsConfig()->yaw_deadband;

    for (int i = 0; i < THROTTLE_LOOKUP_LENGTH; i++) {
        const int16_t tmp = 10 * i - currentControlRateProfile->thrMid8;
        uint8_t y = 1;
        if (tmp > 0)
            y = 100 - currentControlRateProfile->thrMid8;
        if (tmp < 0)
            y = currentControlRateProfile->thrMid8;
        lookupThrottleRC[i] = 10 * currentControlRateProfile->thrMid8 + tmp * (100 - currentControlRateProfile->thrExpo8 + (int32_t) currentControlRateProfile->thrExpo8 * (tmp * tmp) / (y * y)) / 10;
        lookupThrottleRC[i] = PWM_RANGE_MIN + (PWM_RANGE_MAX - PWM_RANGE_MIN) * lookupThrottleRC[i] / 1000; // [MINTHROTTLE;MAXTHROTTLE]
    }

    switch (currentControlRateProfile->rates_type) {
    case RATES_TYPE_BETAFLIGHT:
    default:
        applyRates = applyBetaflightRates;

        break;
    case RATES_TYPE_RACEFLIGHT:
        applyRates = applyRaceFlightRates;

        break;
    case RATES_TYPE_KISS:
        applyRates = applyKissRates;

        break;
    case RATES_TYPE_ACTUAL:
        applyRates = applyActualRates;

        break;
    }

#ifdef USE_YAW_SPIN_RECOVERY
    const int maxYawRate = (int)applyRates(FD_YAW, 1.0f, 1.0f);
    initYawSpinRecovery(maxYawRate);
#endif
}

// send rc smoothing details to blackbox
#ifdef USE_RC_SMOOTHING_FILTER
rcSmoothingFilter_t *getRcSmoothingData(void)
{
    return &rcSmoothingData;
}

bool rcSmoothingInitializationComplete(void) {
    return rcSmoothingData.filterInitialized;
}
#endif // USE_RC_SMOOTHING_FILTER

FAST_CODE float calculateK(float k, int time) {
    if (k == 0.0f) {
        return 0;
    }
    // scale so it feels like running at 62.5hz (16ms) regardless of the current rx rate
    const float dT = time * 1e-6f;
    const float rxRate = 1.0f / dT;
    const float rxRateFactor = (rxRate / 62.5f) * rxRate;
    const float freq = k / ((1.0f / rxRateFactor) * (1.0f - k));
    const float RC = 1.0f / freq;

    return dT / (RC + dT);
}

FAST_CODE float rateDynamics(float rcCommand, int axis, int currentRxRefreshRate)
{
  static FAST_DATA_ZERO_INIT float lastRcCommandData[3];
  static FAST_DATA_ZERO_INIT float iterm[3];

  if (((currentControlRateProfile->rateDynamics.rateSensCenter != 100) || (currentControlRateProfile->rateDynamics.rateSensEnd != 100))
  || ((currentControlRateProfile->rateDynamics.rateWeightCenter > 0) || (currentControlRateProfile->rateDynamics.rateWeightEnd > 0)))
  {
    float pterm_centerStick, pterm_endStick, pterm, iterm_centerStick, iterm_endStick, dterm_centerStick, dterm_endStick, dterm;
    float rcCommandPercent;
    float rcCommandError;
    float rcCommandChange;
    float inverseRcCommandPercent;
    rcCommandPercent = fabsf(rcCommand) / 500.0f; // make rcCommandPercent go from 0 to 1
    inverseRcCommandPercent = 1.0f - rcCommandPercent;

    pterm_centerStick = inverseRcCommandPercent * rcCommand * (currentControlRateProfile->rateDynamics.rateSensCenter / 100.0f); // valid pterm values are between 50-150
    pterm_endStick = rcCommandPercent * rcCommand * (currentControlRateProfile->rateDynamics.rateSensEnd / 100.0f);
    pterm = pterm_centerStick + pterm_endStick;
    rcCommandError = rcCommand - (pterm + iterm[axis]);
    rcCommand = pterm; // add this fake pterm to the rcCommand

    iterm_centerStick = inverseRcCommandPercent * rcCommandError * calculateK(currentControlRateProfile->rateDynamics.rateCorrectionCenter / 100.0f, currentRxRefreshRate); // valid iterm values are between 0-95
    iterm_endStick = rcCommandPercent * rcCommandError * calculateK(currentControlRateProfile->rateDynamics.rateCorrectionEnd / 100.0f, currentRxRefreshRate);
    iterm[axis] += iterm_centerStick + iterm_endStick;
    rcCommand = rcCommand + iterm[axis]; // add the iterm to the rcCommand

    rcCommandChange = lastRcCommandData[axis] - rcCommand;
    dterm_centerStick = inverseRcCommandPercent * rcCommandChange * calculateK(currentControlRateProfile->rateDynamics.rateWeightCenter / 100.0f, currentRxRefreshRate); // valid dterm values are between 0-95
    dterm_endStick = rcCommandPercent * rcCommandChange * calculateK(currentControlRateProfile->rateDynamics.rateWeightEnd / 100.0f, currentRxRefreshRate);
    dterm = dterm_centerStick + dterm_endStick;

    rcCommand = rcCommand + dterm; // add dterm to the rcCommand (this is real dterm)
    lastRcCommandData[axis] = rcCommand;
  }
    return rcCommand;
}
