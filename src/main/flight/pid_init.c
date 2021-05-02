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

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"

#include "drivers/dshot_command.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/feedforward.h"
#include "flight/pid.h"
#include "flight/rpm_filter.h"

#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "pid_init.h"

#define ANTI_GRAVITY_THROTTLE_FILTER_CUTOFF 15  // The anti gravity throttle highpass filter cutoff
#define ANTI_GRAVITY_SMOOTH_FILTER_CUTOFF 3  // The anti gravity P smoothing filter cutoff

static void pidSetTargetLooptime(uint32_t pidLooptime)
{
    targetPidLooptime = pidLooptime;
    pidRuntime.dT = targetPidLooptime * 1e-6f;
    pidRuntime.pidFrequency = 1.0f / pidRuntime.dT;
#ifdef USE_DSHOT
    dshotSetPidLoopTime(targetPidLooptime);
#endif
}

void pidInitFilters(const pidProfile_t *pidProfile)
{
    STATIC_ASSERT(FD_YAW == 2, FD_YAW_incorrect); // ensure yaw axis is 2

    if (targetPidLooptime == 0) {
        // no looptime set, so set all the filters to null
        pidRuntime.dtermNotchApplyFn = nullFilterApply;
        pidRuntime.dtermLowpassApplyFn = nullFilterApply;
        pidRuntime.dtermLowpass2ApplyFn = nullFilterApply;
        pidRuntime.ptermYawLowpassApplyFn = nullFilterApply;
        pidRuntime.dtermABGApplyFn = nullFilterApply;
        return;
    }

    const uint32_t pidFrequencyNyquist = pidRuntime.pidFrequency / 2; // No rounding needed

    uint16_t dTermNotchHz;
    if (pidProfile->dterm_notch_hz <= pidFrequencyNyquist) {
        dTermNotchHz = pidProfile->dterm_notch_hz;
    } else {
        if (pidProfile->dterm_notch_cutoff < pidFrequencyNyquist) {
            dTermNotchHz = pidFrequencyNyquist;
        } else {
            dTermNotchHz = 0;
        }
    }

    if (dTermNotchHz != 0 && pidProfile->dterm_notch_cutoff != 0) {
        pidRuntime.dtermNotchApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(dTermNotchHz, pidProfile->dterm_notch_cutoff);
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            biquadFilterInit(&pidRuntime.dtermNotch[axis], dTermNotchHz, targetPidLooptime, notchQ, FILTER_NOTCH);
        }
    } else {
        pidRuntime.dtermNotchApplyFn = nullFilterApply;
    }

    //1st Dterm Lowpass Filter
    uint16_t dterm_lowpass_hz = pidProfile->dyn_lpf_dterm_min_hz;

    if (dterm_lowpass_hz > 0 && dterm_lowpass_hz < pidFrequencyNyquist) {
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        switch (pidProfile->dterm_filter_type) {
            case FILTER_BUTTERWORTH:
                pidRuntime.dtermLowpass2ApplyFn = (filterApplyFnPtr)biquadCascadeFilterApply;
                biquadFilterLpfCascadeInit(&pidRuntime.dtermLowpass[axis].butterworthFilter, pidProfile->dterm_filter_order, dterm_lowpass_hz, targetPidLooptime);
                break;
            case FILTER_PT:
                pidRuntime.dtermLowpassApplyFn = (filterApplyFnPtr)ptnFilterApply;
                ptnFilterInit(&pidRuntime.dtermLowpass[axis].ptnFilter, pidProfile->dterm_filter_order, dterm_lowpass_hz, pidRuntime.dT);
                break;
            default:
                pidRuntime.dtermLowpassApplyFn = nullFilterApply;
                break;
            }
        }
    } else {
        pidRuntime.dtermLowpassApplyFn = nullFilterApply;
    }

    //2nd Dterm Lowpass Filter
    if (pidProfile->dterm_lowpass2_hz == 0 || pidProfile->dterm_lowpass2_hz > pidFrequencyNyquist) {
        pidRuntime.dtermLowpass2ApplyFn = nullFilterApply;
    } else {
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        switch (pidProfile->dterm_filter2_type) {
            case FILTER_BUTTERWORTH:
                pidRuntime.dtermLowpass2ApplyFn = (filterApplyFnPtr)biquadCascadeFilterApply;
                biquadFilterLpfCascadeInit(&pidRuntime.dtermLowpass[axis].butterworthFilter, pidProfile->dterm_filter2_order, pidProfile->dterm_lowpass2_hz, targetPidLooptime);
                break;
            case FILTER_PT:
                pidRuntime.dtermLowpass2ApplyFn = (filterApplyFnPtr)ptnFilterApply;
                ptnFilterInit(&pidRuntime.dtermLowpass2[axis].ptnFilter, pidProfile->dterm_filter2_order, pidProfile->dterm_lowpass2_hz, pidRuntime.dT);
                break;
            default:
            pidRuntime.dtermLowpass2ApplyFn = nullFilterApply;
                break;
            }
        }
    }

    if (pidProfile->yaw_lowpass_hz == 0 || pidProfile->yaw_lowpass_hz > pidFrequencyNyquist) {
        pidRuntime.ptermYawLowpassApplyFn = nullFilterApply;
    } else {
        pidRuntime.ptermYawLowpassApplyFn = (filterApplyFnPtr)pt1FilterApply;
        pt1FilterInit(&pidRuntime.ptermYawLowpass, pt1FilterGain(pidProfile->yaw_lowpass_hz, pidRuntime.dT));
    }

    if (pidProfile->dtermAlpha == 0) {
        pidRuntime.dtermABGApplyFn = nullFilterApply;
    } else {
        pidRuntime.dtermABGApplyFn = (filterApplyFnPtr)alphaBetaGammaApply;
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            ABGInit(&pidRuntime.dtermABG[axis], pidProfile->dtermAlpha, pidProfile->dterm_abg_boost, pidProfile->dterm_abg_half_life, pidRuntime.dT);
        }
    }

#if defined(USE_THROTTLE_BOOST)
    pt1FilterInit(&throttleLpf, pt1FilterGain(pidProfile->throttle_boost_cutoff, pidRuntime.dT));
#endif

    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        pt1FilterInit(&pidRuntime.stickMovementLpf[i], pt1FilterGain(pidProfile->axis_lock_hz, pidRuntime.dT));
#if defined(USE_ITERM_RELAX)
        if (pidRuntime.itermRelaxCutoff || pidRuntime.itermRelaxCutoffYaw) {
            if (i != FD_YAW) {
                pt1FilterInit(&pidRuntime.windupLpf[i], pt1FilterGain(pidRuntime.itermRelaxCutoff, pidRuntime.dT));
            } else {
                pt1FilterInit(&pidRuntime.windupLpf[i], pt1FilterGain(pidRuntime.itermRelaxCutoffYaw, pidRuntime.dT));
            }
        }
#endif
    }

    pt1FilterInit(&pidRuntime.antiGravityThrottleLpf, pt1FilterGain(ANTI_GRAVITY_THROTTLE_FILTER_CUTOFF, pidRuntime.dT));
    pt1FilterInit(&pidRuntime.antiGravitySmoothLpf, pt1FilterGain(ANTI_GRAVITY_SMOOTH_FILTER_CUTOFF, pidRuntime.dT));

    pidRuntime.ffBoostFactor = (float)pidProfile->feedforward_boost / 10.0f;
}

void pidInit(const pidProfile_t *pidProfile)
{
    pidSetTargetLooptime(gyro.targetLooptime); // Initialize pid looptime
    pidInitFilters(pidProfile);
    pidInitConfig(pidProfile);
#ifdef USE_RPM_FILTER
    rpmFilterInit(rpmFilterConfig());
#endif
}

#ifdef USE_RC_SMOOTHING_FILTER
void pidInitFeedforwardLpf(uint16_t filterCutoff, uint8_t debugAxis)
{
    pidRuntime.rcSmoothingDebugAxis = debugAxis;
    if (filterCutoff > 0) {
        pidRuntime.feedforwardLpfInitialized = true;
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            ptnFilterInit(&pidRuntime.feedforwardPt3[axis], 3, filterCutoff, pidRuntime.dT);
        }
    }
}

void pidUpdateFeedforwardLpf(uint16_t filterCutoff)
{
    if (filterCutoff > 0) {
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            ptnFilterUpdate(&pidRuntime.feedforwardPt3[axis], filterCutoff, pidRuntime.dT);
        }
    }
}
#endif // USE_RC_SMOOTHING_FILTER

void pidInitConfig(const pidProfile_t *pidProfile)
{
    if (pidProfile->feedforwardTransition == 0) {
        pidRuntime.feedforwardTransition = 0;
    } else {
        pidRuntime.feedforwardTransition = 100.0f / pidProfile->feedforwardTransition;
    }
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        pidRuntime.pidCoefficient[axis].Kp = PTERM_SCALE * pidProfile->pid[axis].P;
        pidRuntime.pidCoefficient[axis].Ki = ITERM_SCALE * pidProfile->pid[axis].I;
        pidRuntime.pidCoefficient[axis].Kd = DTERM_SCALE * pidProfile->pid[axis].D;
        pidRuntime.pidCoefficient[axis].Kf = FEEDFORWARD_SCALE * (pidProfile->pid[axis].F / 100.0f);
        pidRuntime.pidCoefficient[axis].Kdf = 0;

        pidRuntime.dynThr[axis] = pidProfile->dynThr[axis];
        for (int pid = 0; pid <= 2; pid++) {
            pidRuntime.stickPositionTransition[pid][axis] = (pidProfile->stickTransition[pid][axis] / 100.0f) - 1.0f;
        }
        pidRuntime.axisLockScaler[axis] = 1.0f;
    }
    pidRuntime.pidCoefficient[FD_YAW].Kdf = DIRECT_FF_SCALE * pidProfile->pid[FD_YAW].DF;

    pidRuntime.dtermMeasurementSlider = pidProfile->dtermMeasurementSlider / 100;
    pidRuntime.dtermMeasurementSliderInverse = 1 - (pidProfile->dtermMeasurementSlider / 100);

    pidRuntime.emuBoostPR = (pidProfile->emuBoostPR * pidProfile->emuBoostPR / 1000000) * 0.003;
    pidRuntime.emuBoostY = (pidProfile->emuBoostY * pidProfile->emuBoostY / 1000000) * 0.003;
    pidRuntime.emuBoostLimitPR = powf(pidProfile->emuBoostPR, 0.75f) * 1.4;
    pidRuntime.emuBoostLimitY = powf(pidProfile->emuBoostY, 0.75f) * 1.4;
    pidRuntime.dtermBoost = (pidProfile->dtermBoost * pidProfile->dtermBoost / 1000000) * 0.003;
    pidRuntime.dtermBoostLimit = powf(pidProfile->dtermBoost, 0.75f) * 1.4;
    pidRuntime.iDecay = pidProfile->i_decay;
    pidRuntime.iDecayCutoff = pidProfile->i_decay_cutoff;

    pidRuntime.P_angle_low = pidProfile->pid[PID_LEVEL_LOW].P * 0.1f;
    pidRuntime.D_angle_low = pidProfile->pid[PID_LEVEL_LOW].D * 0.00017f;
    pidRuntime.DF_angle_low = pidProfile->pid[PID_LEVEL_LOW].DF * DIRECT_FF_SCALE;
    pidRuntime.P_angle_high = pidProfile->pid[PID_LEVEL_HIGH].P * 0.1f;
    pidRuntime.D_angle_high = pidProfile->pid[PID_LEVEL_HIGH].D * 0.00017f;
    pidRuntime.DF_angle_high = pidProfile->pid[PID_LEVEL_HIGH].DF * DIRECT_FF_SCALE;
    pidRuntime.F_angle = pidProfile->pid[PID_LEVEL_LOW].F * 0.00000125f;
    pidRuntime.horizonTransition = (float)pidProfile->horizonTransition;
    pidRuntime.horizonCutoffDegrees = pidProfile->horizon_tilt_effect;
    pidRuntime.racemodeHorizonTransitionFactor = pidRuntime.horizonCutoffDegrees / (pidRuntime.horizonCutoffDegrees - pidRuntime.horizonTransition);
    pidRuntime.horizonStrength = pidProfile->horizon_strength / 50.0f;
    pidRuntime.itermWindupPointInv = 0.0f;
    if (pidProfile->itermWindupPointPercent != 0) {
        const float itermWindupPoint = pidProfile->itermWindupPointPercent / 100.0f;
        pidRuntime.itermWindupPointInv = 1.0f / itermWindupPoint;
    }
    pidRuntime.itermAcceleratorGain = pidProfile->itermAcceleratorGain;
    pidRuntime.crashGyroThreshold = pidProfile->crash_gthreshold;
    pidRuntime.crashDtermThreshold = pidProfile->crash_dthreshold;
    pidRuntime.crashSetpointThreshold = pidProfile->crash_setpoint_threshold;
    pidRuntime.itermLimit = pidProfile->itermLimit;
#if defined(USE_THROTTLE_BOOST)
    throttleBoost = pidProfile->throttle_boost * 0.1f;
#endif
    pidRuntime.itermRotation = pidProfile->iterm_rotation;
    pidRuntime.antiGravityMode = pidProfile->antiGravityMode;

    // Calculate the anti-gravity value that will trigger the OSD display.
    // For classic AG it's either 1.0 for off and > 1.0 for on.
    // For the new AG it's a continuous floating value so we want to trigger the OSD
    // display when it exceeds 25% of its possible range. This gives a useful indication
    // of AG activity without excessive display.
    pidRuntime.antiGravityOsdCutoff = 0.0f;
    if (pidRuntime.antiGravityMode == ANTI_GRAVITY_SMOOTH) {
        pidRuntime.antiGravityOsdCutoff += (pidRuntime.itermAcceleratorGain / 1000.0f) * 0.25f;
    }
    pidRuntime.tpaBreakpoint = pidProfile->tpa_breakpoint;

#if defined(USE_ITERM_RELAX)
    pidRuntime.itermRelaxCutoff = pidProfile->iterm_relax_cutoff;
    pidRuntime.itermRelaxCutoffYaw = pidProfile->iterm_relax_cutoff_yaw;
    pidRuntime.itermRelaxThreshold = pidProfile->iterm_relax_threshold;
    pidRuntime.itermRelaxThresholdYaw = pidProfile->iterm_relax_threshold_yaw;
#endif

    pidRuntime.axisLockMultiplier = pidProfile->axis_lock_multiplier / 100.0f;
    pidRuntime.axisSmoothMultiplier = pidProfile->axis_smooth_multiplier / 100.0f;


#ifdef USE_DYN_LPF
    if (pidProfile->dyn_lpf_dterm_width > 0) {
        switch (pidProfile->dterm_filter_type) {
        case FILTER_BUTTERWORTH:
            pidRuntime.dynLpfFilter = DYN_LPF_BUTTERWORTH;
            break;
        case FILTER_PT:
            pidRuntime.dynLpfFilter = DYN_LPF_PT;
            break;
        default:
            pidRuntime.dynLpfFilter = DYN_LPF_NONE;
            break;
        }
    } else {
        pidRuntime.dynLpfFilter = DYN_LPF_NONE;
    }
    pidRuntime.dynLpfMin = pidProfile->dyn_lpf_dterm_min_hz;
    pidRuntime.dynLpfMax = pidProfile->dyn_lpf_dterm_min_hz + (pidProfile->dyn_lpf_dterm_min_hz * pidProfile->dyn_lpf_dterm_width / 100);
    pidRuntime.dynLpfCurveExpo = pidProfile->dyn_lpf_curve_expo;
    pidRuntime.dynLpf2Gain = pidProfile->dyn_lpf_dterm_gain;
    pidRuntime.dynLpf2Max = (pidProfile->dyn_lpf_dterm_min_hz * pidProfile->dyn_lpf_dterm_width / 100);
#endif

#ifdef USE_LAUNCH_CONTROL
    pidRuntime.launchControlMode = pidProfile->launchControlMode;
    if (sensors(SENSOR_ACC)) {
        pidRuntime.launchControlAngleLimit = pidProfile->launchControlAngleLimit;
    } else {
        pidRuntime.launchControlAngleLimit = 0;
    }
    pidRuntime.launchControlKi = ITERM_SCALE * pidProfile->launchControlGain;
#endif

#ifdef USE_THRUST_LINEARIZATION
    pidRuntime.thrustLinearization = pidProfile->thrustLinearization / 100.0f;
    pidRuntime.throttleCompensateAmount = pidRuntime.thrustLinearization - 0.5f * powf(pidRuntime.thrustLinearization, 2);
#endif
#ifdef USE_FEEDFORWARD
    pidRuntime.feedforwardAveraging = pidProfile->feedforward_averaging;
    if (pidProfile->feedforward_smooth_factor) {
        pidRuntime.ffSmoothFactor = 1.0f - ((float)pidProfile->feedforward_smooth_factor) / 100.0f;
    } else {
        // set automatically according to boost amount, limit to 0.5 for auto
        pidRuntime.ffSmoothFactor = MAX(0.5f, 1.0f - ((float)pidProfile->feedforward_boost) * 2.0f / 100.0f);
    }
    pidRuntime.ffJitterFactor = pidProfile->feedforward_jitter_factor;
    feedforwardInit(pidProfile);
#endif
}

void pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex)
{
    if (dstPidProfileIndex < PID_PROFILE_COUNT && srcPidProfileIndex < PID_PROFILE_COUNT
        && dstPidProfileIndex != srcPidProfileIndex) {
        memcpy(pidProfilesMutable(dstPidProfileIndex), pidProfilesMutable(srcPidProfileIndex), sizeof(pidProfile_t));
    }
}
