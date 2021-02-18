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
#include "common/maths.h"
#include "common/filter.h"

#include "config/config_reset.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/sound_beeper.h"
#include "drivers/time.h"

#include "fc/controlrate_profile.h"

#include "fc/fc_core.h"
#include "fc/fc_rc.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/gps_rescue.h"
#include "flight/mixer.h"

#include "io/gps.h"

#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#include "sensors/battery.h"

const char pidNames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "LEVEL_L;"
    "LEVEL_H;"
    "MAG;";

FAST_RAM_ZERO_INIT uint32_t targetPidLooptime;
FAST_RAM_ZERO_INIT pidAxisData_t pidData[XYZ_AXIS_COUNT];

static FAST_RAM_ZERO_INIT bool pidStabilisationEnabled;

static FAST_RAM_ZERO_INIT bool inCrashRecoveryMode = false;

static FAST_RAM_ZERO_INIT float dT;
static FAST_RAM_ZERO_INIT float pidFrequency;
extern struct pidProfile_s *currentPidProfile;
extern bool linearThrustEnabled;

PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 2);

#ifdef STM32F10X
#define PID_PROCESS_DENOM_DEFAULT 1
#elif defined(USE_GYRO_SPI_MPU6000) || defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_ICM20689)
#define PID_PROCESS_DENOM_DEFAULT 1
#else
#define PID_PROCESS_DENOM_DEFAULT 2
#endif

#ifndef DEFAULT_PIDS_ROLL
#define DEFAULT_PIDS_ROLL {50, 70, 28, 0}
#endif //DEFAULT_PIDS_ROLL

#ifndef DEFAULT_PIDS_PITCH
#define DEFAULT_PIDS_PITCH {58, 70, 30, 0}
#endif //DEFAULT_PIDS_PITCH

#ifndef DEFAULT_PIDS_YAW
#define DEFAULT_PIDS_YAW {60, 70, 5, 0}
#endif //DEFAULT_PIDS_YAW

#ifdef USE_RUNAWAY_TAKEOFF
PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
                  .pid_process_denom = PID_PROCESS_DENOM_DEFAULT,
                  .runaway_takeoff_prevention = true,
                  .runaway_takeoff_deactivate_throttle = 20, // throttle level % needed to accumulate deactivation time
                  .runaway_takeoff_deactivate_delay = 500    // Accumulated time (in milliseconds) before deactivation in successful takeoff
                 );
#else
PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
                  .pid_process_denom = PID_PROCESS_DENOM_DEFAULT);
#endif

PG_REGISTER_ARRAY_WITH_RESET_FN(pidProfile_t, PID_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 8);

void resetPidProfile(pidProfile_t *pidProfile) {
    RESET_CONFIG(pidProfile_t, pidProfile,
    .pid = {
        [PID_ROLL] = DEFAULT_PIDS_ROLL,
        [PID_PITCH] = DEFAULT_PIDS_PITCH,
        [PID_YAW] = DEFAULT_PIDS_YAW,
        [PID_LEVEL_LOW] = {100, 70, 10, 40},
        [PID_LEVEL_HIGH] = {35, 0, 1, 0},
        [PID_MAG] = {40, 0, 0, 0},
    },
    .dFilter = {
        [PID_ROLL] = {2, 65, 200, 0},  // wc, dtermlpf, dtermlpf2, smartSmoothing
        [PID_PITCH] = {2, 65, 200, 0}, // wc, dtermlpf, dtermlpf2, smartSmoothing
        [PID_YAW] = {0, 65, 200, 0},    // wc, dtermlpf, dtermlpf2, smartSmoothing
    },
    .pidSumLimit = PIDSUM_LIMIT_MAX,
    .pidSumLimitYaw = PIDSUM_LIMIT_YAW,
    .dterm_filter_type = FILTER_PT1,
    .itermWindupPointPercent = 70,
    .pidAtMinThrottle = PID_STABILISATION_ON,
    .levelAngleLimit = 45,
    .angleExpo = 10,
    .setPointPTransition[ROLL] = 110,
    .setPointPTransition[PITCH] = 110,
    .setPointPTransition[YAW] = 130,
    .setPointITransition[ROLL] = 85,
    .setPointITransition[PITCH] = 85,
    .setPointITransition[YAW] = 70,
    .setPointDTransition[ROLL] = 110,
    .setPointDTransition[PITCH] = 110,
    .setPointDTransition[YAW] = 130,
    .feathered_pids = 100,
    .i_decay = 4,
    .i_decay_cutoff = 200,                   // value of 0 mimicks old i_decay behaviour
    .errorBoost = 15,
    .errorBoostYaw = 40,
    .errorBoostLimit = 20,
    .errorBoostLimitYaw = 40,
    .dtermBoost = 0,
    .dtermBoostLimit = 0,
    .yawRateAccelLimit = 0,
    .rateAccelLimit = 0,
    .crash_time = 500,                        // ms
    .crash_delay = 0,                         // ms
    .crash_recovery_angle = 10,               // degrees
    .crash_recovery_rate = 100,               // degrees/second
    .crash_dthreshold = 50,                   // degrees/second/second
    .crash_gthreshold = 400,                  // degrees/second
    .crash_setpoint_threshold = 350,          // degrees/second
    .crash_recovery = PID_CRASH_RECOVERY_OFF, // off by default
    .crash_limit_yaw = 200,
    .itermLimit = 400,
    .throttle_boost = 5,
    .throttle_boost_cutoff = 15,
    .iterm_rotation = true,
    .iterm_relax_cutoff = 11,
    .iterm_relax_cutoff_yaw = 25,
    .iterm_relax_threshold = 35,
    .iterm_relax_threshold_yaw = 35,
    .motor_output_limit = 100,
    .auto_profile_cell_count = AUTO_PROFILE_CELL_COUNT_STAY,
    .horizon_tilt_effect = 80,
    .horizonTransition = 0,
    .axis_lock_hz = 2,
    .axis_lock_multiplier = 0,
    .linear_thrust_low_output = 65,
    .linear_thrust_high_output = 0,
    .linear_throttle = false,
    .mixer_impl = MIXER_IMPL_LEGACY,
    .mixer_laziness = false,
    .horizonStrength = 15,
    .directFF_yaw = 15,
    .emuGravityGain = 100,

                );
}

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles) {
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPidProfile(&pidProfiles[i]);
    }
}

static void pidSetTargetLooptime(uint32_t pidLooptime) {
    targetPidLooptime = pidLooptime;
    dT = targetPidLooptime * 1e-6f;
    pidFrequency = 1.0f / dT;
}

void pidStabilisationState(pidStabilisationState_e pidControllerState) {
    pidStabilisationEnabled = (pidControllerState == PID_STABILISATION_ON) ? true : false;
}

const angle_index_t rcAliasToAngleIndexMap[] = {AI_ROLL, AI_PITCH};

typedef union dtermLowpass_u {
    pt1Filter_t pt1Filter;
    biquadFilter_t biquadFilter;
} dtermLowpass_t;

static FAST_RAM_ZERO_INIT float previousPidSetpoint[XYZ_AXIS_COUNT];
static FAST_RAM filterApplyFnPtr dtermLowpassApplyFn = nullFilterApply;
static FAST_RAM_ZERO_INIT dtermLowpass_t dtermLowpass[XYZ_AXIS_COUNT];
static FAST_RAM filterApplyFnPtr dtermLowpass2ApplyFn = nullFilterApply;
static FAST_RAM_ZERO_INIT dtermLowpass_t dtermLowpass2[XYZ_AXIS_COUNT];

#if defined(USE_ITERM_RELAX)
static FAST_RAM_ZERO_INIT pt1Filter_t windupLpf[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT uint8_t itermRelaxCutoff;
static FAST_RAM_ZERO_INIT uint8_t itermRelaxCutoffYaw;
#endif

static FAST_RAM_ZERO_INIT pt1Filter_t emuGravityThrottleLpf;
static FAST_RAM_ZERO_INIT pt1Filter_t axisLockLpf[XYZ_AXIS_COUNT];

static FAST_RAM_ZERO_INIT float iDecay;

#ifdef USE_RC_SMOOTHING_FILTER
static FAST_RAM_ZERO_INIT pt1Filter_t setpointDerivativePt1[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT biquadFilter_t setpointDerivativeBiquad[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT bool setpointDerivativeLpfInitialized;
static FAST_RAM_ZERO_INIT uint8_t rcSmoothingDebugAxis;
static FAST_RAM_ZERO_INIT uint8_t rcSmoothingFilterType;
#endif // USE_RC_SMOOTHING_FILTER

void pidInitFilters(const pidProfile_t *pidProfile) {
    BUILD_BUG_ON(FD_YAW != 2);                             // ensure yaw axis is 2
    const uint32_t pidFrequencyNyquist = pidFrequency / 2; // No rounding needed
    dtermLowpassApplyFn = nullFilterApply;
    dtermLowpass2ApplyFn = nullFilterApply;
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        if (pidProfile->dFilter[axis].dLpf && pidProfile->dFilter[axis].dLpf <= pidFrequencyNyquist) {
            switch (pidProfile->dterm_filter_type) {
            case FILTER_PT1:
                dtermLowpassApplyFn = (filterApplyFnPtr)pt1FilterApply;
                pt1FilterInit(&dtermLowpass[axis].pt1Filter, pt1FilterGain(pidProfile->dFilter[axis].dLpf, dT));
                break;
            case FILTER_BIQUAD:
            default:
                dtermLowpassApplyFn = (filterApplyFnPtr)biquadFilterApply;
                biquadFilterInitLPF(&dtermLowpass[axis].biquadFilter, pidProfile->dFilter[axis].dLpf, targetPidLooptime);
                break;
            }
        }
        if (pidProfile->dFilter[axis].dLpf2 && pidProfile->dFilter[axis].dLpf2 <= pidFrequencyNyquist) {
            switch (pidProfile->dterm_filter_type) {
            case FILTER_PT1:
                dtermLowpass2ApplyFn = (filterApplyFnPtr)pt1FilterApply;
                pt1FilterInit(&dtermLowpass2[axis].pt1Filter, pt1FilterGain(pidProfile->dFilter[axis].dLpf2, dT));
                break;
            case FILTER_BIQUAD:
            default:
                dtermLowpass2ApplyFn = (filterApplyFnPtr)biquadFilterApply;
                biquadFilterInitLPF(&dtermLowpass2[axis].biquadFilter, pidProfile->dFilter[axis].dLpf2, targetPidLooptime);
                break;
            }
        }
    }
#if defined(USE_THROTTLE_BOOST)
    pt1FilterInit(&throttleLpf, pt1FilterGain(pidProfile->throttle_boost_cutoff, dT));
#endif

    pt1FilterInit(&emuGravityThrottleLpf, pt1FilterGain(EMU_GRAVITY_THROTTLE_FILTER_CUTOFF, dT));

    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        pt1FilterInit(&axisLockLpf[i], pt1FilterGain(pidProfile->axis_lock_hz, dT));
#if defined(USE_ITERM_RELAX)
        if (i != FD_YAW) {
            pt1FilterInit(&windupLpf[i], pt1FilterGain(itermRelaxCutoff, dT));
        } else {
            pt1FilterInit(&windupLpf[i], pt1FilterGain(itermRelaxCutoffYaw, dT));
        }
#endif
    }
}



#ifdef USE_RC_SMOOTHING_FILTER
void pidInitSetpointDerivativeLpf(uint16_t filterCutoff, uint8_t debugAxis, uint8_t filterType) {
    rcSmoothingDebugAxis = debugAxis;
    rcSmoothingFilterType = filterType;
    if ((filterCutoff > 0) && (rcSmoothingFilterType != RC_SMOOTHING_DERIVATIVE_OFF)) {
        setpointDerivativeLpfInitialized = true;
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            switch (rcSmoothingFilterType) {
            case RC_SMOOTHING_DERIVATIVE_PT1:
                pt1FilterInit(&setpointDerivativePt1[axis], pt1FilterGain(filterCutoff, dT));
                break;
            case RC_SMOOTHING_DERIVATIVE_BIQUAD:
                biquadFilterInitLPF(&setpointDerivativeBiquad[axis], filterCutoff, targetPidLooptime);
                break;
            }
        }
    }
}

void pidUpdateSetpointDerivativeLpf(uint16_t filterCutoff) {
    if ((filterCutoff > 0) && (rcSmoothingFilterType != RC_SMOOTHING_DERIVATIVE_OFF)) {
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            switch (rcSmoothingFilterType) {
            case RC_SMOOTHING_DERIVATIVE_PT1:
                pt1FilterUpdateCutoff(&setpointDerivativePt1[axis], pt1FilterGain(filterCutoff, dT));
                break;
            case RC_SMOOTHING_DERIVATIVE_BIQUAD:
                biquadFilterUpdateLPF(&setpointDerivativeBiquad[axis], filterCutoff, targetPidLooptime);
                break;
            }
        }
    }
}
#endif // USE_RC_SMOOTHING_FILTER

typedef struct pidCoefficient_s {
    float Kp;
    float Ki;
    float Kd;
    float Kf;
} pidCoefficient_t;

static FAST_RAM_ZERO_INIT pidCoefficient_t pidCoefficient[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float directFF[3];
static FAST_RAM_ZERO_INIT float maxVelocity[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float feathered_pids;
static FAST_RAM_ZERO_INIT float smart_dterm_smoothing[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float setPointPTransition[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float setPointITransition[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float setPointDTransition[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float dtermBoostMultiplier, dtermBoostLimitPercent;
static FAST_RAM_ZERO_INIT float P_angle_low, D_angle_low, P_angle_high, D_angle_high, F_angle, DF_angle_low, DF_angle_high, horizonTransition, horizonCutoffDegrees, horizonStrength;
static FAST_RAM_ZERO_INIT float ITermWindupPointInv;
static FAST_RAM_ZERO_INIT timeDelta_t crashTimeLimitUs;
static FAST_RAM_ZERO_INIT timeDelta_t crashTimeDelayUs;
static FAST_RAM_ZERO_INIT int32_t crashRecoveryAngleDeciDegrees;
static FAST_RAM_ZERO_INIT float crashRecoveryRate;
static FAST_RAM_ZERO_INIT float crashDtermThreshold;
static FAST_RAM_ZERO_INIT float crashGyroThreshold;
static FAST_RAM_ZERO_INIT float crashSetpointThreshold;
static FAST_RAM_ZERO_INIT float crashLimitYaw;
static FAST_RAM_ZERO_INIT float itermLimit;
static FAST_RAM_ZERO_INIT float axisLockMultiplier;
#if defined(USE_THROTTLE_BOOST)
FAST_RAM_ZERO_INIT float throttleBoost;
pt1Filter_t throttleLpf;
#endif
static FAST_RAM_ZERO_INIT bool itermRotation;
static FAST_RAM_ZERO_INIT float temporaryIterm[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float emuGravityThrottleHpf;

void pidResetITerm(void) {
    for (int axis = 0; axis < 3; axis++) {
        temporaryIterm[axis] = 0.0f;
    }
}

void pidUpdateEmuGravityThrottleFilter(float throttle) {
      emuGravityThrottleHpf = throttle - pt1FilterApply(&emuGravityThrottleLpf, throttle);
}

void pidInitConfig(const pidProfile_t *pidProfile) {
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        pidCoefficient[axis].Kp = PTERM_SCALE * pidProfile->pid[axis].P;
        pidCoefficient[axis].Ki = ITERM_SCALE * pidProfile->pid[axis].I;
        pidCoefficient[axis].Kd = DTERM_SCALE * pidProfile->pid[axis].D;
        setPointPTransition[axis] = pidProfile->setPointPTransition[axis] / 100.0f;
        setPointITransition[axis] = pidProfile->setPointITransition[axis] / 100.0f;
        setPointDTransition[axis] = pidProfile->setPointDTransition[axis] / 100.0f;
        smart_dterm_smoothing[axis] = pidProfile->dFilter[axis].smartSmoothing;
    }
    directFF[0] = DIRECT_FF_SCALE * pidProfile->directFF_yaw;
    DF_angle_low = DIRECT_FF_SCALE * pidProfile->pid[PID_LEVEL_LOW].I;
    DF_angle_high = DIRECT_FF_SCALE * pidProfile->pid[PID_LEVEL_HIGH].I;
    feathered_pids = pidProfile->feathered_pids / 100.0f;
    dtermBoostMultiplier = (pidProfile->dtermBoost * pidProfile->dtermBoost / 1000000) * 0.003;
    dtermBoostLimitPercent = pidProfile->dtermBoostLimit / 100.0f;
    P_angle_low = pidProfile->pid[PID_LEVEL_LOW].P * 0.1f;
    D_angle_low = pidProfile->pid[PID_LEVEL_LOW].D * 0.00017f;
    P_angle_high = pidProfile->pid[PID_LEVEL_HIGH].P * 0.1f;
    D_angle_high = pidProfile->pid[PID_LEVEL_HIGH].D * 0.00017f;
    F_angle = pidProfile->pid[PID_LEVEL_LOW].F * 0.00000125f;
    horizonTransition = (float)pidProfile->horizonTransition;
    horizonCutoffDegrees = pidProfile->horizon_tilt_effect;
    horizonStrength = pidProfile->horizonStrength / 50.0f;
    maxVelocity[FD_ROLL] = maxVelocity[FD_PITCH] = pidProfile->rateAccelLimit * 100 * dT;
    maxVelocity[FD_YAW] = pidProfile->yawRateAccelLimit * 100 * dT;
    ITermWindupPointInv = 0.0f;
    if (pidProfile->itermWindupPointPercent != 0) {
        const float itermWindupPoint = pidProfile->itermWindupPointPercent / 100.0f;
        ITermWindupPointInv = 1.0f / itermWindupPoint;
    }
    crashTimeLimitUs = pidProfile->crash_time * 1000;
    crashTimeDelayUs = pidProfile->crash_delay * 1000;
    crashRecoveryAngleDeciDegrees = pidProfile->crash_recovery_angle * 10;
    crashRecoveryRate = pidProfile->crash_recovery_rate;
    crashGyroThreshold = pidProfile->crash_gthreshold;
    crashDtermThreshold = pidProfile->crash_dthreshold;
    crashSetpointThreshold = pidProfile->crash_setpoint_threshold;
    crashLimitYaw = pidProfile->crash_limit_yaw;
    itermLimit = pidProfile->itermLimit;
#if defined(USE_THROTTLE_BOOST)
    throttleBoost = pidProfile->throttle_boost * 0.1f;
#endif
    itermRotation = pidProfile->iterm_rotation;
#if defined(USE_ITERM_RELAX)
    itermRelaxCutoff = pidProfile->iterm_relax_cutoff;
    itermRelaxCutoffYaw = pidProfile->iterm_relax_cutoff_yaw;
#endif
    iDecay = (float)pidProfile->i_decay;
    axisLockMultiplier = pidProfile->axis_lock_multiplier / 100.0f;
    mixerInitProfile();
}

void pidInit(const pidProfile_t *pidProfile) {
    pidSetTargetLooptime(gyro.targetLooptime * pidConfig()->pid_process_denom); // Initialize pid looptime
    pidInitFilters(pidProfile);
    pidInitConfig(pidProfile);
}

void pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex) {
    if ((dstPidProfileIndex < PID_PROFILE_COUNT - 1 && srcPidProfileIndex < PID_PROFILE_COUNT - 1) && dstPidProfileIndex != srcPidProfileIndex) {
        memcpy(pidProfilesMutable(dstPidProfileIndex), pidProfilesMutable(srcPidProfileIndex), sizeof(pidProfile_t));
    }
}

// calculates strength of horizon leveling; 0 = none, 1.0 = most leveling
static float calcHorizonLevelStrength(void) {
    float horizonLevelStrength;
    // 0 at level, 90 at vertical, 180 at inverted (degrees):
    const float currentInclination = MAX(ABS(attitude.values.roll), ABS(attitude.values.pitch)) / 10.0f;
    // Used as a factor in the numerator of inclinationLevelRatio - this will cause the entry point of the fade of leveling strength to be adjustable via horizon transition in configurator for RACEMODEhorizon
    const float racemodeHorizonTransitionFactor = horizonCutoffDegrees / (horizonCutoffDegrees - horizonTransition);
    // Used as a factor in the numerator of inclinationLevelRatio - this will cause the fade of leveling strength to start at levelAngleLimit for RACEMODEangle
    // horizonTiltExpertMode:  0 = RACEMODEangle - ANGLE LIMIT BEHAVIOUR ON ROLL AXIS
    //                         1 = RACEMODEhorizon - HORIZON TYPE BEHAVIOUR ON ROLL AXIS
    //determines the leveling strength of RACEMODEhorizon
    if (horizonCutoffDegrees > 0 && horizonTransition < horizonCutoffDegrees) {
        //if racemode_tilt_effect>0 and if horizonTransition<racemode_tilt_effect
        //causes leveling to fade from horizonTransition angle to horizonCutoffDegrees  where leveling goes to zero
        const float inclinationLevelRatio = constrainf(((horizonCutoffDegrees - currentInclination) * racemodeHorizonTransitionFactor) / horizonCutoffDegrees, 0, 1);
        // apply inclination ratio to horizonLevelStrength which lowers leveling to zero as a function of angle and regardless of stick position
        horizonLevelStrength = inclinationLevelRatio;
    } else {
        // if racemode_tilt_effect = 0 or horizonTransition>racemode_tilt_effect means no leveling
        horizonLevelStrength = 1;
    }
    return constrainf(horizonLevelStrength, 0, 1);
}

static float pidLevel(int axis, const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim, float currentPidSetpoint) {
    // calculate error angle and limit the angle to the max inclination
    // rcDeflection is in range [-1.0, 1.0]
    static float attitudePrevious[2], previousAngle[2];
    float p_term_low, p_term_high, d_term_low, d_term_high, f_term_low;
    float angle = pidProfile->levelAngleLimit * getRcDeflection(axis);
    if (pidProfile->angleExpo > 0) {
        const float expof = pidProfile->angleExpo / 100.0f;
        angle = pidProfile->levelAngleLimit * (getRcDeflection(axis) * power3(getRcDeflectionAbs(axis)) * expof + getRcDeflection(axis) * (1 - expof));
    }
#ifdef USE_GPS_RESCUE
    angle += gpsRescueAngle[axis] / 100; // ANGLE IS IN CENTIDEGREES
#endif
    f_term_low = (angle - previousAngle[axis]) * F_angle / dT;
    previousAngle[axis] = angle;
    angle = constrainf(angle, -pidProfile->levelAngleLimit, pidProfile->levelAngleLimit);
    float errorAngle = angle - ((attitude.raw[axis] - angleTrim->raw[axis]) * 0.1f);
    errorAngle = constrainf(errorAngle, -90.0f, 90.0f);
    const float errorAnglePercent = fabsf(errorAngle / 90.0f);
    // ANGLE mode - control is angle based
    p_term_low = (1 - errorAnglePercent) * errorAngle * P_angle_low;
    p_term_high = errorAnglePercent * errorAngle * P_angle_high;
    d_term_low = (1 - errorAnglePercent) * (attitudePrevious[axis] - attitude.raw[axis]) * 0.1f * D_angle_low;
    d_term_high = errorAnglePercent * (attitudePrevious[axis] - attitude.raw[axis]) * 0.1f * D_angle_high;
    attitudePrevious[axis] = attitude.raw[axis];
    currentPidSetpoint = p_term_low + p_term_high;
    currentPidSetpoint += d_term_low + d_term_high;
    currentPidSetpoint += f_term_low;
    if (FLIGHT_MODE(HORIZON_MODE)) {
        // HORIZON mode - mix of ANGLE and ACRO modes
        // mix in errorAngle to currentPidSetpoint to add a little auto-level feel
        const float horizonLevelStrength = calcHorizonLevelStrength();
        currentPidSetpoint = ((getSetpointRate(axis) * (1 - horizonLevelStrength)) + getSetpointRate(axis)) * 0.5f + (currentPidSetpoint * horizonLevelStrength * horizonStrength);
    }
    directFF[axis] = (1 - fabsf(errorAnglePercent)) * DF_angle_low;
    directFF[axis] += fabsf(errorAnglePercent) * DF_angle_high;
    return currentPidSetpoint;
}

static float accelerationLimit(int axis, float currentPidSetpoint) {
    static float previousSetpoint[XYZ_AXIS_COUNT];
    const float currentVelocity = currentPidSetpoint - previousSetpoint[axis];
    if (ABS(currentVelocity) > maxVelocity[axis]) {
        currentPidSetpoint = (currentVelocity > 0) ? previousSetpoint[axis] + maxVelocity[axis] : previousSetpoint[axis] - maxVelocity[axis];
    }
    previousSetpoint[axis] = currentPidSetpoint;
    return currentPidSetpoint;
}

static timeUs_t crashDetectedAtUs;

static void handleCrashRecovery(
    const pidCrashRecovery_e crash_recovery, const rollAndPitchTrims_t *angleTrim,
    const int axis, const timeUs_t currentTimeUs, const float gyroRate, float *currentPidSetpoint, float *errorRate) {
    if (inCrashRecoveryMode && cmpTimeUs(currentTimeUs, crashDetectedAtUs) > crashTimeDelayUs) {
        if (crash_recovery == PID_CRASH_RECOVERY_BEEP) {
            BEEP_ON;
        }
        if (axis == FD_YAW) {
            *errorRate = constrainf(*errorRate, -crashLimitYaw, crashLimitYaw);
        } else {
            // on roll and pitch axes calculate currentPidSetpoint and errorRate to level the aircraft to recover from crash
            if (sensors(SENSOR_ACC)) {
                // errorAngle is deviation from horizontal
                const float errorAngle = -(attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
                *currentPidSetpoint = errorAngle * P_angle_low;
                *errorRate = *currentPidSetpoint - gyroRate;
            }
        }
        // reset ITerm, since accumulated error before crash is now meaningless
        // and ITerm windup during crash recovery can be extreme, especially on yaw axis
        temporaryIterm[axis] = 0.0f;
        if (
            cmpTimeUs(currentTimeUs, crashDetectedAtUs) > crashTimeLimitUs ||
            (getControllerMixRange() < 1.0f &&
             ABS(gyro.gyroADCf[FD_ROLL]) < crashRecoveryRate &&
             ABS(gyro.gyroADCf[FD_PITCH]) < crashRecoveryRate &&
             ABS(gyro.gyroADCf[FD_YAW]) < crashRecoveryRate
            )
        ) {
            if (sensors(SENSOR_ACC)) {
                // check aircraft nearly level
                if (ABS(attitude.raw[FD_ROLL] - angleTrim->raw[FD_ROLL]) < crashRecoveryAngleDeciDegrees && ABS(attitude.raw[FD_PITCH] - angleTrim->raw[FD_PITCH]) < crashRecoveryAngleDeciDegrees) {
                    inCrashRecoveryMode = false;
                    BEEP_OFF;
                }
            } else {
                inCrashRecoveryMode = false;
                BEEP_OFF;
            }
        }
    }
}

static void detectAndSetCrashRecovery(
    const pidCrashRecovery_e crash_recovery, const int axis,
    const timeUs_t currentTimeUs, const float delta, const float errorRate) {
    // if crash recovery is on and accelerometer enabled and there is no gyro overflow, then check for a crash
    // no point in trying to recover if the crash is so severe that the gyro overflows
    if ((crash_recovery || FLIGHT_MODE(GPS_RESCUE_MODE)) && !gyroOverflowDetected()) {
        if (ARMING_FLAG(ARMED)) {
            if (getControllerMixRange() >= 1.0f && !inCrashRecoveryMode && ABS(delta) > crashDtermThreshold && ABS(errorRate) > crashGyroThreshold && ABS(getSetpointRate(axis)) < crashSetpointThreshold) {
                inCrashRecoveryMode = true;
                crashDetectedAtUs = currentTimeUs;
            }
            if (inCrashRecoveryMode && cmpTimeUs(currentTimeUs, crashDetectedAtUs) < crashTimeDelayUs && (ABS(errorRate) < crashGyroThreshold || ABS(getSetpointRate(axis)) > crashSetpointThreshold)) {
                inCrashRecoveryMode = false;
                BEEP_OFF;
            }
        } else if (inCrashRecoveryMode) {
            inCrashRecoveryMode = false;
            BEEP_OFF;
        }
    }
}

static void rotateVector(float v[XYZ_AXIS_COUNT], float rotation[XYZ_AXIS_COUNT]) {
    // rotate v around rotation vector rotation
    // rotation in radians, all elements must be small
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        int i_1 = (i + 1) % 3;
        int i_2 = (i + 2) % 3;
        float newV = v[i_1] + v[i_2] * rotation[i];
        v[i_2] -= v[i_1] * rotation[i];
        v[i_1] = newV;
    }
}

static void rotateITermAndAxisError() {
    if (itermRotation) {
        const float gyroToAngle = dT * RAD;
        float rotationRads[XYZ_AXIS_COUNT];
        for (int i = FD_ROLL; i <= FD_YAW; i++) {
            rotationRads[i] = gyro.gyroADCf[i] * gyroToAngle;
        }
        float v[XYZ_AXIS_COUNT];
        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
            v[i] = temporaryIterm[i];
        }
        rotateVector(v, rotationRads);
        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
            temporaryIterm[i] = v[i];
        }
    }
}

static FAST_RAM_ZERO_INIT float scaledAxisPid[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float stickMovement[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float lastRcDeflectionAbs[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float previousError[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float previousMeasurement[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float previousdDelta[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float kdRingBuffer[XYZ_AXIS_COUNT][KD_RING_BUFFER_SIZE];
static FAST_RAM_ZERO_INIT float kdRingBufferSum[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT uint8_t kdRingBufferPoint[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT timeUs_t crashDetectedAtUs;

void pidController(const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim, timeUs_t currentTimeUs) {
    float axisLock[XYZ_AXIS_COUNT];
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        axisLock[axis] = pt1FilterApply(&axisLockLpf[axis], stickMovement[axis]) * axisLockMultiplier;
    }

    scaledAxisPid[ROLL] = constrainf(1 - axisLock[PITCH] - axisLock[YAW] + axisLock[ROLL], 0.0f, 1.0f);
    scaledAxisPid[PITCH] = constrainf(1 - axisLock[ROLL] - axisLock[YAW] + axisLock[PITCH], 0.0f, 1.0f);
    scaledAxisPid[YAW] = constrainf(1 - axisLock[ROLL] - axisLock[PITCH] + axisLock[YAW], 0.0f, 1.0f);

    // gradually scale back integration when above windup point
    float dynCi = dT;
    if (ITermWindupPointInv != 0.0f) {
        dynCi *= constrainf((1.0f - getControllerMixRange()) * ITermWindupPointInv, 0.0f, 1.0f);
    }
    float errorRate;
    // ----------PID controller----------
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {

        // emugravity, the different hopefully better version of antiGravity no effect on yaw
        float errorAccelerator = 1.0f;
        if (pidProfile->emuGravityGain != 0 && axis != FD_YAW) {
            errorAccelerator = 1.0f + fabsf(emuGravityThrottleHpf) * 0.1f * (pidProfile->emuGravityGain);
        }
        float currentPidSetpoint = getSetpointRate(axis);
        if (maxVelocity[axis]) {
            currentPidSetpoint = accelerationLimit(axis, currentPidSetpoint);
        }
        // Yaw control is GYRO based, direct sticks control is applied to rate PID
        // NFE racermode applies angle only to the roll axis
        // disable directFF for pitch and roll while not in angle
        if (axis != FD_YAW) {
            directFF[axis] = 0.0f;
        }

        if (axis == FD_YAW) {
        } else if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
            currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint);
        } else if ((FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) && !FLIGHT_MODE(NFE_RACE_MODE)) {
            currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint);
        } else if ((FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) && FLIGHT_MODE(NFE_RACE_MODE) && (axis != FD_PITCH)) {
            currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint);
        }
        // Handle yaw spin recovery - zero the setpoint on yaw to aid in recovery
        // It's not necessary to zero the set points for R/P because the PIDs will be zeroed below
#ifdef USE_YAW_SPIN_RECOVERY
        if ((axis == FD_YAW) && gyroYawSpinDetected()) {
            currentPidSetpoint = 0.0f;
        }
#endif // USE_YAW_SPIN_RECOVERY

        previousPidSetpoint[axis] = currentPidSetpoint;

        stickMovement[axis] = (getRcDeflectionAbs(axis) - lastRcDeflectionAbs[axis]) * pidFrequency;
        lastRcDeflectionAbs[axis] = getRcDeflectionAbs(axis);

        const float gyroRate = gyro.gyroADCf[axis];
        // -----calculate error rate
        errorRate = currentPidSetpoint - gyroRate; // r - y
        errorRate *= errorAccelerator; // increase the error as throttle moves, = much more agressive antiGravity

        // EmuFlight pid controller, which will be maintained in the future with additional features specialised for current (mini) multirotor usage.
        // Based on 2DOF reference design (matlab)
        float errorBoostAxis;
        float errorLimitAxis;
        if (axis <= FD_PITCH) {
            errorBoostAxis = pidProfile->errorBoost;
            errorLimitAxis = pidProfile->errorBoostLimit;
        } else {
            errorBoostAxis = pidProfile->errorBoostYaw;
            errorLimitAxis = pidProfile->errorBoostLimitYaw;
        }
        errorLimitAxis = errorLimitAxis / 100;
        float errorMultiplier = (errorBoostAxis * errorBoostAxis / 1000000) * 0.003;
        float boostedErrorRate;
        boostedErrorRate = (errorRate * fabsf(errorRate)) * errorMultiplier;
        if (fabsf(errorRate * errorLimitAxis) < fabsf(boostedErrorRate)) {
            boostedErrorRate = errorRate * errorLimitAxis;
        }
        rotateITermAndAxisError();
        // --------low-level gyro-based PID based on 2DOF PID controller. ----------
        // 2-DOF PID controller with optional filter on derivative term.
        // derivative term can be based on measurement or error using a sliding value from 0-100
        float itermErrorRate = boostedErrorRate + errorRate;
        float iterm          = temporaryIterm[axis];
#if defined(USE_ITERM_RELAX)
        float itermRelaxFactor;
        if ((itermRelaxCutoff && axis != FD_YAW) || (itermRelaxCutoffYaw && axis == FD_YAW)) {
            const float setpointLpf = pt1FilterApply(&windupLpf[axis], currentPidSetpoint);
            const float setpointHpf = fabsf(currentPidSetpoint - setpointLpf);
            if (axis != FD_YAW) {
                itermRelaxFactor = MAX(1 - setpointHpf / pidProfile->iterm_relax_threshold, 0.0f);
            } else {
                itermRelaxFactor = MAX(1 - setpointHpf / pidProfile->iterm_relax_threshold_yaw, 0.0f);
            }
            if (SIGN(iterm) == SIGN(itermErrorRate)) {
                itermErrorRate *= itermRelaxFactor;
            }
            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_ITERM_RELAX, 0, lrintf(setpointHpf));
                DEBUG_SET(DEBUG_ITERM_RELAX, 1, lrintf(itermRelaxFactor * 100.0f));
                DEBUG_SET(DEBUG_ITERM_RELAX, 2, lrintf(itermErrorRate));
            }
        }
#endif // USE_ITERM_RELAX
        // -----calculate P component
        pidData[axis].P = (pidCoefficient[axis].Kp * (boostedErrorRate + errorRate));
        // -----calculate I component
        //float iterm = constrainf(pidData[axis].I + (pidCoefficient[axis].Ki * errorRate) * dynCi, -itermLimit, itermLimit);
        float iDecayMultiplier = iDecay;
        float ITermNew = pidCoefficient[axis].Ki * itermErrorRate * dynCi;
        if (ITermNew != 0.0f) {
            if (SIGN(iterm) != SIGN(ITermNew)) {
                // at low iterm iDecayMultiplier will be 1 and at high iterm it will be equivilant to iDecay
                iDecayMultiplier = 1.0f + (iDecay - 1.0f) * constrainf(iterm / pidProfile->i_decay_cutoff, 0.0f, 1.0f);
                const float newVal = ITermNew * iDecayMultiplier;
                if (fabs(iterm) > fabs(newVal)) {
                    ITermNew = newVal;
                }
            }
        }
        iterm = constrainf(iterm + ITermNew, -itermLimit, itermLimit);
        if (!mixerIsOutputSaturated(axis, errorRate) || ABS(iterm) < ABS(temporaryIterm[axis])) {
            // Only increase ITerm if output is not saturated
            temporaryIterm[axis] = iterm;
        }
        // -----calculate D component
        if (pidCoefficient[axis].Kd > 0) {
            //filter Kd properly, no setpoint filtering
            const float pureError = errorRate - previousError[axis];
            const float pureMeasurement = -(gyroRate - previousMeasurement[axis]);
            previousMeasurement[axis] = gyroRate;
            previousError[axis] = errorRate;
            float dDelta = ((feathered_pids * pureMeasurement) + ((1 - feathered_pids) * pureError)) * pidFrequency; //calculating the dterm determine how much is calculated using measurement vs error
            //filter the dterm
            dDelta = dtermLowpassApplyFn((filter_t *)&dtermLowpass[axis], dDelta);
            dDelta = dtermLowpass2ApplyFn((filter_t *)&dtermLowpass2[axis], dDelta);
            if (pidProfile->dFilter[axis].Wc > 1) {
                kdRingBuffer[axis][kdRingBufferPoint[axis]++] = dDelta;
                kdRingBufferSum[axis] += dDelta;
                if (kdRingBufferPoint[axis] == pidProfile->dFilter[axis].Wc) {
                    kdRingBufferPoint[axis] = 0;
                }
                dDelta = (float)(kdRingBufferSum[axis] / (float)(pidProfile->dFilter[axis].Wc));
                kdRingBufferSum[axis] -= kdRingBuffer[axis][kdRingBufferPoint[axis]];
            }
            //dterm boost, similar to emuboost
            float boostedDtermRate;
            boostedDtermRate = (dDelta * fabsf(dDelta)) * dtermBoostMultiplier;
            if (fabsf(dDelta * dtermBoostLimitPercent) < fabsf(boostedDtermRate)) {
                boostedDtermRate = dDelta * dtermBoostLimitPercent;
            }
            dDelta += boostedDtermRate;
            dDelta = pidCoefficient[axis].Kd * dDelta;
            float dDeltaMultiplier;
            if (smart_dterm_smoothing[axis] > 0) {
                dDeltaMultiplier = constrainf(fabsf((dDelta + previousdDelta[axis]) / (4 * smart_dterm_smoothing[axis])) + 0.5, 0.5f, 1.0f); //smooth transition from 0.5-1.0f for the multiplier.
                dDelta = dDelta * dDeltaMultiplier;
                previousdDelta[axis] = dDelta;
                DEBUG_SET(DEBUG_SMART_SMOOTHING, axis, dDeltaMultiplier * 1000.0f);
            }
            // Divide rate change by dT to get differential (ie dr/dt).
            // dT is fixed and calculated from the target PID loop time
            // This is done to avoid DTerm spikes that occur with dynamically
            // calculated deltaT whenever another task causes the PID
            // loop execution to be delayed.
            pidData[axis].D = dDelta;
        } else {
            pidData[axis].D = 0;
        }

        handleCrashRecovery(pidProfile->crash_recovery, angleTrim, axis, currentTimeUs, errorRate, &currentPidSetpoint, &errorRate);
        detectAndSetCrashRecovery(pidProfile->crash_recovery, axis, currentTimeUs, pidData[axis].D, errorRate);

        // -----calculate direct yaw feedforward component
        // feedforward as betaflight calls it is really a setpoint derivative
        // this feedforward is literally setpoint * feedforward
        // since yaw acts different this will only work for yaw
        // actually this also works in angle mode so pitch and roll have angle mode values
        float directFeedForward = currentPidSetpoint * directFF[axis];

#ifdef USE_YAW_SPIN_RECOVERY
        if (gyroYawSpinDetected()) {
            temporaryIterm[axis] = 0; // in yaw spin always disable I
            if (axis <= FD_PITCH) {
                // zero PIDs on pitch and roll leaving yaw P to correct spin
                pidData[axis].P = 0;
                pidData[axis].D = 0;
            }
        }
#endif // USE_YAW_SPIN_RECOVERY
        // Disable PID control if at zero throttle or if gyro overflow detected
        // This may look very innefficient, but it is done on purpose to always show real CPU usage as in flight
        if (!pidStabilisationEnabled || gyroOverflowDetected()) {
            pidData[axis].P = 0;
            temporaryIterm[axis] = 0;
            pidData[axis].D = 0;
            pidData[axis].Sum = 0;
        }

        // applying SetPointAttenuation
        // SPA boost if SPA > 100 SPA cut if SPA < 100
        float setPointPAttenuation = 1 + (getRcDeflectionAbs(axis) * (setPointPTransition[axis] - 1));
        float setPointIAttenuation = 1 + (getRcDeflectionAbs(axis) * (setPointITransition[axis] - 1));
        float setPointDAttenuation = 1 + (getRcDeflectionAbs(axis) * (setPointDTransition[axis] - 1));
        pidData[axis].P *= setPointPAttenuation;
        pidData[axis].I = temporaryIterm[axis] * setPointIAttenuation; // you can't use pidData[axis].I to calculate iterm or with tpa you get issues
        pidData[axis].D *= setPointDAttenuation;

        if (!linearThrustEnabled) { // thrust linearization already solves high throttle PID problems
            pidData[axis].P *= getThrottlePAttenuation();
            pidData[axis].I *= getThrottleIAttenuation();
            pidData[axis].D *= getThrottleDAttenuation();
        }

        const float pidSum = pidData[axis].P + pidData[axis].I + pidData[axis].D + directFeedForward;
        pidData[axis].Sum = pidSum * scaledAxisPid[axis];
    }
}

bool crashRecoveryModeActive(void) {
    return inCrashRecoveryMode;
}

float pidGetPreviousSetpoint(int axis) {
    return previousPidSetpoint[axis];
}
