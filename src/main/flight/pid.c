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

#define ITERM_RELAX_SETPOINT_THRESHOLD 30.0f

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

void resetPidProfile(pidProfile_t *pidProfile)
{
    RESET_CONFIG(pidProfile_t, pidProfile,
        .pid = {
            [PID_ROLL] = DEFAULT_PIDS_ROLL,
            [PID_PITCH] = DEFAULT_PIDS_PITCH,
            [PID_YAW] = DEFAULT_PIDS_YAW,
            [PID_LEVEL_LOW] = {100, 0, 10, 40},
            [PID_LEVEL_HIGH] = {35, 0, 1, 0},
            [PID_MAG] = {40, 0, 0, 0},
        },

        .dFilter = {
            [PID_ROLL] = {2, 100, 250, 0},  // wc, dtermlpf, dtermlpf2, smartSmoothing
            [PID_PITCH] = {2, 100, 250, 0}, // wc, dtermlpf, dtermlpf2, smartSmoothing
            [PID_YAW] = {0, 100, 250, 0},    // wc, dtermlpf, dtermlpf2, smartSmoothing
        },

        .pidSumLimit = PIDSUM_LIMIT_MAX,
        .pidSumLimitYaw = PIDSUM_LIMIT_YAW,
        .dterm_filter_type = FILTER_PT1,
        .itermWindupPointPercent = 50,
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
        .errorBoost = 15,
        .errorBoostYaw = 40,
        .errorBoostLimit = 20,
        .errorBoostLimitYaw = 40,
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
        .horizon_tilt_effect = 130,
        .nfe_racermode = false,
        .crash_limit_yaw = 200,
        .itermLimit = 400,
        .throttle_boost = 5,
        .throttle_boost_cutoff = 15,
        .iterm_rotation = true,
        .motor_output_limit = 100,
        .auto_profile_cell_count = AUTO_PROFILE_CELL_COUNT_STAY,
        .horizonTransition = 0,
    );
}

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++)
    {
        resetPidProfile(&pidProfiles[i]);
    }
}

static void pidSetTargetLooptime(uint32_t pidLooptime)
{
    targetPidLooptime = pidLooptime;
    dT = targetPidLooptime * 1e-6f;
    pidFrequency = 1.0f / dT;
}

void pidStabilisationState(pidStabilisationState_e pidControllerState)
{
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

static FAST_RAM_ZERO_INIT float iDecay;

#ifdef USE_RC_SMOOTHING_FILTER
static FAST_RAM_ZERO_INIT pt1Filter_t setpointDerivativePt1[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT biquadFilter_t setpointDerivativeBiquad[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT bool setpointDerivativeLpfInitialized;
static FAST_RAM_ZERO_INIT uint8_t rcSmoothingDebugAxis;
static FAST_RAM_ZERO_INIT uint8_t rcSmoothingFilterType;
#endif // USE_RC_SMOOTHING_FILTER

void pidInitFilters(const pidProfile_t *pidProfile)
{
    BUILD_BUG_ON(FD_YAW != 2);                             // ensure yaw axis is 2
    const uint32_t pidFrequencyNyquist = pidFrequency / 2; // No rounding needed

    dtermLowpassApplyFn = nullFilterApply;
    dtermLowpass2ApplyFn = nullFilterApply;

    for (int axis = FD_ROLL; axis <= FD_YAW; axis++)
    {
        if (pidProfile->dFilter[axis].dLpf && pidProfile->dFilter[axis].dLpf <= pidFrequencyNyquist)
        {
            switch (pidProfile->dterm_filter_type)
            {
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
        if (pidProfile->dFilter[axis].dLpf2 && pidProfile->dFilter[axis].dLpf2 <= pidFrequencyNyquist)
        {
            switch (pidProfile->dterm_filter_type)
            {
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
}

#ifdef USE_RC_SMOOTHING_FILTER
void pidInitSetpointDerivativeLpf(uint16_t filterCutoff, uint8_t debugAxis, uint8_t filterType)
{
    rcSmoothingDebugAxis = debugAxis;
    rcSmoothingFilterType = filterType;
    if ((filterCutoff > 0) && (rcSmoothingFilterType != RC_SMOOTHING_DERIVATIVE_OFF))
    {
        setpointDerivativeLpfInitialized = true;
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++)
        {
            switch (rcSmoothingFilterType)
            {
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

void pidUpdateSetpointDerivativeLpf(uint16_t filterCutoff)
{
    if ((filterCutoff > 0) && (rcSmoothingFilterType != RC_SMOOTHING_DERIVATIVE_OFF))
    {
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++)
        {
            switch (rcSmoothingFilterType)
            {
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

typedef struct pidCoefficient_s
{
    float Kp;
    float Ki;
    float Kd;
    float Kf;
} pidCoefficient_t;

static FAST_RAM_ZERO_INIT pidCoefficient_t pidCoefficient[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float maxVelocity[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float feathered_pids;
static FAST_RAM_ZERO_INIT uint8_t nfe_racermode;
static FAST_RAM_ZERO_INIT float smart_dterm_smoothing[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float setPointPTransition[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float setPointITransition[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float setPointDTransition[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float P_angle_low, D_angle_low, P_angle_high, D_angle_high, F_angle, horizonTransition, horizonCutoffDegrees;
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
#if defined(USE_THROTTLE_BOOST)
FAST_RAM_ZERO_INIT float throttleBoost;
pt1Filter_t throttleLpf;
#endif
static FAST_RAM_ZERO_INIT bool itermRotation;
static FAST_RAM_ZERO_INIT float temporaryIterm[XYZ_AXIS_COUNT];

void pidResetITerm(void)
{
    for (int axis = 0; axis < 3; axis++)
    {
        temporaryIterm[axis] = 0.0f;
    }
}

void pidInitConfig(const pidProfile_t *pidProfile)
{

    for (int axis = FD_ROLL; axis <= FD_YAW; axis++)
    {
        pidCoefficient[axis].Kp = PTERM_SCALE * pidProfile->pid[axis].P;
        pidCoefficient[axis].Ki = ITERM_SCALE * pidProfile->pid[axis].I;
        pidCoefficient[axis].Kd = DTERM_SCALE * pidProfile->pid[axis].D;
        setPointPTransition[axis] = pidProfile->setPointPTransition[axis] / 100.0f;
        setPointITransition[axis] = pidProfile->setPointITransition[axis] / 100.0f;
        setPointDTransition[axis] = pidProfile->setPointDTransition[axis] / 100.0f;
        smart_dterm_smoothing[axis] = pidProfile->dFilter[axis].smartSmoothing;
    }
    feathered_pids = pidProfile->feathered_pids / 100.0f;
    nfe_racermode = pidProfile->nfe_racermode;
    P_angle_low = pidProfile->pid[PID_LEVEL_LOW].P * 0.1f;
    D_angle_low = pidProfile->pid[PID_LEVEL_LOW].D * 0.00017f;
    P_angle_high = pidProfile->pid[PID_LEVEL_HIGH].P * 0.1f;
    D_angle_high = pidProfile->pid[PID_LEVEL_HIGH].D * 0.00017f;
    F_angle = pidProfile->pid[PID_LEVEL_LOW].F * 0.00000125f;
    horizonTransition = (float)pidProfile->horizonTransition;
    horizonCutoffDegrees = pidProfile->horizon_tilt_effect;
    maxVelocity[FD_ROLL] = maxVelocity[FD_PITCH] = pidProfile->rateAccelLimit * 100 * dT;
    maxVelocity[FD_YAW] = pidProfile->yawRateAccelLimit * 100 * dT;
    const float ITermWindupPoint = (float)pidProfile->itermWindupPointPercent / 100.0f;
    ITermWindupPointInv = 1.0f / (1.0f - ITermWindupPoint);
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
    iDecay = (float)pidProfile->i_decay;
}

void pidInit(const pidProfile_t *pidProfile)
{
    pidSetTargetLooptime(gyro.targetLooptime * pidConfig()->pid_process_denom); // Initialize pid looptime
    pidInitFilters(pidProfile);
    pidInitConfig(pidProfile);
}

void pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex)
{

    if ((dstPidProfileIndex < PID_PROFILE_COUNT - 1 && srcPidProfileIndex < PID_PROFILE_COUNT - 1) && dstPidProfileIndex != srcPidProfileIndex)
    {
        memcpy(pidProfilesMutable(dstPidProfileIndex), pidProfilesMutable(srcPidProfileIndex), sizeof(pidProfile_t));
    }
}

// calculates strength of horizon leveling; 0 = none, 1.0 = most leveling
static float calcHorizonLevelStrength(void)
{
    float horizonLevelStrength;
    // 0 at level, 90 at vertical, 180 at inverted (degrees):
    const float currentInclination = MAX(ABS(attitude.values.roll), ABS(attitude.values.pitch)) / 10.0f;
    // Used as a factor in the numerator of inclinationLevelRatio - this will cause the entry point of the fade of leveling strength to be adjustable via horizon transition in configurator for RACEMODEhorizon
    const float racemodeHorizonTransitionFactor = horizonCutoffDegrees / (horizonCutoffDegrees - horizonTransition);
    // Used as a factor in the numerator of inclinationLevelRatio - this will cause the fade of leveling strength to start at levelAngleLimit for RACEMODEangle
    // horizonTiltExpertMode:  0 = RACEMODEangle - ANGLE LIMIT BEHAVIOUR ON ROLL AXIS
    //                         1 = RACEMODEhorizon - HORIZON TYPE BEHAVIOUR ON ROLL AXIS

    //determines the leveling strength of RACEMODEhorizon

    if (horizonCutoffDegrees > 0 && horizonTransition < horizonCutoffDegrees)
    { //if racemode_tilt_effect>0 and if horizonTransition<racemode_tilt_effect

        //causes leveling to fade from horizonTransition angle to horizonCutoffDegrees	where leveling goes to zero
        const float inclinationLevelRatio = constrainf(((horizonCutoffDegrees - currentInclination) * racemodeHorizonTransitionFactor) / horizonCutoffDegrees, 0, 1);
        // apply inclination ratio to horizonLevelStrength which lowers leveling to zero as a function of angle and regardless of stick position
        horizonLevelStrength = inclinationLevelRatio;
    }
    else
    { // if racemode_tilt_effect = 0 or horizonTransition>racemode_tilt_effect means no leveling
        horizonLevelStrength = 0;
    }
    return constrainf(horizonLevelStrength, 0, 1);
}

#define SIGN(x) ((x > 0.0f) - (x < 0.0f))

static float pidLevel(int axis, const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim, float currentPidSetpoint)
{
    // calculate error angle and limit the angle to the max inclination
    // rcDeflection is in range [-1.0, 1.0]
    static float attitudePrevious[2], previousAngle[2];
    float p_term_low, p_term_high, d_term_low, d_term_high, f_term_low;

    float angle = pidProfile->levelAngleLimit * getRcDeflection(axis);
    if (pidProfile->angleExpo > 0)
    {
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
    const float errorAnglePercent = errorAngle / 90.0f;

    // ANGLE mode - control is angle based
    p_term_low = (1 - fabsf(errorAnglePercent)) * errorAngle * P_angle_low;
    p_term_high = fabsf(errorAnglePercent) * errorAngle * P_angle_high;

    d_term_low = (1 - fabsf(errorAnglePercent)) * (attitudePrevious[axis] - attitude.raw[axis]) * 0.1f * D_angle_low;
    d_term_high = fabsf(errorAnglePercent) * (attitudePrevious[axis] - attitude.raw[axis]) * 0.1f * D_angle_high;
    attitudePrevious[axis] = attitude.raw[axis];

    currentPidSetpoint = p_term_low + p_term_high;
    currentPidSetpoint += d_term_low + d_term_high;
    currentPidSetpoint += f_term_low;

    if (FLIGHT_MODE(HORIZON_MODE))
    {
        // HORIZON mode - mix of ANGLE and ACRO modes
        // mix in errorAngle to currentPidSetpoint to add a little auto-level feel
        const float horizonLevelStrength = calcHorizonLevelStrength();
        currentPidSetpoint = (((getSetpointRate(axis) * (1 - horizonLevelStrength)) + getSetpointRate(axis)) * 0.5f) + (currentPidSetpoint * horizonLevelStrength);
    }

    return currentPidSetpoint;
}

static float accelerationLimit(int axis, float currentPidSetpoint)
{
    static float previousSetpoint[XYZ_AXIS_COUNT];
    const float currentVelocity = currentPidSetpoint - previousSetpoint[axis];

    if (ABS(currentVelocity) > maxVelocity[axis])
    {
        currentPidSetpoint = (currentVelocity > 0) ? previousSetpoint[axis] + maxVelocity[axis] : previousSetpoint[axis] - maxVelocity[axis];
    }

    previousSetpoint[axis] = currentPidSetpoint;
    return currentPidSetpoint;
}

static timeUs_t crashDetectedAtUs;

static void handleCrashRecovery(
    const pidCrashRecovery_e crash_recovery, const rollAndPitchTrims_t *angleTrim,
    const int axis, const timeUs_t currentTimeUs, const float gyroRate, float *currentPidSetpoint, float *errorRate)
{
    if (inCrashRecoveryMode && cmpTimeUs(currentTimeUs, crashDetectedAtUs) > crashTimeDelayUs)
    {
        if (crash_recovery == PID_CRASH_RECOVERY_BEEP)
        {
            BEEP_ON;
        }
        if (axis == FD_YAW)
        {
            *errorRate = constrainf(*errorRate, -crashLimitYaw, crashLimitYaw);
        }
        else
        {
            // on roll and pitch axes calculate currentPidSetpoint and errorRate to level the aircraft to recover from crash
            if (sensors(SENSOR_ACC))
            {
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
            (getMotorMixRange() < 1.0f &&
                ABS(gyro.gyroADCf[FD_ROLL]) < crashRecoveryRate &&
                ABS(gyro.gyroADCf[FD_PITCH]) < crashRecoveryRate &&
                ABS(gyro.gyroADCf[FD_YAW]) < crashRecoveryRate
            )
        )
        {
            if (sensors(SENSOR_ACC))
            {
                // check aircraft nearly level
                if (ABS(attitude.raw[FD_ROLL] - angleTrim->raw[FD_ROLL]) < crashRecoveryAngleDeciDegrees && ABS(attitude.raw[FD_PITCH] - angleTrim->raw[FD_PITCH]) < crashRecoveryAngleDeciDegrees)
                {
                    inCrashRecoveryMode = false;
                    BEEP_OFF;
                }
            }
            else
            {
                inCrashRecoveryMode = false;
                BEEP_OFF;
            }
        }
    }
}

static void detectAndSetCrashRecovery(
    const pidCrashRecovery_e crash_recovery, const int axis,
    const timeUs_t currentTimeUs, const float delta, const float errorRate)
{
    // if crash recovery is on and accelerometer enabled and there is no gyro overflow, then check for a crash
    // no point in trying to recover if the crash is so severe that the gyro overflows
    if ((crash_recovery || FLIGHT_MODE(GPS_RESCUE_MODE)) && !gyroOverflowDetected())
    {
        if (ARMING_FLAG(ARMED))
        {
            if (getMotorMixRange() >= 1.0f && !inCrashRecoveryMode && ABS(delta) > crashDtermThreshold && ABS(errorRate) > crashGyroThreshold && ABS(getSetpointRate(axis)) < crashSetpointThreshold)
            {
                inCrashRecoveryMode = true;
                crashDetectedAtUs = currentTimeUs;
            }
            if (inCrashRecoveryMode && cmpTimeUs(currentTimeUs, crashDetectedAtUs) < crashTimeDelayUs && (ABS(errorRate) < crashGyroThreshold || ABS(getSetpointRate(axis)) > crashSetpointThreshold))
            {
                inCrashRecoveryMode = false;
                BEEP_OFF;
            }
        }
        else if (inCrashRecoveryMode)
        {
            inCrashRecoveryMode = false;
            BEEP_OFF;
        }
    }
}

static void rotateVector(float v[XYZ_AXIS_COUNT], float rotation[XYZ_AXIS_COUNT])
{
    // rotate v around rotation vector rotation
    // rotation in radians, all elements must be small
    for (int i = 0; i < XYZ_AXIS_COUNT; i++)
    {
        int i_1 = (i + 1) % 3;
        int i_2 = (i + 2) % 3;
        float newV = v[i_1] + v[i_2] * rotation[i];
        v[i_2] -= v[i_1] * rotation[i];
        v[i_1] = newV;
    }
}

static void rotateITermAndAxisError()
{
    if (itermRotation)
    {
        const float gyroToAngle = dT * RAD;
        float rotationRads[XYZ_AXIS_COUNT];
        for (int i = FD_ROLL; i <= FD_YAW; i++)
        {
            rotationRads[i] = gyro.gyroADCf[i] * gyroToAngle;
        }
        float v[XYZ_AXIS_COUNT];
        for (int i = 0; i < XYZ_AXIS_COUNT; i++)
        {
            v[i] = temporaryIterm[i];
        }
        rotateVector(v, rotationRads);
        for (int i = 0; i < XYZ_AXIS_COUNT; i++)
        {
            temporaryIterm[i] = v[i];
        }
    }
}

static FAST_RAM_ZERO_INIT float previousError[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float previousMeasurement[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float previousdDelta[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float kdRingBuffer[XYZ_AXIS_COUNT][KD_RING_BUFFER_SIZE];
static FAST_RAM_ZERO_INIT float kdRingBufferSum[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT uint8_t kdRingBufferPoint[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float setPointPAttenuation[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float setPointIAttenuation[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float setPointDAttenuation[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT timeUs_t crashDetectedAtUs;

void pidController(const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim, timeUs_t currentTimeUs)
{
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++)
    { // calculate spa
        // SPA boost if SPA > 100 SPA cut if SPA < 100
        setPointPAttenuation[axis] = 1 + (getRcDeflectionAbs(axis) * (setPointPTransition[axis] - 1));
        setPointIAttenuation[axis] = 1 + (getRcDeflectionAbs(axis) * (setPointITransition[axis] - 1));
        setPointDAttenuation[axis] = 1 + (getRcDeflectionAbs(axis) * (setPointDTransition[axis] - 1));
    }

    //vbat pid compensation on just the p term :) thanks NFE
    float vbatCompensationFactor = calculateVbatCompensation(currentControlRateProfile->vbat_comp_type, currentControlRateProfile->vbat_comp_ref);
    vbatCompensationFactor = scaleRangef(currentControlRateProfile->vbat_comp_pid_level, 0.0f, 100.0f, 1.0f, vbatCompensationFactor);

    // gradually scale back integration when above windup point
    const float dynCi = constrainf((1.0f - getMotorMixRange()) * ITermWindupPointInv, 0.0f, 1.0f) * dT;
    float errorRate;

    // ----------PID controller----------
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++)
    {
        float currentPidSetpoint = getSetpointRate(axis);

        if (maxVelocity[axis])
        {
            currentPidSetpoint = accelerationLimit(axis, currentPidSetpoint);
        }

        // Yaw control is GYRO based, direct sticks control is applied to rate PID
        // NFE racermode applies angle only to the roll axis
        if (FLIGHT_MODE(GPS_RESCUE_MODE) && axis != FD_YAW)
        {
            currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint);
        }
        else if ((FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) && !nfe_racermode && (axis != FD_YAW))
        {
            currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint);
        }
        else if ((FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) && nfe_racermode && ((axis != FD_YAW) && (axis != FD_PITCH)))
        {
            currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint);
        }

        // Handle yaw spin recovery - zero the setpoint on yaw to aid in recovery
        // It's not necessary to zero the set points for R/P because the PIDs will be zeroed below
#ifdef USE_YAW_SPIN_RECOVERY
        if ((axis == FD_YAW) && gyroYawSpinDetected())
        {
            currentPidSetpoint = 0.0f;
        }
#endif // USE_YAW_SPIN_RECOVERY

        previousPidSetpoint[axis] = currentPidSetpoint;

        // -----calculate error rate
        errorRate = currentPidSetpoint - gyro.gyroADCf[axis]; // r - y

        // EmuFlight pid controller, which will be maintained in the future with additional features specialised for current (mini) multirotor usage.
        // Based on 2DOF reference design (matlab)

        const float gyroRate = gyro.gyroADCf[axis];
        float errorBoostAxis;
        float errorLimitAxis;

        if (axis <= FD_PITCH)
        {
            errorBoostAxis = pidProfile->errorBoost;
            errorLimitAxis = pidProfile->errorBoostLimit;
        }
        else
        {
            errorBoostAxis = pidProfile->errorBoostYaw;
            errorLimitAxis = pidProfile->errorBoostLimitYaw;
        }

        errorLimitAxis = errorLimitAxis / 100;
        float errorMultiplier = (errorBoostAxis * errorBoostAxis / 1000000) * 0.003;
        float boostedErrorRate;

        boostedErrorRate = (errorRate * fabsf(errorRate)) * errorMultiplier;

        if (fabsf(errorRate * errorLimitAxis) < fabsf(boostedErrorRate))
        {
            boostedErrorRate = errorRate * errorLimitAxis;
        }

        rotateITermAndAxisError();
        // --------low-level gyro-based PID based on 2DOF PID controller. ----------
        // 2-DOF PID controller with optional filter on derivative term.
        // derivative term can be based on measurement or error using a sliding value from 0-100

        // -----calculate P component
        pidData[axis].P = (pidCoefficient[axis].Kp * (boostedErrorRate + errorRate)) * vbatCompensationFactor;

        // -----calculate I component
        //float iterm = constrainf(pidData[axis].I + (pidCoefficient[axis].Ki * errorRate) * dynCi, -itermLimit, itermLimit);
        float iterm    = temporaryIterm[axis];
        float ITermNew = pidCoefficient[axis].Ki * (boostedErrorRate + errorRate) * dynCi;
        if (ITermNew != 0.0f)
        {
            if (SIGN(iterm) != SIGN(ITermNew))
            {
            	  const float newVal = ITermNew * iDecay;
            	  if (fabs(iterm) > fabs(newVal))
            	  {
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
        if (pidCoefficient[axis].Kd > 0)
        {
            //filter Kd properly, no setpoint filtering
            const float pureRD = getSetpointRate(axis) - gyroRate; // cr - y
            const float pureError = pureRD - previousError[axis];
            const float pureMeasurement = -(gyro.gyroADCf[axis] - previousMeasurement[axis]);
            previousMeasurement[axis] = gyro.gyroADCf[axis];
            previousError[axis] = pureRD;
            float dDelta = ((feathered_pids * pureMeasurement) + ((1 - feathered_pids) * pureError)) * pidFrequency; //calculating the dterm determine how much is calculated using measurement vs error
            //filter the dterm
            dDelta = dtermLowpassApplyFn((filter_t *)&dtermLowpass[axis], dDelta);
            dDelta = dtermLowpass2ApplyFn((filter_t *)&dtermLowpass2[axis], dDelta);

            if (pidProfile->dFilter[axis].Wc > 1)
            {
                kdRingBuffer[axis][kdRingBufferPoint[axis]++] = dDelta;
                kdRingBufferSum[axis] += dDelta;

                if (kdRingBufferPoint[axis] == pidProfile->dFilter[axis].Wc)
                {
                    kdRingBufferPoint[axis] = 0;
                }

                dDelta = (float)(kdRingBufferSum[axis] / (float)(pidProfile->dFilter[axis].Wc));
                kdRingBufferSum[axis] -= kdRingBuffer[axis][kdRingBufferPoint[axis]];
            }
            dDelta = pidCoefficient[axis].Kd * dDelta;

            float dDeltaMultiplier;

            if (smart_dterm_smoothing[axis] > 0)
            {
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
        }
        else
        {
            pidData[axis].D = 0;
        }

        handleCrashRecovery(pidProfile->crash_recovery, angleTrim, axis, currentTimeUs, errorRate, &currentPidSetpoint, &errorRate);

        detectAndSetCrashRecovery(pidProfile->crash_recovery, axis, currentTimeUs, pidData[axis].D, errorRate);

#ifdef USE_YAW_SPIN_RECOVERY
        if (gyroYawSpinDetected())
        {
            temporaryIterm[axis] = 0; // in yaw spin always disable I
            if (axis <= FD_PITCH)
            {
                // zero PIDs on pitch and roll leaving yaw P to correct spin
                pidData[axis].P = 0;
                pidData[axis].D = 0;
            }
        }
#endif // USE_YAW_SPIN_RECOVERY

        // Disable PID control if at zero throttle or if gyro overflow detected
        // This may look very innefficient, but it is done on purpose to always show real CPU usage as in flight
        if (!pidStabilisationEnabled || gyroOverflowDetected())
        {
            pidData[axis].P = 0;
            temporaryIterm[axis] = 0;
            pidData[axis].D = 0;

            pidData[axis].Sum = 0;
        }

        // calculating the PID sum and TPA and SPA
        // multiply these things to the pidData so that logs shows the pid data correctly
        pidData[axis].P = pidData[axis].P * getThrottlePAttenuation() * setPointPAttenuation[axis];
        pidData[axis].I = temporaryIterm[axis] * getThrottleIAttenuation() * setPointIAttenuation[axis]; // you can't use pidData[axis].I to calculate iterm or with tpa you get issues
        pidData[axis].D = pidData[axis].D * getThrottleDAttenuation() * setPointDAttenuation[axis];
        const float pidSum = pidData[axis].P + pidData[axis].I + pidData[axis].D;
        pidData[axis].Sum = pidSum;
    }
}

bool crashRecoveryModeActive(void)
{
    return inCrashRecoveryMode;
}

float pidGetPreviousSetpoint(int axis)
{
    return previousPidSetpoint[axis];
}
