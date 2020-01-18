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


extern float r_weight;

#define ITERM_RELAX_SETPOINT_THRESHOLD 30.0f

const char pidNames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "LEVEL;"
    "MAG;";

FAST_RAM_ZERO_INIT uint32_t targetPidLooptime;
FAST_RAM_ZERO_INIT pidAxisData_t pidData[XYZ_AXIS_COUNT];

static FAST_RAM_ZERO_INIT bool pidStabilisationEnabled;

static FAST_RAM_ZERO_INIT bool inCrashRecoveryMode = false;

static FAST_RAM_ZERO_INIT float dT;
static FAST_RAM_ZERO_INIT float pidFrequency;

static FAST_RAM_ZERO_INIT uint8_t antiGravityMode;
static FAST_RAM_ZERO_INIT float antiGravityThrottleHpf;
static FAST_RAM_ZERO_INIT uint16_t itermAcceleratorGain;
static FAST_RAM float antiGravityOsdCutoff = 1.0f;
static FAST_RAM_ZERO_INIT bool antiGravityEnabled;

PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 2);

#ifdef STM32F10X
#define PID_PROCESS_DENOM_DEFAULT       1
#elif defined(USE_GYRO_SPI_MPU6000) || defined(USE_GYRO_SPI_MPU6500)  || defined(USE_GYRO_SPI_ICM20689)
#define PID_PROCESS_DENOM_DEFAULT       4
#else
#define PID_PROCESS_DENOM_DEFAULT       2
#endif

#ifndef DEFAULT_PIDS_ROLL
#define DEFAULT_PIDS_ROLL { 50, 65, 28, 0, 3 }
#endif //DEFAULT_PIDS_ROLL

#ifndef DEFAULT_PIDS_PITCH
#define DEFAULT_PIDS_PITCH { 58, 65, 30, 0, 3 }
#endif //DEFAULT_PIDS_PITCH

#ifndef DEFAULT_PIDS_YAW
#define DEFAULT_PIDS_YAW { 55, 65, 5, 0, 3 }
#endif //DEFAULT_PIDS_YAW

#ifdef USE_RUNAWAY_TAKEOFF
PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = PID_PROCESS_DENOM_DEFAULT,
    .runaway_takeoff_prevention = true,
    .runaway_takeoff_deactivate_throttle = 20,  // throttle level % needed to accumulate deactivation time
    .runaway_takeoff_deactivate_delay = 500     // Accumulated time (in milliseconds) before deactivation in successful takeoff
);
#else
PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = PID_PROCESS_DENOM_DEFAULT
);
#endif

#ifdef USE_ACRO_TRAINER
#define ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT 500.0f  // Max gyro rate for lookahead time scaling
#define ACRO_TRAINER_SETPOINT_LIMIT       1000.0f // Limit the correcting setpoint
#endif // USE_ACRO_TRAINER

#define ANTI_GRAVITY_THROTTLE_FILTER_CUTOFF 15  // The anti gravity throttle highpass filter cutoff

PG_REGISTER_ARRAY_WITH_RESET_FN(pidProfile_t, PID_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 7);

void resetPidProfile(pidProfile_t *pidProfile)
{
    RESET_CONFIG(pidProfile_t, pidProfile,
        .pid = {
            [PID_ROLL] =  DEFAULT_PIDS_ROLL,
            [PID_PITCH] = DEFAULT_PIDS_PITCH,
            [PID_YAW] =   DEFAULT_PIDS_YAW,
            [PID_LEVEL_LOW] = { 100, 50, 10, 40, 0},
            [PID_LEVEL_HIGH] = { 35, 0, 1, 0, 0},
            [PID_MAG] =   { 40, 0, 0, 0, 0},
        },

        .pidSumLimit = PIDSUM_LIMIT_MAX,
        .pidSumLimitYaw = PIDSUM_LIMIT_YAW,
        .yaw_lowpass_hz = 0,
        .dterm_lowpass_hz = 65,     // filtering ON by default
        .dterm_lowpass2_hz = 200,   // second Dterm LPF ON by default
        .dterm_notch_hz = 0,
        .dterm_notch_cutoff = 0,
        .dterm_filter_type = FILTER_PT1,
        .smart_dterm_smoothing = 50,
        .itermWindupPointPercent = 50,
        .vbatPidCompensation = 0,
        .pidAtMinThrottle = PID_STABILISATION_ON,
        .levelAngleLimit = 65,
        .angleExpo = 30,
        .feedForwardTransition = 0,
        .setPointPTransition = 100,
        .setPointITransition = 100,
        .setPointDTransition = 100,
        .setPointPTransitionYaw = 100,
        .setPointITransitionYaw = 100,
        .setPointDTransitionYaw = 100,
        .feathered_pids = 100,
        .i_decay = 4,
        .r_weight = 67,
        .errorBoost = 15,
        .errorBoostYaw = 40,
        .errorBoostLimit = 20,
        .errorBoostLimitYaw = 40,
        .yawRateAccelLimit = 0,
        .rateAccelLimit = 0,
        .itermThrottleThreshold = 350,
        .itermAcceleratorGain = 1000,
        .crash_time = 500,          // ms
        .crash_delay = 0,           // ms
        .crash_recovery_angle = 10, // degrees
        .crash_recovery_rate = 100, // degrees/second
        .crash_dthreshold = 50,     // degrees/second/second
        .crash_gthreshold = 400,    // degrees/second
        .crash_setpoint_threshold = 350, // degrees/second
        .crash_recovery = PID_CRASH_RECOVERY_OFF, // off by default
        .horizon_tilt_effect = 130,
        .nfe_racermode = 0,
        .cinematic_setpoint = 0,
        .crash_limit_yaw = 200,
        .itermLimit = 400,
        .throttle_boost = 5,
        .throttle_boost_cutoff = 15,
        .iterm_rotation = true,
        .iterm_relax = ITERM_RELAX_OFF,
        .iterm_relax_cutoff = 11,
        .iterm_relax_type = ITERM_RELAX_GYRO,
        .acro_trainer_angle_limit = 20,
        .acro_trainer_lookahead_ms = 50,
        .acro_trainer_debug_axis = FD_ROLL,
        .acro_trainer_gain = 75,
        .abs_control_gain = 0,
        .abs_control_limit = 90,
        .abs_control_error_limit = 20,
        .antiGravityMode = ANTI_GRAVITY_SMOOTH,
        .use_integrated_yaw = false,
        .integrated_yaw_relax = 200,
        .motor_output_limit = 100,
        .auto_profile_cell_count = AUTO_PROFILE_CELL_COUNT_STAY,
        .horizonTransition = 0,
    );
}

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles)
{
  for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPidProfile(&pidProfiles[i]);
    }
}

extern void init_pwm_filter(float dT);

static void pidSetTargetLooptime(uint32_t pidLooptime)
{
    targetPidLooptime = pidLooptime;
    dT = targetPidLooptime * 1e-6f;
    pidFrequency = 1.0f / dT;
}

static FAST_RAM float itermAccelerator = 1.0f;

void pidSetItermAccelerator(float newItermAccelerator)
{
    itermAccelerator = newItermAccelerator;
}

bool pidOsdAntiGravityActive(void)
{
    return (itermAccelerator > antiGravityOsdCutoff);
}

void pidStabilisationState(pidStabilisationState_e pidControllerState)
{
    pidStabilisationEnabled = (pidControllerState == PID_STABILISATION_ON) ? true : false;
}

const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

typedef union dtermLowpass_u {
    pt1Filter_t pt1Filter;
    biquadFilter_t biquadFilter;
} dtermLowpass_t;

static FAST_RAM_ZERO_INIT float previousPidSetpoint[XYZ_AXIS_COUNT];
static FAST_RAM filterApplyFnPtr dtermNotchApplyFn = nullFilterApply;
static FAST_RAM_ZERO_INIT biquadFilter_t dtermNotch[3];
static FAST_RAM filterApplyFnPtr dtermLowpassApplyFn = nullFilterApply;
static FAST_RAM_ZERO_INIT dtermLowpass_t dtermLowpass[3];

#if defined(USE_ITERM_RELAX)
static FAST_RAM_ZERO_INIT pt1Filter_t windupLpf[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT uint8_t itermRelax;
static FAST_RAM_ZERO_INIT uint8_t itermRelaxType;
static FAST_RAM_ZERO_INIT uint8_t itermRelaxCutoff;
#endif

#ifdef USE_RC_SMOOTHING_FILTER
static FAST_RAM_ZERO_INIT pt1Filter_t setpointDerivativePt1[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT biquadFilter_t setpointDerivativeBiquad[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT bool setpointDerivativeLpfInitialized;
static FAST_RAM_ZERO_INIT uint8_t rcSmoothingDebugAxis;
static FAST_RAM_ZERO_INIT uint8_t rcSmoothingFilterType;
#endif // USE_RC_SMOOTHING_FILTER

static FAST_RAM_ZERO_INIT pt1Filter_t antiGravityThrottleLpf;

void pidInitFilters(const pidProfile_t *pidProfile)
{
    BUILD_BUG_ON(FD_YAW != 2); // ensure yaw axis is 2
    dtermNotchApplyFn = nullFilterApply;
    dtermLowpassApplyFn = nullFilterApply;
    const uint32_t pidFrequencyNyquist = pidFrequency / 2; // No rounding needed

    r_weight = (float) pidProfile->r_weight / 100.0f;

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
        dtermNotchApplyFn = (filterApplyFnPtr)biquadFilterApply;
        const float notchQ = filterGetNotchQ(dTermNotchHz, pidProfile->dterm_notch_cutoff);
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            biquadFilterInit(&dtermNotch[axis], dTermNotchHz, targetPidLooptime, notchQ, FILTER_NOTCH);
        }
    }

    if (pidProfile->dterm_lowpass_hz && pidProfile->dterm_lowpass_hz <= pidFrequencyNyquist)
    {
        for (int axis = FD_ROLL; axis <= FD_YAW; axis++)
        {
            switch (pidProfile->dterm_filter_type)
            {
            case FILTER_PT1:
                    dtermLowpassApplyFn = (filterApplyFnPtr)pt1FilterApply;
                    pt1FilterInit(&dtermLowpass[axis].pt1Filter, pt1FilterGain(pidProfile->dterm_lowpass_hz, dT));
                break;
            case FILTER_BIQUAD:
            default:
                    dtermLowpassApplyFn = (filterApplyFnPtr)biquadFilterApply;
                    biquadFilterInitLPF(&dtermLowpass[axis].biquadFilter, pidProfile->dterm_lowpass_hz, targetPidLooptime);
                break;
            }
        }
    }


#if defined(USE_THROTTLE_BOOST)
    pt1FilterInit(&throttleLpf, pt1FilterGain(pidProfile->throttle_boost_cutoff, dT));
#endif
#if defined(USE_ITERM_RELAX)
    if (itermRelax) {
        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
            pt1FilterInit(&windupLpf[i], pt1FilterGain(itermRelaxCutoff, dT));
        }
    }
#endif
    pt1FilterInit(&antiGravityThrottleLpf, pt1FilterGain(ANTI_GRAVITY_THROTTLE_FILTER_CUTOFF, dT));
}

#ifdef USE_RC_SMOOTHING_FILTER
void pidInitSetpointDerivativeLpf(uint16_t filterCutoff, uint8_t debugAxis, uint8_t filterType)
{
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

void pidUpdateSetpointDerivativeLpf(uint16_t filterCutoff)
{
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
static FAST_RAM_ZERO_INIT float maxVelocity[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float feedForwardTransition;
static FAST_RAM_ZERO_INIT float feathered_pids;
static FAST_RAM_ZERO_INIT uint8_t nfe_racermode;
static FAST_RAM_ZERO_INIT uint8_t cinematic_setpoint;
static FAST_RAM_ZERO_INIT float smart_dterm_smoothing;
static FAST_RAM_ZERO_INIT float setPointPTransition;
static FAST_RAM_ZERO_INIT float setPointITransition;
static FAST_RAM_ZERO_INIT float setPointDTransition;
static FAST_RAM_ZERO_INIT float setPointPTransitionYaw;
static FAST_RAM_ZERO_INIT float setPointITransitionYaw;
static FAST_RAM_ZERO_INIT float setPointDTransitionYaw;
static FAST_RAM_ZERO_INIT float P_angle_low, I_angle_low, D_angle_low, P_angle_high, I_angle_high, D_angle_high, F_angle, horizonTransition, horizonCutoffDegrees, horizonFactorRatio;
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

#if defined(USE_ABSOLUTE_CONTROL)
static FAST_RAM_ZERO_INIT float axisError[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float acGain;
static FAST_RAM_ZERO_INIT float acLimit;
static FAST_RAM_ZERO_INIT float acErrorLimit;
#endif

#ifdef USE_INTEGRATED_YAW_CONTROL
static FAST_RAM_ZERO_INIT bool useIntegratedYaw;
static FAST_RAM_ZERO_INIT uint8_t integratedYawRelax;
#endif

void pidResetITerm(void)
{
    for (int axis = 0; axis < 3; axis++) {
        pidData[axis].I = 0.0f;
#if defined(USE_ABSOLUTE_CONTROL)
        axisError[axis] = 0.0f;
#endif
    }
}


#ifdef USE_ACRO_TRAINER
static FAST_RAM_ZERO_INIT float acroTrainerAngleLimit;
static FAST_RAM_ZERO_INIT float acroTrainerLookaheadTime;
static FAST_RAM_ZERO_INIT uint8_t acroTrainerDebugAxis;
static FAST_RAM_ZERO_INIT bool acroTrainerActive;
static FAST_RAM_ZERO_INIT int acroTrainerAxisState[2];  // only need roll and pitch
static FAST_RAM_ZERO_INIT float acroTrainerGain;
#endif // USE_ACRO_TRAINER

void pidUpdateAntiGravityThrottleFilter(float throttle)
{
    if (antiGravityMode == ANTI_GRAVITY_SMOOTH) {
        antiGravityThrottleHpf = throttle - pt1FilterApply(&antiGravityThrottleLpf, throttle);
    }
}

void pidInitConfig(const pidProfile_t *pidProfile)
{
    if (pidProfile->feedForwardTransition == 0) {
        feedForwardTransition = 0;
    } else {
        feedForwardTransition = 100.0f / pidProfile->feedForwardTransition;
    }

    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        pidCoefficient[axis].Kp = PTERM_SCALE * pidProfile->pid[axis].P;
        pidCoefficient[axis].Ki = ITERM_SCALE * pidProfile->pid[axis].I;
        pidCoefficient[axis].Kd = DTERM_SCALE * pidProfile->pid[axis].D;
        pidCoefficient[axis].Kf = FEEDFORWARD_SCALE * (pidProfile->pid[axis].F / 100.0f);
    }

    feathered_pids = pidProfile->feathered_pids / 100.0f;
    nfe_racermode = pidProfile->nfe_racermode;
    cinematic_setpoint = pidProfile->cinematic_setpoint;
    smart_dterm_smoothing = pidProfile->smart_dterm_smoothing;
    setPointPTransition = pidProfile->setPointPTransition / 100.0f;
    setPointITransition = pidProfile->setPointITransition / 100.0f;
    setPointDTransition = pidProfile->setPointDTransition / 100.0f;
    setPointPTransitionYaw = pidProfile->setPointPTransitionYaw / 100.0f;
    setPointITransitionYaw = pidProfile->setPointITransitionYaw / 100.0f;
    setPointDTransitionYaw = pidProfile->setPointDTransitionYaw / 100.0f;
    P_angle_low = pidProfile->pid[PID_LEVEL_LOW].P * 0.1f;
    I_angle_low = pidProfile->pid[PID_LEVEL_LOW].I * 0.76f;
    D_angle_low = pidProfile->pid[PID_LEVEL_LOW].D * 0.00017f;
    P_angle_high = pidProfile->pid[PID_LEVEL_HIGH].P * 0.1f;
    I_angle_high = pidProfile->pid[PID_LEVEL_HIGH].I * 0.76f;
    D_angle_high = pidProfile->pid[PID_LEVEL_HIGH].D * 0.00017f;
    F_angle = pidProfile->pid[PID_LEVEL_LOW].F * 0.00000125f;
    horizonTransition = (float)pidProfile->horizonTransition;
    horizonCutoffDegrees = pidProfile->horizon_tilt_effect;
    horizonFactorRatio = (100 - pidProfile->horizon_tilt_effect) * 0.01f;
    maxVelocity[FD_ROLL] = maxVelocity[FD_PITCH] = pidProfile->rateAccelLimit * 100 * dT;
    maxVelocity[FD_YAW] = pidProfile->yawRateAccelLimit * 100 * dT;
    const float ITermWindupPoint = (float)pidProfile->itermWindupPointPercent / 100.0f;
    ITermWindupPointInv = 1.0f / (1.0f - ITermWindupPoint);
    itermAcceleratorGain = pidProfile->itermAcceleratorGain;
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
    antiGravityMode = pidProfile->antiGravityMode;

    // Calculate the anti-gravity value that will trigger the OSD display.
    // For classic AG it's either 1.0 for off and > 1.0 for on.
    // For the new AG it's a continuous floating value so we want to trigger the OSD
    // display when it exceeds 25% of its possible range. This gives a useful indication
    // of AG activity without excessive display.
    antiGravityOsdCutoff = 1.0f;
    if (antiGravityMode == ANTI_GRAVITY_SMOOTH) {
        antiGravityOsdCutoff += ((itermAcceleratorGain - 1000) / 1000.0f) * 0.25f;
    }


#if defined(USE_ITERM_RELAX)
    itermRelax = pidProfile->iterm_relax;
    itermRelaxType = pidProfile->iterm_relax_type;
    itermRelaxCutoff = pidProfile->iterm_relax_cutoff;
#endif

#ifdef USE_ACRO_TRAINER
    acroTrainerAngleLimit = pidProfile->acro_trainer_angle_limit;
    acroTrainerLookaheadTime = (float)pidProfile->acro_trainer_lookahead_ms / 1000.0f;
    acroTrainerDebugAxis = pidProfile->acro_trainer_debug_axis;
    acroTrainerGain = (float)pidProfile->acro_trainer_gain / 10.0f;
#endif // USE_ACRO_TRAINER

#if defined(USE_ABSOLUTE_CONTROL)
    acGain = (float)pidProfile->abs_control_gain;
    acLimit = (float)pidProfile->abs_control_limit;
    acErrorLimit = (float)pidProfile->abs_control_error_limit;
#endif

#ifdef USE_INTEGRATED_YAW_CONTROL
    useIntegratedYaw = pidProfile->use_integrated_yaw;
    integratedYawRelax = pidProfile->integrated_yaw_relax;
#endif
}

void pidInit(const pidProfile_t *pidProfile)
{
    pidSetTargetLooptime(gyro.targetLooptime * pidConfig()->pid_process_denom); // Initialize pid looptime
    pidInitFilters(pidProfile);
    pidInitConfig(pidProfile);
}

#ifdef USE_ACRO_TRAINER
void pidAcroTrainerInit(void)
{
    acroTrainerAxisState[FD_ROLL] = 0;
    acroTrainerAxisState[FD_PITCH] = 0;
}
#endif // USE_ACRO_TRAINER

void pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex)
{

   if ((dstPidProfileIndex < PID_PROFILE_COUNT-1 && srcPidProfileIndex < PID_PROFILE_COUNT-1)
        && dstPidProfileIndex != srcPidProfileIndex
    ) {
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

      if (horizonCutoffDegrees > 0 && horizonTransition < horizonCutoffDegrees) { //if racemode_tilt_effect>0 and if horizonTransition<racemode_tilt_effect

  //causes leveling to fade from horizonTransition angle to horizonCutoffDegrees	where leveling goes to zero
        const float inclinationLevelRatio = constrainf(((horizonCutoffDegrees - currentInclination) * racemodeHorizonTransitionFactor) / horizonCutoffDegrees, 0, 1);
        // apply inclination ratio to horizonLevelStrength which lowers leveling to zero as a function of angle and regardless of stick position
        horizonLevelStrength = inclinationLevelRatio;
      } else  { // if racemode_tilt_effect = 0 or horizonTransition>racemode_tilt_effect means no leveling
        horizonLevelStrength = 0;
      }
return constrainf(horizonLevelStrength, 0, 1);
}

#define SIGN(x) ((x > 0.0f) - (x < 0.0f))

static float pidLevel(int axis, const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim, float currentPidSetpoint, const float deltaT) {
    // calculate error angle and limit the angle to the max inclination
    // rcDeflection is in range [-1.0, 1.0]
    static float i_term[2], attitudePrevious[2], previousAngle[2];
    float p_term_low, p_term_high, d_term_low, d_term_high, f_term_low;

    float angle = pidProfile->levelAngleLimit * getRcDeflection(axis);
    if (pidProfile->angleExpo > 0) {
      const float expof = pidProfile->angleExpo / 100.0f;
      angle = pidProfile->levelAngleLimit * (getRcDeflection(axis) * power3(fabsf(getRcDeflection(axis))) * expof + getRcDeflection(axis) * (1 - expof));
    }

#ifdef USE_GPS_RESCUE
    angle += gpsRescueAngle[axis] / 100; // ANGLE IS IN CENTIDEGREES
#endif

    f_term_low = (angle - previousAngle[axis]) * F_angle * pidFrequency;
    previousAngle[axis] = angle;

    angle = constrainf(angle, -pidProfile->levelAngleLimit, pidProfile->levelAngleLimit);
    float errorAngle = angle - ((attitude.raw[axis] - angleTrim->raw[axis]) * 0.1f);
    errorAngle = constrainf(errorAngle, -90.0f, 90.0f);
    const float errorAnglePercent = errorAngle / 90.0f;

    // ANGLE mode - control is angle based
    p_term_low = (1 - fabsf(errorAnglePercent)) * errorAngle * P_angle_low;
    p_term_high = fabsf(errorAnglePercent) * errorAngle * P_angle_high;

    float i_new_low = (1 - fabsf(errorAnglePercent)) * errorAngle * deltaT * I_angle_low;
    float i_new_high = fabsf(errorAnglePercent) * errorAngle * deltaT * I_angle_high;
    if (i_new_low != 0.0f)
{
    if (SIGN(i_term[axis]) != SIGN(i_new_low))
    {
      i_term[axis] *= 0.70f;
    }
}
    i_term[axis] += i_new_low + i_new_high;

    d_term_low = (1 - fabsf(errorAnglePercent)) * (attitudePrevious[axis] - attitude.raw[axis]) * 0.1f * D_angle_low;
    d_term_high = fabsf(errorAnglePercent) * (attitudePrevious[axis] - attitude.raw[axis]) * 0.1f * D_angle_high;
    attitudePrevious[axis] = attitude.raw[axis];

    currentPidSetpoint = p_term_low + p_term_high;
    currentPidSetpoint += i_term[axis];
    currentPidSetpoint += d_term_low + d_term_high;
    currentPidSetpoint += f_term_low;

if (FLIGHT_MODE(HORIZON_MODE)) {
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

    if (ABS(currentVelocity) > maxVelocity[axis]) {
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
                const float errorAngle =  -(attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
                *currentPidSetpoint = errorAngle * P_angle_low;
                *errorRate = *currentPidSetpoint - gyroRate;
            }
        }
        // reset ITerm, since accumulated error before crash is now meaningless
        // and ITerm windup during crash recovery can be extreme, especially on yaw axis
        pidData[axis].I = 0.0f;
        if (cmpTimeUs(currentTimeUs, crashDetectedAtUs) > crashTimeLimitUs
            || (getMotorMixRange() < 1.0f
                   && ABS(gyro.gyroADCf[FD_ROLL]) < crashRecoveryRate
                   && ABS(gyro.gyroADCf[FD_PITCH]) < crashRecoveryRate
                   && ABS(gyro.gyroADCf[FD_YAW]) < crashRecoveryRate)) {
            if (sensors(SENSOR_ACC)) {
                // check aircraft nearly level
                if (ABS(attitude.raw[FD_ROLL] - angleTrim->raw[FD_ROLL]) < crashRecoveryAngleDeciDegrees
                   && ABS(attitude.raw[FD_PITCH] - angleTrim->raw[FD_PITCH]) < crashRecoveryAngleDeciDegrees) {
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
    const timeUs_t currentTimeUs, const float delta, const float errorRate)
{
    // if crash recovery is on and accelerometer enabled and there is no gyro overflow, then check for a crash
    // no point in trying to recover if the crash is so severe that the gyro overflows
    if ((crash_recovery || FLIGHT_MODE(GPS_RESCUE_MODE)) && !gyroOverflowDetected()) {
        if (ARMING_FLAG(ARMED)) {
            if (getMotorMixRange() >= 1.0f && !inCrashRecoveryMode
                && ABS(delta) > crashDtermThreshold
                && ABS(errorRate) > crashGyroThreshold
                && ABS(getSetpointRate(axis)) < crashSetpointThreshold) {
                inCrashRecoveryMode = true;
                crashDetectedAtUs = currentTimeUs;
            }
            if (inCrashRecoveryMode && cmpTimeUs(currentTimeUs, crashDetectedAtUs) < crashTimeDelayUs && (ABS(errorRate) < crashGyroThreshold
                || ABS(getSetpointRate(axis)) > crashSetpointThreshold)) {
                inCrashRecoveryMode = false;
                BEEP_OFF;
            }
        } else if (inCrashRecoveryMode) {
            inCrashRecoveryMode = false;
            BEEP_OFF;
        }
    }
}

static void rotateVector(float v[XYZ_AXIS_COUNT], float rotation[XYZ_AXIS_COUNT])
{
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

static void rotateITermAndAxisError()
{
    if (itermRotation
#if defined(USE_ABSOLUTE_CONTROL)
        || acGain > 0
#endif
        ) {
        const float gyroToAngle = dT * RAD;
        float rotationRads[XYZ_AXIS_COUNT];
        for (int i = FD_ROLL; i <= FD_YAW; i++) {
            rotationRads[i] = gyro.gyroADCf[i] * gyroToAngle;
        }
#if defined(USE_ABSOLUTE_CONTROL)
        if (acGain > 0) {
            rotateVector(axisError, rotationRads);
        }
#endif
        if (itermRotation) {
            float v[XYZ_AXIS_COUNT];
            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                v[i] = pidData[i].I;
            }
            rotateVector(v, rotationRads );
            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                pidData[i].I = v[i];
            }
        }
    }
}

#ifdef USE_ACRO_TRAINER

int acroTrainerSign(float x)
{
    return x > 0 ? 1 : -1;
}

// Acro Trainer - Manipulate the setPoint to limit axis angle while in acro mode
// There are three states:
// 1. Current angle has exceeded limit
//    Apply correction to return to limit (similar to pidLevel)
// 2. Future overflow has been projected based on current angle and gyro rate
//    Manage the setPoint to control the gyro rate as the actual angle  approaches the limit (try to prevent overshoot)
// 3. If no potential overflow is detected, then return the original setPoint

// Use the FAST_CODE_NOINLINE directive to avoid this code from being inlined into ITCM RAM. We accept the
// performance decrease when Acro Trainer mode is active under the assumption that user is unlikely to be
// expecting ultimate flight performance at very high loop rates when in this mode.
static FAST_CODE_NOINLINE float applyAcroTrainer(int axis, const rollAndPitchTrims_t *angleTrim, float setPoint)
{
    float ret = setPoint;

    if (!FLIGHT_MODE(ANGLE_MODE) && !FLIGHT_MODE(HORIZON_MODE) && !FLIGHT_MODE(GPS_RESCUE_MODE)) {
        bool resetIterm = false;
        float projectedAngle = 0;
        const int setpointSign = acroTrainerSign(setPoint);
        const float currentAngle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
        const int angleSign = acroTrainerSign(currentAngle);

        if ((acroTrainerAxisState[axis] != 0) && (acroTrainerAxisState[axis] != setpointSign)) {  // stick has reversed - stop limiting
            acroTrainerAxisState[axis] = 0;
        }

        // Limit and correct the angle when it exceeds the limit
        if ((fabsf(currentAngle) > acroTrainerAngleLimit) && (acroTrainerAxisState[axis] == 0)) {
            if (angleSign == setpointSign) {
                acroTrainerAxisState[axis] = angleSign;
                resetIterm = true;
            }
        }

        if (acroTrainerAxisState[axis] != 0) {
            ret = constrainf(((acroTrainerAngleLimit * angleSign) - currentAngle) * acroTrainerGain, -ACRO_TRAINER_SETPOINT_LIMIT, ACRO_TRAINER_SETPOINT_LIMIT);
        } else {

        // Not currently over the limit so project the angle based on current angle and
        // gyro angular rate using a sliding window based on gyro rate (faster rotation means larger window.
        // If the projected angle exceeds the limit then apply limiting to minimize overshoot.
        // Calculate the lookahead window by scaling proportionally with gyro rate from 0-500dps
            float checkInterval = constrainf(fabsf(gyro.gyroADCf[axis]) / ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT, 0.0f, 1.0f) * acroTrainerLookaheadTime;
            projectedAngle = (gyro.gyroADCf[axis] * checkInterval) + currentAngle;
            const int projectedAngleSign = acroTrainerSign(projectedAngle);
            if ((fabsf(projectedAngle) > acroTrainerAngleLimit) && (projectedAngleSign == setpointSign)) {
                ret = ((acroTrainerAngleLimit * projectedAngleSign) - projectedAngle) * acroTrainerGain;
                resetIterm = true;
            }
        }

        if (resetIterm) {
            pidData[axis].I = 0;
        }

        if (axis == acroTrainerDebugAxis) {
            DEBUG_SET(DEBUG_ACRO_TRAINER, 0, lrintf(currentAngle * 10.0f));
            DEBUG_SET(DEBUG_ACRO_TRAINER, 1, acroTrainerAxisState[axis]);
            DEBUG_SET(DEBUG_ACRO_TRAINER, 2, lrintf(ret));
            DEBUG_SET(DEBUG_ACRO_TRAINER, 3, lrintf(projectedAngle * 10.0f));
        }
    }

    return ret;
}
#endif // USE_ACRO_TRAINER

#ifdef USE_RC_SMOOTHING_FILTER
float FAST_CODE applyRcSmoothingDerivativeFilter(int axis, float pidSetpointDelta)
{
    float ret = pidSetpointDelta;
    if (axis == rcSmoothingDebugAxis) {
        DEBUG_SET(DEBUG_RC_SMOOTHING, 1, lrintf(pidSetpointDelta * 100.0f));
    }
    if (setpointDerivativeLpfInitialized) {
        switch (rcSmoothingFilterType) {
            case RC_SMOOTHING_DERIVATIVE_PT1:
                ret = pt1FilterApply(&setpointDerivativePt1[axis], pidSetpointDelta);
                break;
            case RC_SMOOTHING_DERIVATIVE_BIQUAD:
                ret = biquadFilterApplyDF1(&setpointDerivativeBiquad[axis], pidSetpointDelta);
                break;
        }
        if (axis == rcSmoothingDebugAxis) {
            DEBUG_SET(DEBUG_RC_SMOOTHING, 2, lrintf(ret * 100.0f));
        }
    }
    return ret;
}
#endif // USE_RC_SMOOTHING_FILTER


static FAST_RAM_ZERO_INIT float previousError[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float previousMeasurement[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float previousdDelta[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT float kdRingBuffer[XYZ_AXIS_COUNT][KD_RING_BUFFER_SIZE];
static FAST_RAM_ZERO_INIT float kdRingBufferSum[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT uint16_t kdRingBufferPoint[XYZ_AXIS_COUNT];
static FAST_RAM_ZERO_INIT timeUs_t crashDetectedAtUs;
static FAST_RAM_ZERO_INIT timeUs_t previousTimeUs;

    void pidController(const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim, timeUs_t currentTimeUs)
    {
    const float deltaT = (currentTimeUs - previousTimeUs) * 1e-6f;
    previousTimeUs = currentTimeUs;
    // calculate actual deltaT in seconds
    const float iDT = 1.0f/deltaT; //divide once
    // calculate actual deltaT in seconds
    // Dynamic i component,
    if ((antiGravityMode == ANTI_GRAVITY_SMOOTH) && antiGravityEnabled) {
        itermAccelerator = 1 + fabsf(antiGravityThrottleHpf) * 0.01f * (itermAcceleratorGain - 1000);
        DEBUG_SET(DEBUG_ANTI_GRAVITY, 1, lrintf(antiGravityThrottleHpf * 1000));
    }
    DEBUG_SET(DEBUG_ANTI_GRAVITY, 0, lrintf(itermAccelerator * 1000));


    // gradually scale back integration when above windup point
    const float dynCi = constrainf((1.1f - getMotorMixRange()) * ITermWindupPointInv, 0.1f, 1.0f) * deltaT * itermAccelerator;
    float errorRate;

    // ----------PID controller----------
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        float currentPidSetpoint = getSetpointRate(axis);
        if (maxVelocity[axis]) {
            currentPidSetpoint = accelerationLimit(axis, currentPidSetpoint);
        }

        // Yaw control is GYRO based, direct sticks control is applied to rate PID
        // NFE racermode applies angle only to the roll axis
        if (FLIGHT_MODE(GPS_RESCUE_MODE) && axis != FD_YAW) {
            currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint, deltaT);
        } else if ((FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) && !nfe_racermode && (axis != FD_YAW)) {
            currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint, deltaT);
        } else if ((FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) && nfe_racermode && ((axis != FD_YAW) && (axis != FD_PITCH))) {
            currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint, deltaT);
        }

        // -----calculate feedforward component
        // Use angle feedforward for angle mode and level feedforward for pitch/roll or roll if in nfe_racermode
        float feedforwardGain;

        if (!FLIGHT_MODE(GPS_RESCUE_MODE)) {
        feedforwardGain = pidCoefficient[axis].Kf;
      } else {
        feedforwardGain = 0;
      }


            float pidSetpointDelta = currentPidSetpoint - previousPidSetpoint[axis];

#ifdef USE_RC_SMOOTHING_FILTER
            pidSetpointDelta = applyRcSmoothingDerivativeFilter(axis, pidSetpointDelta);
#endif // USE_RC_SMOOTHING_FILTER
            float transition = 1.0f;

                //calculate the cinematic_setpoint code and apply change to the setPoint
            if (cinematic_setpoint) {
               transition = MAX(4 - (4 * (currentPidSetpoint / 1000.0f)), 1.0f);
               float setpointSmoother = MIN(.95f,((pidSetpointDelta * 0.041262f) / 1000.0f) * transition);
               currentPidSetpoint = currentPidSetpoint - (pidSetpointDelta * setpointSmoother);
               pidData[axis].F = 0;
            } else if (feedforwardGain > 0) {
            // no transition if feedForwardTransition == 0 or cinematic_setpoint is enabled
            transition = MIN(1.0f, getRcDeflectionAbs(axis) * feedForwardTransition);
            pidData[axis].F = feedforwardGain * transition * pidSetpointDelta * iDT;
          }

        previousPidSetpoint[axis] = currentPidSetpoint;


#ifdef USE_ACRO_TRAINER
        if ((axis != FD_YAW) && acroTrainerActive && !inCrashRecoveryMode) {
            currentPidSetpoint = applyAcroTrainer(axis, angleTrim, currentPidSetpoint);
        }
#endif // USE_ACRO_TRAINER

        // Handle yaw spin recovery - zero the setpoint on yaw to aid in recovery
        // It's not necessary to zero the set points for R/P because the PIDs will be zeroed below
#ifdef USE_YAW_SPIN_RECOVERY
        if ((axis == FD_YAW) && gyroYawSpinDetected()) {
            currentPidSetpoint = 0.0f;
        }
#endif // USE_YAW_SPIN_RECOVERY

        // -----calculate error rate
        errorRate = currentPidSetpoint - gyro.gyroADCf[axis]; // r - y

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

        if (fabsf(errorRate * errorLimitAxis) < fabsf(boostedErrorRate))
          {
            boostedErrorRate = errorRate * errorLimitAxis;
        }

            rotateITermAndAxisError();
            // --------low-level gyro-based PID based on 2DOF PID controller. ----------
            // 2-DOF PID controller with optional filter on derivative term.
            // b = 1 and only c (feedforward weight) can be tuned (amount derivative on measurement or error).

        #ifdef USE_ABSOLUTE_CONTROL
            float acCorrection = 0;
            float acErrorRate;
        #endif

                const float gyroRate = gyro.gyroADCf[axis];
                float ITerm = pidData[axis].I;
                float itermErrorRate = boostedErrorRate + errorRate;

        #if defined(USE_ITERM_RELAX)
            if (itermRelax && (axis < FD_YAW || itermRelax == ITERM_RELAX_RPY || itermRelax == ITERM_RELAX_RPY_INC)) {
                const float setpointLpf = pt1FilterApply(&windupLpf[axis], currentPidSetpoint);
                const float setpointHpf = fabsf(currentPidSetpoint - setpointLpf);
                const float itermRelaxFactor = 1 - setpointHpf / ITERM_RELAX_SETPOINT_THRESHOLD;

                const bool isDecreasingI = ((ITerm > 0) && (itermErrorRate < 0)) || ((ITerm < 0) && (itermErrorRate > 0));
                if ((itermRelax >= ITERM_RELAX_RP_INC) && isDecreasingI) {
                    // Do Nothing, use the precalculed itermErrorRate
                } else if (itermRelaxType == ITERM_RELAX_SETPOINT && setpointHpf < ITERM_RELAX_SETPOINT_THRESHOLD) {
                    itermErrorRate *= itermRelaxFactor;
                } else if (itermRelaxType == ITERM_RELAX_GYRO ) {
                    itermErrorRate = fapplyDeadband(setpointLpf - gyroRate, setpointHpf);
                } else {
                    itermErrorRate = 0.0f;
                }

                if (axis == FD_ROLL) {
                    DEBUG_SET(DEBUG_ITERM_RELAX, 0, lrintf(setpointHpf));
                    DEBUG_SET(DEBUG_ITERM_RELAX, 1, lrintf(itermRelaxFactor * 100.0f));
                    DEBUG_SET(DEBUG_ITERM_RELAX, 2, lrintf(itermErrorRate));
                }

        #if defined(USE_ABSOLUTE_CONTROL)
                const float gmaxac = setpointLpf + 2 * setpointHpf;
                const float gminac = setpointLpf - 2 * setpointHpf;
                if (gyroRate >= gminac && gyroRate <= gmaxac) {
                    float acErrorRate1 = gmaxac - gyroRate;
                    float acErrorRate2 = gminac - gyroRate;
                    if (acErrorRate1 * axisError[axis] < 0) {
                        acErrorRate = acErrorRate1;
                    } else {
                        acErrorRate = acErrorRate2;
                    }
                    if (fabsf(acErrorRate * dT) > fabsf(axisError[axis]) ) {
                        acErrorRate = -axisError[axis] / dT;
                    }
                } else {
                    acErrorRate = (gyroRate > gmaxac ? gmaxac : gminac ) - gyroRate;
                }
        #endif // USE_ABSOLUTE_CONTROL
            } else
        #endif // USE_ITERM_RELAX
            {
        #if defined(USE_ABSOLUTE_CONTROL)
                acErrorRate = itermErrorRate;
        #endif // USE_ABSOLUTE_CONTROL
            }

        #if defined(USE_ABSOLUTE_CONTROL)
            if (acGain > 0 && isAirmodeActivated()) {
                axisError[axis] = constrainf(axisError[axis] + acErrorRate * dT, -acErrorLimit, acErrorLimit);
                acCorrection = constrainf(axisError[axis] * acGain, -acLimit, acLimit);
                currentPidSetpoint += acCorrection;
                itermErrorRate += acCorrection;
                if (axis == FD_ROLL) {
                    DEBUG_SET(DEBUG_ITERM_RELAX, 3, lrintf(axisError[axis] * 10));
                }
            }
        #endif

                // -----calculate P component and add Dynamic Part based on stick input
            pidData[axis].P = (pidCoefficient[axis].Kp * (boostedErrorRate + errorRate));
            // -----calculate I component
            float ITermNew = pidCoefficient[axis].Ki * itermErrorRate * dynCi;
            if (ITermNew != 0.0f)
            {
                if (SIGN(ITerm) != SIGN(ITermNew))
                {
                	const float newVal = ITermNew * (float)pidProfile->i_decay;
                	if (fabsf(ITerm) > fabsf(newVal))
                	{
                		ITermNew = newVal;
                	}
                }
            }
            ITermNew = constrainf(ITerm + ITermNew, -itermLimit, itermLimit);

            const bool outputSaturated = mixerIsOutputSaturated(axis, errorRate);
            if (outputSaturated == false || ABS(ITermNew) < ABS(ITerm)) {
                // Only increase ITerm if output is not saturated
                pidData[axis].I = ITermNew;
            }

            // -----calculate D component
            float gyroRateFiltered = dtermNotchApplyFn((filter_t *) &dtermNotch[axis], gyroRate);

                //filter Kd properly, no setpoint filtering
                const float pureRD = getSetpointRate(axis) - gyroRateFiltered;    // cr - y
                const float pureError = pureRD - previousError[axis];
                const float pureMeasurement = -(gyro.gyroADCf[axis] - previousMeasurement[axis]);
                previousMeasurement[axis] = gyro.gyroADCf[axis];
                previousError[axis] = pureRD;
                float dDelta = ((feathered_pids * pureMeasurement) + ((1 - feathered_pids) * pureError)) * iDT; //calculating the dterm
                //filter the dterm
                dDelta = dtermLowpassApplyFn((filter_t *) &dtermLowpass[axis], dDelta);

                float dDeltaMultiplier = constrainf(fabsf(dDelta + previousdDelta[axis]) / (2 * smart_dterm_smoothing), 0.0f, 1.0f);
                dDelta = dDelta * dDeltaMultiplier;
                previousdDelta[axis] = dDelta;

            if (pidProfile->pid[axis].Wc > 1)
              {
                kdRingBuffer[axis][kdRingBufferPoint[axis]++] = dDelta;
                kdRingBufferSum[axis] += dDelta;

                if (kdRingBufferPoint[axis] == pidProfile->pid[axis].Wc) {
                kdRingBufferPoint[axis] = 0;
                }

                dDelta = (float)(kdRingBufferSum[axis] / (float) (pidProfile->pid[axis].Wc));
                kdRingBufferSum[axis] -= kdRingBuffer[axis][kdRingBufferPoint[axis]];
              }

        if (pidCoefficient[axis].Kd > 0) {
                // Divide rate change by dT to get differential (ie dr/dt).
                // dT is fixed and calculated from the target PID loop time
                // This is done to avoid DTerm spikes that occur with dynamically
                // calculated deltaT whenever another task causes the PID
                // loop execution to be delayed.
                pidData[axis].D = pidCoefficient[axis].Kd * dDelta;
            } else {
                pidData[axis].D = 0;
            }

            handleCrashRecovery(
                pidProfile->crash_recovery, angleTrim, axis, currentTimeUs, errorRate,
                &currentPidSetpoint, &errorRate);


            detectAndSetCrashRecovery(pidProfile->crash_recovery, axis, currentTimeUs, dDelta, errorRate);

           // calculate SPA
           float setPointPAttenuation;
           float setPointIAttenuation;
           float setPointDAttenuation;

           // SPA boost if SPA > 100 SPA cut if SPA < 100
           if (axis <= FD_PITCH) {
           setPointPAttenuation = 1 + (getRcDeflectionAbs(axis) * (setPointPTransition - 1));
           setPointIAttenuation = 1 + (getRcDeflectionAbs(axis) * (setPointITransition - 1));
           setPointDAttenuation = 1 + (getRcDeflectionAbs(axis) * (setPointDTransition - 1));
         } else {
           setPointPAttenuation = 1 + (getRcDeflectionAbs(axis) * (setPointPTransitionYaw - 1));
           setPointIAttenuation = 1 + (getRcDeflectionAbs(axis) * (setPointITransitionYaw - 1));
           setPointDAttenuation = 1 + (getRcDeflectionAbs(axis) * (setPointDTransitionYaw - 1));
         }
#ifdef USE_YAW_SPIN_RECOVERY
        if (gyroYawSpinDetected()) {
            pidData[axis].I = 0;  // in yaw spin always disable I
            if (axis <= FD_PITCH)  {
                // zero PIDs on pitch and roll leaving yaw P to correct spin
                pidData[axis].P = 0;
                pidData[axis].D = 0;
                pidData[axis].F = 0;
            }
        }
#endif // USE_YAW_SPIN_RECOVERY
    // Disable PID control if at zero throttle or if gyro overflow detected
    // This may look very innefficient, but it is done on purpose to always show real CPU usage as in flight
        if (!pidStabilisationEnabled || gyroOverflowDetected()) {
            pidData[axis].P = 0;
            pidData[axis].I = 0;
            pidData[axis].D = 0;
            pidData[axis].F = 0;

            pidData[axis].Sum = 0;
        }
        // calculating the PID sum and TPA and SPA
        const float pidSum = (pidData[axis].P * getThrottlePAttenuation() * setPointPAttenuation) + (pidData[axis].I * getThrottleIAttenuation() * setPointIAttenuation) + (pidData[axis].D * getThrottleDAttenuation() * setPointDAttenuation) + pidData[axis].F;
#ifdef USE_INTEGRATED_YAW_CONTROL
        if (axis == FD_YAW && useIntegratedYaw) {
            pidData[axis].Sum += pidSum * dT * 100.0f;
            pidData[axis].Sum -= pidData[axis].Sum * integratedYawRelax / 100000.0f * dT / 0.000125f;
        } else
#endif
        {
            pidData[axis].Sum = pidSum;
        }
    }
}

bool crashRecoveryModeActive(void)
{
    return inCrashRecoveryMode;
}

#ifdef USE_ACRO_TRAINER
void pidSetAcroTrainerState(bool newState)
{
    if (acroTrainerActive != newState) {
        if (newState) {
            pidAcroTrainerInit();
        }
        acroTrainerActive = newState;
    }
}
#endif // USE_ACRO_TRAINER

void pidSetAntiGravityState(bool newState)
{
    if (newState != antiGravityEnabled) {
        // reset the accelerator on state changes
        itermAccelerator = 1.0f;
    }
    antiGravityEnabled = newState;
}

bool pidAntiGravityEnabled(void)
{
    return antiGravityEnabled;
}


float pidGetPreviousSetpoint(int axis)
{
    return previousPidSetpoint[axis];
}
