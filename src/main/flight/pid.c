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
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"

#include "config/config_reset.h"

#include "drivers/pwm_output.h"
#include "drivers/sound_beeper.h"
#include "drivers/time.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/rpm_filter.h"
#include "flight/feedforward.h"

#include "io/gps.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "pid.h"

typedef enum {
    LEVEL_MODE_OFF = 0,
    LEVEL_MODE_R,
    LEVEL_MODE_RP,
} levelMode_e;

const char pidNames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "LEVEL_L;"
    "LEVEL_H;"
    "MAG;";

FAST_DATA_ZERO_INIT uint32_t targetPidLooptime;
FAST_DATA_ZERO_INIT pidAxisData_t pidData[XYZ_AXIS_COUNT];
FAST_DATA_ZERO_INIT pidRuntime_t pidRuntime;

#if defined(USE_THROTTLE_BOOST)
FAST_DATA_ZERO_INIT float throttleBoost;
pt1Filter_t throttleLpf;
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 2);

#if defined(STM32F1)
#define PID_PROCESS_DENOM_DEFAULT       8
#elif defined(STM32F3)
#define PID_PROCESS_DENOM_DEFAULT       4
#elif defined(STM32F411xE)
#define PID_PROCESS_DENOM_DEFAULT       2
#else
#define PID_PROCESS_DENOM_DEFAULT       1
#endif

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

#define LAUNCH_CONTROL_YAW_ITERM_LIMIT 50 // yaw iterm windup limit when launch mode is "FULL" (all axes)

PG_REGISTER_ARRAY_WITH_RESET_FN(pidProfile_t, PID_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 2);

void resetPidProfile(pidProfile_t *pidProfile)
{
    RESET_CONFIG(pidProfile_t, pidProfile,
        .pid = {
            [PID_ROLL] =       { 50, 70, 28, 0,  0 },
            [PID_PITCH] =      { 58, 70, 30, 0,  0 },
            [PID_YAW] =        { 60, 70, 5,  0,  30 },
            [PID_LEVEL_LOW] =  {100, 0,  10, 40, 70 },
            [PID_LEVEL_HIGH] = { 35, 0,  1,  0,  0 },
            [PID_MAG] =        { 40, 0,  0,  0,  0 },
        },
        .stickTransition = {
          { 110, 110, 130 }, // p roll, p pitch, p yaw
          { 85,  85,  75  }, // i roll, i pitch, i yaw
          { 110, 110, 130 }, // d roll, d pitch, d yaw
        },
        .pidSumLimit = PIDSUM_LIMIT,
        .pidSumLimitYaw = PIDSUM_LIMIT_YAW,
        .yaw_lowpass_hz = 0,
        .dterm_notch_hz = 0,
        .dterm_notch_cutoff = 0,
        .itermWindupPointPercent = 70,
        .pidAtMinThrottle = PID_STABILISATION_ON,
        .levelAngleLimit = 55,
        .feedforwardTransition = 0,
        .itermThrottleThreshold = 250,
        .itermAcceleratorGain = 3500,
        .crash_dthreshold = 50,     // degrees/second/second
        .crash_gthreshold = 400,    // degrees/second
        .crash_setpoint_threshold = 350, // degrees/second
        .crash_recovery = PID_CRASH_RECOVERY_DISARM, // off by default
        .angleExpo = 10,
        .horizonTransition = 0,
        .horizon_tilt_effect = 130,
        .horizon_strength = 15,
        .itermLimit = 400,
        .throttle_boost = 5,
        .throttle_boost_cutoff = 15,
        .iterm_rotation = true,
        .iterm_relax_cutoff = 11,
        .iterm_relax_cutoff_yaw = 25,
        .iterm_relax_threshold = 35,
        .iterm_relax_threshold_yaw = 35,
        .antiGravityMode = ANTI_GRAVITY_SMOOTH,
        .dterm_filter_type = FILTER_PT,
        .dterm_filter_order = 1,
        .dterm_filter2_type = FILTER_BUTTERWORTH,
        .dterm_filter2_order = 1,
        .dyn_lpf_dterm_min_hz = 65, // NOTE: this lpf is static unless dyn_lpf_dterm_width > 0
        .dyn_lpf_dterm_width = 0,
        .dyn_lpf_dterm_gain = 20,
        .dterm_lowpass2_hz = 200,   // second Dterm LPF ON by default
        .launchControlMode = LAUNCH_CONTROL_MODE_NORMAL,
        .launchControlThrottlePercent = 20,
        .launchControlAngleLimit = 0,
        .launchControlGain = 40,
        .launchControlAllowTriggerReset = true,
        .thrustLinearization = 0,
        .motor_output_limit = 100,
        .auto_profile_cell_count = AUTO_PROFILE_CELL_COUNT_STAY,
        .profileName = { 0 },
        .dyn_idle_min_rpm = 0,
        .dyn_idle_p_gain = 50,
        .dyn_idle_i_gain = 50,
        .dyn_idle_d_gain = 50,
        .dyn_idle_max_increase = 150,
        .feedforward_averaging = FEEDFORWARD_AVERAGING_OFF,
        .feedforward_max_rate_limit = 100,
        .feedforward_smooth_factor = 37,
        .feedforward_jitter_factor = 7,
        .feedforward_boost = 15,
        .dyn_lpf_curve_expo = 5,
        .vbat_sag_compensation = 0,
        .dtermMeasurementSlider = 100,
        .emuBoostPR = 15,
        .emuBoostY = 40,
        .dtermBoost = 0,
        .i_decay = 4,
        .i_decay_cutoff = 200,
        .dynThr = { 75, 125, 65 },
        .tpa_breakpoint = 1600,
        .dtermAlpha = 0,
        .dterm_abg_boost = 375,
        .dterm_abg_half_life = 50,
        .axis_lock_hz = 2,
        .axis_lock_multiplier = 0,
        .axis_smooth_multiplier = 0,
    );
}

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPidProfile(&pidProfiles[i]);
    }
}


void pidSetItermAccelerator(float newItermAccelerator)
{
    pidRuntime.itermAccelerator = newItermAccelerator;
}

bool pidOsdAntiGravityActive(void)
{
    return (pidRuntime.itermAccelerator > pidRuntime.antiGravityOsdCutoff);
}

void pidStabilisationState(pidStabilisationState_e pidControllerState)
{
    pidRuntime.pidStabilisationEnabled = (pidControllerState == PID_STABILISATION_ON) ? true : false;
}

const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

float pidGetFfBoostFactor()
{
    return pidRuntime.ffBoostFactor;
}

#ifdef USE_FEEDFORWARD
float pidGetFfSmoothFactor()
{
    return pidRuntime.ffSmoothFactor;
}

float pidGetFfJitterFactor()
{
    return pidRuntime.ffJitterFactor;
}
#endif

void pidResetIterm(void)
{
    for (int axis = 0; axis < 3; axis++) {
        pidData[axis].I = 0.0f;
    }
}

void pidUpdateAntiGravityThrottleFilter(float throttle)
{
    if (pidRuntime.antiGravityMode == ANTI_GRAVITY_SMOOTH) {
        // calculate a boost factor for P in the same way as for I when throttle changes quickly
        const float antiGravityThrottleLpf = pt1FilterApply(&pidRuntime.antiGravityThrottleLpf, throttle);
        // focus P boost on low throttle range only
        if (throttle < 0.5f) {
            pidRuntime.antiGravityPBoost = 0.5f - throttle;
        } else {
            pidRuntime.antiGravityPBoost = 0.0f;
        }
        // use lowpass to identify start of a throttle up, use this to reduce boost at start by half
        if (antiGravityThrottleLpf < throttle) {
            pidRuntime.antiGravityPBoost *= 0.5f;
        }
        // high-passed throttle focuses boost on faster throttle changes
        pidRuntime.antiGravityThrottleHpf = fabsf(throttle - antiGravityThrottleLpf);
        pidRuntime.antiGravityPBoost = pidRuntime.antiGravityPBoost * pidRuntime.antiGravityThrottleHpf;
        // smooth the P boost at 3hz to remove the jagged edges and prolong the effect after throttle stops
        pidRuntime.antiGravityPBoost = pt1FilterApply(&pidRuntime.antiGravitySmoothLpf, pidRuntime.antiGravityPBoost);
    }
}

#ifdef USE_THRUST_LINEARIZATION
float pidCompensateThrustLinearization(float throttle)
{
    if (pidRuntime.thrustLinearization != 0.0f) {
        // for whoops where a lot of TL is needed, allow more throttle boost
        const float throttleReversed = (1.0f - throttle);
        throttle /= 1.0f + pidRuntime.throttleCompensateAmount * powf(throttleReversed, 2);
    }
    return throttle;
}

float pidApplyThrustLinearization(float motorOutput)
{
    if (pidRuntime.thrustLinearization != 0.0f) {
        if (motorOutput > 0.0f) {
            const float motorOutputReversed = (1.0f - motorOutput);
            motorOutput *= 1.0f + powf(motorOutputReversed, 2) * pidRuntime.thrustLinearization;
        }
    }
    return motorOutput;
}
#endif

#if defined(USE_ACC)
// calculates strength of horizon leveling; 0 = none, 1.0 = most leveling
static float calcHorizonLevelStrength(void) {
    float horizonLevelStrength;
    // 0 at level, 90 at vertical, 180 at inverted (degrees):
    const float currentInclination = MAX(ABS(attitude.values.roll), ABS(attitude.values.pitch)) / 10.0f;
    // Used as a factor in the numerator of inclinationLevelRatio - this will cause the entry point of the fade of leveling strength to be adjustable via horizon transition in configurator for RACEMODEhorizon
    if (pidRuntime.horizonCutoffDegrees > 0 && pidRuntime.horizonTransition < pidRuntime.horizonCutoffDegrees) {
        //if horizon_tilt_effect>0 and if horizonTransition<horizon_tilt_effect
        //causes leveling to fade from horizonTransition angle to horizonCutoffDegrees  where leveling goes to zero
        const float inclinationLevelRatio = constrainf(((pidRuntime.horizonCutoffDegrees - currentInclination) * pidRuntime.racemodeHorizonTransitionFactor) / pidRuntime.horizonCutoffDegrees, 0, 1);
        // apply inclination ratio to horizonLevelStrength which lowers leveling to zero as a function of angle and regardless of stick position
        horizonLevelStrength = inclinationLevelRatio;
    }
    return constrainf(horizonLevelStrength, 0, 1);
}

// Use the FAST_CODE_NOINLINE directive to avoid this code from being inlined into ITCM RAM to avoid overflow.
// The impact is possibly slightly slower performance on F7/H7 but they have more than enough
// processing power that it should be a non-issue.
STATIC_UNIT_TESTED FAST_CODE_NOINLINE float pidLevel(int axis, const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim, float currentPidSetpoint) {
    // calculate error angle and limit the angle to the max inclination
    // rcDeflection is in range [-1.0, 1.0]
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

    f_term_low = (angle - pidRuntime.previousAngle[axis]) * pidRuntime.F_angle / pidRuntime.dT;
    pidRuntime.previousAngle[axis] = angle;

    angle = constrainf(angle, -pidProfile->levelAngleLimit, pidProfile->levelAngleLimit);
    float errorAngle = angle - ((attitude.raw[axis] - angleTrim->raw[axis]) * 0.1f);
    errorAngle = constrainf(errorAngle, -90.0f, 90.0f);
    const float errorAnglePercent = errorAngle / 90.0f;
    const float absErrorAnglePercent = fabsf(errorAnglePercent);
    const float inverseErrorAnglePercent = 1.0f - absErrorAnglePercent;
    const float angleDterm = (pidRuntime.attitudePrevious[axis] - attitude.raw[axis]) * 0.1f;
    pidRuntime.attitudePrevious[axis] = attitude.raw[axis];

    // ANGLE mode - control is angle based
    p_term_low = inverseErrorAnglePercent * errorAngle * pidRuntime.P_angle_low;
    p_term_high = absErrorAnglePercent * errorAngle * pidRuntime.P_angle_high;

    d_term_low = inverseErrorAnglePercent * angleDterm * pidRuntime.D_angle_low;
    d_term_high = absErrorAnglePercent * angleDterm * pidRuntime.D_angle_high;

    currentPidSetpoint = p_term_low + p_term_high;
    currentPidSetpoint += d_term_low + d_term_high;
    currentPidSetpoint += f_term_low;

    pidRuntime.pidCoefficient[axis].Kdf = inverseErrorAnglePercent * pidRuntime.DF_angle_low;
    pidRuntime.pidCoefficient[axis].Kdf += absErrorAnglePercent * pidRuntime.DF_angle_high;

    if (FLIGHT_MODE(HORIZON_MODE)) {
        // HORIZON mode - mix of ANGLE and ACRO modes
        // mix in errorAngle to currentPidSetpoint to add a little auto-level feel
        const float horizonLevelStrength = calcHorizonLevelStrength();
        currentPidSetpoint = ((getSetpointRate(axis) * (1 - horizonLevelStrength)) + getSetpointRate(axis)) * 0.5f + (currentPidSetpoint * horizonLevelStrength * pidRuntime.horizonStrength);
    }
    return currentPidSetpoint;
}

static void detectAndSetCrashRecovery(
    const pidCrashRecovery_e crash_recovery, const int axis,
    const float delta, const float errorRate)
{
    // if crash recovery is on and gps rescue on, then check for a crash
    if (FLIGHT_MODE(GPS_RESCUE_MODE) && crash_recovery == PID_CRASH_RECOVERY_DISARM) {
        if (ARMING_FLAG(ARMED)) {
            if (getMotorMixRange() >= 1.0f
                && fabsf(delta) > pidRuntime.crashDtermThreshold
                && fabsf(errorRate) > pidRuntime.crashGyroThreshold
                && fabsf(getSetpointRate(axis)) < pidRuntime.crashSetpointThreshold) {
                    setArmingDisabled(ARMING_DISABLED_CRASH_DETECTED);
                    disarm(DISARM_REASON_CRASH_PROTECTION);
            }
        }
    }
}
#endif // USE_ACC

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

STATIC_UNIT_TESTED void rotateItermAndAxisError()
{
    if (pidRuntime.itermRotation) {
        const float gyroToAngle = pidRuntime.dT * RAD;
        float rotationRads[XYZ_AXIS_COUNT];
        for (int i = FD_ROLL; i <= FD_YAW; i++) {
            rotationRads[i] = gyro.gyroADCf[i] * gyroToAngle;
        }
        if (pidRuntime.itermRotation) {
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

#ifdef USE_RC_SMOOTHING_FILTER
float FAST_CODE applyRcSmoothingFeedforwardFilter(int axis, float pidSetpointDelta)
{
    float ret = pidSetpointDelta;
    if (axis == pidRuntime.rcSmoothingDebugAxis) {
        DEBUG_SET(DEBUG_RC_SMOOTHING, 1, lrintf(pidSetpointDelta * 100.0f));
    }
    if (pidRuntime.feedforwardLpfInitialized) {
        ret = ptnFilterApply(&pidRuntime.feedforwardPt3[axis], pidSetpointDelta);
        if (axis == pidRuntime.rcSmoothingDebugAxis) {
            DEBUG_SET(DEBUG_RC_SMOOTHING, 2, lrintf(ret * 100.0f));
        }
    }
    return ret;
}
#endif // USE_RC_SMOOTHING_FILTER

#if defined(USE_ITERM_RELAX)
STATIC_UNIT_TESTED void applyItermRelax(const int axis, const float iterm,
    float *itermErrorRate, float currentPidSetpoint)
{
  if ((pidRuntime.itermRelaxCutoff && axis != FD_YAW) || (pidRuntime.itermRelaxCutoffYaw && axis == FD_YAW)) {
      static float itermRelaxFactor;
      const float setpointLpf = pt1FilterApply(&pidRuntime.windupLpf[axis], currentPidSetpoint);
      const float setpointHpf = fabsf(currentPidSetpoint - setpointLpf);
      if (axis != FD_YAW) {
          itermRelaxFactor = MAX(1 - setpointHpf / pidRuntime.itermRelaxThreshold, 0.0f);
      } else {
          itermRelaxFactor = MAX(1 - setpointHpf / pidRuntime.itermRelaxThresholdYaw, 0.0f);
      }
      if (SIGN(iterm) == SIGN(*itermErrorRate)) {
          *itermErrorRate *= itermRelaxFactor;
      }
      if (axis == FD_ROLL) {
          DEBUG_SET(DEBUG_ITERM_RELAX, 0, lrintf(setpointHpf));
          DEBUG_SET(DEBUG_ITERM_RELAX, 1, lrintf(itermRelaxFactor * 100.0f));
          DEBUG_SET(DEBUG_ITERM_RELAX, 2, lrintf(*itermErrorRate));
      }
  }
}
#endif


#ifdef USE_LAUNCH_CONTROL
#define LAUNCH_CONTROL_MAX_RATE 100.0f
#define LAUNCH_CONTROL_MIN_RATE 5.0f
#define LAUNCH_CONTROL_ANGLE_WINDOW 10.0f  // The remaining angle degrees where rate dampening starts

// Use the FAST_CODE_NOINLINE directive to avoid this code from being inlined into ITCM RAM to avoid overflow.
// The impact is possibly slightly slower performance on F7/H7 but they have more than enough
// processing power that it should be a non-issue.
static FAST_CODE_NOINLINE float applyLaunchControl(int axis, const rollAndPitchTrims_t *angleTrim)
{
    float ret = 0.0f;

    // Scale the rates based on stick deflection only. Fixed rates with a max of 100deg/sec
    // reached at 50% stick deflection. This keeps the launch control positioning consistent
    // regardless of the user's rates.
    if ((axis == FD_PITCH) || (pidRuntime.launchControlMode != LAUNCH_CONTROL_MODE_PITCHONLY)) {
        const float stickDeflection = constrainf(getRcDeflection(axis), -0.5f, 0.5f);
        ret = LAUNCH_CONTROL_MAX_RATE * stickDeflection * 2;
    }

#if defined(USE_ACC)
    // If ACC is enabled and a limit angle is set, then try to limit forward tilt
    // to that angle and slow down the rate as the limit is approached to reduce overshoot
    if ((axis == FD_PITCH) && (pidRuntime.launchControlAngleLimit > 0) && (ret > 0)) {
        const float currentAngle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
        if (currentAngle >= pidRuntime.launchControlAngleLimit) {
            ret = 0.0f;
        } else {
            //for the last 10 degrees scale the rate from the current input to 5 dps
            const float angleDelta = pidRuntime.launchControlAngleLimit - currentAngle;
            if (angleDelta <= LAUNCH_CONTROL_ANGLE_WINDOW) {
                ret = scaleRangef(angleDelta, 0, LAUNCH_CONTROL_ANGLE_WINDOW, LAUNCH_CONTROL_MIN_RATE, ret);
            }
        }
    }
#else
    UNUSED(angleTrim);
#endif

    return ret;
}
#endif

static FAST_CODE float emuboost(float input, float boostMultiplier, float boostLimit)
{
    float boostedRate = (input * fabsf(input)) * boostMultiplier;
    if (fabsf(input * boostLimit) < fabsf(boostedRate))
    {
        boostedRate = input * boostLimit;
    }

    input += boostedRate;
    return input;
}

static FAST_CODE float stickPositionAttenuation(int axis, int pid) {
    return 1 + (getRcDeflectionAbs(axis) * pidRuntime.stickPositionTransition[pid][axis]);
}

static FAST_CODE void stickMovement(int axis) {
    pidRuntime.filteredStickMovement[axis] = fabsf(getRcDeflection(axis) - pidRuntime.previousRcDeflection[axis]) * pidRuntime.pidFrequency;
    pidRuntime.previousRcDeflection[axis] = getRcDeflection(axis);
    pidRuntime.filteredStickMovement[axis] = pt1FilterApply(&pidRuntime.stickMovementLpf[axis], pidRuntime.filteredStickMovement[axis]);
}

static FAST_CODE void axisLockScaling(void) {
    if (pidRuntime.axisLockMultiplier != 0.0f || pidRuntime.axisSmoothMultiplier != 0.0f) {
        pidRuntime.axisLockScaler[ROLL] = constrainf(1 - (pidRuntime.filteredStickMovement[PITCH] + pidRuntime.filteredStickMovement[YAW] - pidRuntime.filteredStickMovement[ROLL]) * pidRuntime.axisLockMultiplier, 0.0f, 1.0f);
        pidRuntime.axisLockScaler[PITCH] = constrainf(1 - (pidRuntime.filteredStickMovement[ROLL] + pidRuntime.filteredStickMovement[YAW] - pidRuntime.filteredStickMovement[PITCH]) * pidRuntime.axisLockMultiplier, 0.0f, 1.0f);
        pidRuntime.axisLockScaler[YAW] = constrainf(1 - (pidRuntime.filteredStickMovement[ROLL] + pidRuntime.filteredStickMovement[PITCH] - pidRuntime.filteredStickMovement[YAW]) * pidRuntime.axisLockMultiplier, 0.0f, 1.0f);
        pidRuntime.axisLockScaler[ROLL] = constrainf(1 - pidRuntime.filteredStickMovement[ROLL] * pidRuntime.axisSmoothMultiplier, 0.0f, 1.0f);
        pidRuntime.axisLockScaler[PITCH] = constrainf(1 - pidRuntime.filteredStickMovement[PITCH] * pidRuntime.axisSmoothMultiplier, 0.0f, 1.0f);
        pidRuntime.axisLockScaler[YAW] = constrainf(1 - pidRuntime.filteredStickMovement[YAW] * pidRuntime.axisSmoothMultiplier, 0.0f, 1.0f);
      }
}

// EmuFlight pid controller, which will be maintained in the future with additional features specialised for current (mini) multirotor usage.
// Based on 2DOF reference design (matlab)
void FAST_CODE pidController(const pidProfile_t *pidProfile)
{
    static float previousGyroRate[XYZ_AXIS_COUNT];
    static float previousErrorRate[XYZ_AXIS_COUNT];

    axisLockScaling();

#if defined(USE_ACC)
    const rollAndPitchTrims_t *angleTrim = &accelerometerConfig()->accelerometerTrims;
#endif

#ifdef USE_YAW_SPIN_RECOVERY
    const bool yawSpinActive = gyroYawSpinDetected();
#endif

    const bool launchControlActive = isLaunchControlActive();

#if defined(USE_ACC)
    const bool gpsRescueIsActive = FLIGHT_MODE(GPS_RESCUE_MODE);
    levelMode_e levelMode;
    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE) || gpsRescueIsActive) {
        if (FLIGHT_MODE(NFE_RACE_MODE) && !gpsRescueIsActive) {
            levelMode = LEVEL_MODE_R;
        } else {
            levelMode = LEVEL_MODE_RP;
        }
    } else {
        levelMode = LEVEL_MODE_OFF;
    }
#endif

    // Dynamic i component,
    if ((pidRuntime.antiGravityMode == ANTI_GRAVITY_SMOOTH) && pidRuntime.antiGravityEnabled) {
        // traditional itermAccelerator factor for iTerm
        pidRuntime.itermAccelerator = pidRuntime.antiGravityThrottleHpf * 0.01f * pidRuntime.itermAcceleratorGain;
        DEBUG_SET(DEBUG_ANTI_GRAVITY, 1, lrintf(pidRuntime.itermAccelerator * 1000));
        // users AG Gain changes P boost
        pidRuntime.antiGravityPBoost *= pidRuntime.itermAcceleratorGain;
        // add some percentage of that slower, longer acting P boost factor to prolong AG effect on iTerm
        pidRuntime.itermAccelerator += pidRuntime.antiGravityPBoost * 0.05f;
        // set the final P boost amount
        pidRuntime.antiGravityPBoost *= 0.02f;
    } else {
        pidRuntime.antiGravityPBoost = 0.0f;
    }
    // Debug 1 is the multiplier P
    DEBUG_SET(DEBUG_ANTI_GRAVITY, 1, lrintf(pidRuntime.itermAccelerator * 1000));

    float agGain = pidRuntime.dT * pidRuntime.itermAccelerator * AG_KI;

    // gradually scale back integration when above windup point
    float dynCi = pidRuntime.dT;
    if (pidRuntime.itermWindupPointInv != 0.0f) {
        dynCi *= constrainf((1.0f - getMotorMixRange()) * pidRuntime.itermWindupPointInv, 0.0f, 1.0f);
    }

    rotateItermAndAxisError();

#ifdef USE_RPM_FILTER
    rpmFilterUpdate();
#endif

#ifdef USE_FEEDFORWARD
    bool newRcFrame = false;
    if (getShouldUpdateFeedforward()) {
        newRcFrame = true;
    }
#endif

    // ----------PID controller----------
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
        // Yaw control is GYRO based, direct sticks control is applied to rate PID
        // When Race Mode is active PITCH control is also GYRO based in level or horizon mode
        stickMovement(axis);
        float currentPidSetpoint = getSetpointRate(axis);
#if defined(USE_ACC)
        switch (levelMode) {
        case LEVEL_MODE_OFF:
            if (axis == FD_PITCH || axis == FD_ROLL) {
                pidRuntime.pidCoefficient[axis].Kdf = 0;
            }
            break;
        case LEVEL_MODE_R:
            if (axis == FD_PITCH) {
                pidRuntime.pidCoefficient[axis].Kdf = 0;
                break;
            }

            FALLTHROUGH;
        case LEVEL_MODE_RP:
            if (axis == FD_YAW) {
                break;
            }
            currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint);
        }
#endif

#ifdef USE_LAUNCH_CONTROL
        if (launchControlActive) {
#if defined(USE_ACC)
            currentPidSetpoint = applyLaunchControl(axis, angleTrim);
#else
            currentPidSetpoint = applyLaunchControl(axis, NULL);
#endif
        }
#endif
        // Handle yaw spin recovery - zero the setpoint on yaw to aid in recovery
        // It's not necessary to zero the set points for R/P because the PIDs will be zeroed below
#ifdef USE_YAW_SPIN_RECOVERY
        if ((axis == FD_YAW) && yawSpinActive) {
            currentPidSetpoint = 0.0f;
        }
#endif // USE_YAW_SPIN_RECOVERY

        // -----calculate error rate
        const float gyroRate = gyro.gyroADCf[axis]; // Process variable from gyro output in deg/sec
        float errorRate = currentPidSetpoint - gyroRate; // r - y

        //Emuboost
        float boostedErrorRate;
        if (axis == FD_YAW)
        {
            boostedErrorRate = emuboost(errorRate, pidRuntime.emuBoostY, pidRuntime.emuBoostLimitY);
        } else {
            boostedErrorRate = emuboost(errorRate, pidRuntime.emuBoostPR, pidRuntime.emuBoostLimitPR);
        }

        if (axis == FD_ROLL) {
            DEBUG_SET(DEBUG_BOOST, 0, lrintf(errorRate - boostedErrorRate));
        } else if (axis == FD_PITCH) {
            DEBUG_SET(DEBUG_BOOST, 1, lrintf(errorRate - boostedErrorRate));
        }

        float iterm = pidData[axis].I;
        float itermErrorRate = boostedErrorRate;

#if defined(USE_ITERM_RELAX)
        if (!launchControlActive) {
            applyItermRelax(axis, iterm, &itermErrorRate, currentPidSetpoint);
        }
#endif

        // --------low-level gyro-based PID based on 2DOF PID controller. ----------
        // 2-DOF PID controller with optional filter on derivative term.
        // b = 1 and only c (feedforward weight) can be tuned (amount derivative on measurement or error).

        // -----calculate P component
        pidData[axis].P = pidRuntime.pidCoefficient[axis].Kp * boostedErrorRate * getThrottlePAttenuation() * stickPositionAttenuation(axis, 0);
        if (axis == FD_YAW) {
            pidData[axis].P = pidRuntime.ptermYawLowpassApplyFn((filter_t *) &pidRuntime.ptermYawLowpass, pidData[axis].P);
        }

        // -----calculate I component
        float Ki;
#ifdef USE_LAUNCH_CONTROL
        // if launch control is active override the iterm gains and apply iterm windup protection to all axes
        if (launchControlActive) {
            Ki = pidRuntime.launchControlKi;
        } else
#endif
        {
            Ki = pidRuntime.pidCoefficient[axis].Ki;
        }

        float iTermNew = (Ki * dynCi + agGain) * itermErrorRate * getThrottleIAttenuation() * stickPositionAttenuation(axis, 1);

        if (SIGN(iterm) != SIGN(iTermNew))
        {
            // at low iterm iDecayMultiplier will be 1 and at high iterm it will be equivilant to iDecay
            const float iDecayMultiplier = 1.0f + (pidRuntime.iDecay - 1.0f) * constrainf(iterm / pidRuntime.iDecayCutoff, 0.0f, 1.0f);
            const float newVal = iTermNew * iDecayMultiplier;

        	  if (fabs(iterm) > fabs(newVal))
        	  {
            		iTermNew = newVal;
        	  }
        }

        pidData[axis].I = constrainf(iterm + iTermNew * pidRuntime.axisLockScaler[axis], -pidRuntime.itermLimit, pidRuntime.itermLimit);

        // -----calculate pidSetpointDelta
        float pidSetpointDelta = 0;
#ifdef USE_FEEDFORWARD
        pidSetpointDelta = feedforwardApply(axis, newRcFrame, pidRuntime.feedforwardAveraging);
#endif
        pidRuntime.previousPidSetpoint[axis] = currentPidSetpoint;

        // -----calculate D component
        // disable D if launch control is active
        if ((pidRuntime.pidCoefficient[axis].Kd > 0) && !launchControlActive) {

            // Divide rate change by dT to get differential (ie dr/dt).
            // dT is fixed and calculated from the target PID loop time
            // This is done to avoid DTerm spikes that occur with dynamically
            // calculated deltaT whenever another task causes the PID
            // loop execution to be delayed.
            float dtermFromMeasurement = -(gyro.gyroADCf[axis] - previousGyroRate[axis]);
            float dtermFromError = errorRate - previousErrorRate[axis];
            previousGyroRate[axis] = gyro.gyroADCf[axis];
            previousErrorRate[axis] = errorRate;
            float delta = ((dtermFromMeasurement * pidRuntime.dtermMeasurementSlider) + (dtermFromError * pidRuntime.dtermMeasurementSliderInverse)) * pidRuntime.pidFrequency;

            delta *= pidRuntime.pidCoefficient[axis].Kd;

            // log unfiltered roll and pitch dterm, log filtered later so we can compare without dmin/dboost
            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_D_LPF, 0, lrintf(delta));
            } else if (axis == FD_PITCH) {
                DEBUG_SET(DEBUG_D_LPF, 2, lrintf(delta));
            }

            delta = pidRuntime.dtermNotchApplyFn((filter_t *) &pidRuntime.dtermNotch[axis], delta);
            delta = pidRuntime.dtermLowpassApplyFn((filter_t *) &pidRuntime.dtermLowpass[axis], delta);
            delta = pidRuntime.dtermLowpass2ApplyFn((filter_t *) &pidRuntime.dtermLowpass2[axis], delta);

            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_D_ABG, 0, lrintf(delta));
            } else if (axis == FD_PITCH) {
                DEBUG_SET(DEBUG_D_ABG, 2, lrintf(delta));
            }
            delta = pidRuntime.dtermABGApplyFn((filter_t *) &pidRuntime.dtermABG[axis], delta);
            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_D_ABG, 1, lrintf(delta));
                DEBUG_SET(DEBUG_D_LPF, 1, lrintf(delta));
                DEBUG_SET(DEBUG_D_ABG_STATE, 0, lrintf(ABGVelocity(&pidRuntime.dtermABG[axis])));
                DEBUG_SET(DEBUG_D_ABG_STATE, 1, lrintf(ABGAcceleration(&pidRuntime.dtermABG[axis])));
                DEBUG_SET(DEBUG_D_ABG_STATE, 2, lrintf(ABGJerk(&pidRuntime.dtermABG[axis])));
                DEBUG_SET(DEBUG_D_ABG_STATE, 3, lrintf(ABGResidualError(&pidRuntime.dtermABG[axis])));
            } else if (axis == FD_PITCH) {
                DEBUG_SET(DEBUG_D_ABG, 3, lrintf(delta));
                DEBUG_SET(DEBUG_D_LPF, 3, lrintf(delta));
            }

            float preDBoost = delta;
            //dterm boost
            delta = emuboost(delta, pidRuntime.dtermBoost, pidRuntime.dtermBoostLimit);
            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_BOOST, 2, lrintf(delta - preDBoost));
            } else if (axis == FD_PITCH) {
                DEBUG_SET(DEBUG_BOOST, 3, lrintf(delta - preDBoost));
            }

            pidData[axis].D = delta * getThrottleDAttenuation() * stickPositionAttenuation(axis, 2);
        } else {
            pidData[axis].D = 0;
        }

#if defined(USE_ACC)
            detectAndSetCrashRecovery(pidProfile->crash_recovery, axis, pidData[axis].D, errorRate);
#endif
        // -----calculate feedforward component
        // Only enable feedforward for rate mode and if launch control is inactive
        const float feedforwardGain = (flightModeFlags || launchControlActive) ? 0.0f : pidRuntime.pidCoefficient[axis].Kf;
        if (feedforwardGain > 0) {
            // no transition if feedforwardTransition == 0
            float transition = pidRuntime.feedforwardTransition > 0 ? MIN(1.f, getRcDeflectionAbs(axis) * pidRuntime.feedforwardTransition) : 1;
            float feedForward = feedforwardGain * transition * pidSetpointDelta * pidRuntime.pidFrequency;

#ifdef USE_FEEDFORWARD
            pidData[axis].F = shouldApplyFeedforwardLimits(axis) ?
                applyFeedforwardLimit(axis, feedForward, pidRuntime.pidCoefficient[axis].Kp, currentPidSetpoint) : feedForward;
#else
            pidData[axis].F = feedForward;
#endif
#ifdef USE_RC_SMOOTHING_FILTER
            pidData[axis].F = applyRcSmoothingFeedforwardFilter(axis, pidData[axis].F);
#endif // USE_RC_SMOOTHING_FILTER
        } else {
            pidData[axis].F = 0;
        }

        // -----calculate true yaw feedforward component
        // feedforward as betaflight calls it is really a setpoint derivative
        // this feedforward is literally setpoint * feedforward
        // since yaw acts different this will only work for yaw
        // allowed on roll and pitch, but only in angle mode
        pidData[axis].F += currentPidSetpoint * pidRuntime.pidCoefficient[axis].Kdf;

#ifdef USE_YAW_SPIN_RECOVERY
        if (yawSpinActive) {
            pidData[axis].I = 0;  // in yaw spin always disable I
            if (axis <= FD_PITCH)  {
                // zero PIDs on pitch and roll leaving yaw P to correct spin
                pidData[axis].P = 0;
                pidData[axis].D = 0;
                pidData[axis].F = 0;
            }
        }
#endif // USE_YAW_SPIN_RECOVERY

#ifdef USE_LAUNCH_CONTROL
        // Disable P/I appropriately based on the launch control mode
        if (launchControlActive) {
            // if not using FULL mode then disable I accumulation on yaw as
            // yaw has a tendency to windup. Otherwise limit yaw iterm accumulation.
            const int launchControlYawItermLimit = (pidRuntime.launchControlMode == LAUNCH_CONTROL_MODE_FULL) ? LAUNCH_CONTROL_YAW_ITERM_LIMIT : 0;
            pidData[FD_YAW].I = constrainf(pidData[FD_YAW].I, -launchControlYawItermLimit, launchControlYawItermLimit);

            // for pitch-only mode we disable everything except pitch P/I
            if (pidRuntime.launchControlMode == LAUNCH_CONTROL_MODE_PITCHONLY) {
                pidData[FD_ROLL].P = 0;
                pidData[FD_ROLL].I = 0;
                pidData[FD_YAW].P = 0;
                // don't let I go negative (pitch backwards) as front motors are limited in the mixer
                pidData[FD_PITCH].I = MAX(0.0f, pidData[FD_PITCH].I);
            }
        }
#endif
        // calculating the PID sum

        // P boost at the end of throttle chop
        // attenuate effect if turning more than 50 deg/s, half at 100 deg/s
        float agBoostAttenuator = fabsf(currentPidSetpoint) / 50.0f;
        agBoostAttenuator = MAX(agBoostAttenuator, 1.0f);
        const float agBoost = 1.0f + (pidRuntime.antiGravityPBoost / agBoostAttenuator);
        if (axis != FD_YAW) {
            pidData[axis].P *= agBoost;
            DEBUG_SET(DEBUG_ANTI_GRAVITY, axis + 2, lrintf(agBoost * 1000));
        }

        const float pidSum = pidData[axis].P + pidData[axis].I + pidData[axis].D + pidData[axis].F;

        pidData[axis].Sum = pidSum * pidRuntime.axisLockScaler[axis];

    }

    // Disable PID control if at zero throttle or if gyro overflow detected
    // This may look very innefficient, but it is done on purpose to always show real CPU usage as in flight
    if (!pidRuntime.pidStabilisationEnabled || gyroOverflowDetected()) {
        for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
            pidData[axis].P = 0;
            pidData[axis].I = 0;
            pidData[axis].D = 0;
            pidData[axis].F = 0;

            pidData[axis].Sum = 0;
        }
    } else if (pidRuntime.zeroThrottleItermReset) {
        pidResetIterm();
    }
}

void pidSetAntiGravityState(bool newState)
{
    if (newState != pidRuntime.antiGravityEnabled) {
        // reset the accelerator on state changes
        pidRuntime.itermAccelerator = 0.0f;
    }
    pidRuntime.antiGravityEnabled = newState;
}

bool pidAntiGravityEnabled(void)
{
    return pidRuntime.antiGravityEnabled;
}

#ifdef USE_DYN_LPF
void dynLpfDTermUpdate(float cutoff[XYZ_AXIS_COUNT])
{
    if (pidRuntime.dynLpfFilter == DYN_LPF_PT) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            ptnFilterUpdate(&pidRuntime.dtermLowpass[axis].ptnFilter, cutoff[axis], pidRuntime.dT);
        }
    } else if (pidRuntime.dynLpfFilter == DYN_LPF_BUTTERWORTH) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            biquadFilterLpfCascadeUpdate(&pidRuntime.dtermLowpass[axis].butterworthFilter, cutoff[axis], targetPidLooptime);
        }
    }
}

uint16_t dynLpfDtermThrCut(float throttle) {
    unsigned int cutoffFreq;
    cutoffFreq = dynLpfCutoffFreq(throttle, pidRuntime.dynLpfMin, pidRuntime.dynLpfMax, pidRuntime.dynLpfCurveExpo);
    return cutoffFreq;
}

float dynLpfDtermCutoff(uint16_t throttle, float dynlpf2_cutoff) {
    float cutoff;
    cutoff = MIN(dynlpf2_cutoff * pidRuntime.dynLpf2Gain, pidRuntime.dynLpf2Max);
    cutoff = throttle + cutoff;
    return cutoff;
}
#endif

float dynLpfCutoffFreq(float throttle, uint16_t dynLpfMin, uint16_t dynLpfMax, uint8_t expo) {
    const float expof = expo / 10.0f;
    static float curve;
    curve = throttle * (1 - throttle) * expof + throttle;
    return (dynLpfMax - dynLpfMin) * curve + dynLpfMin;
}

void pidSetItermReset(bool enabled)
{
    pidRuntime.zeroThrottleItermReset = enabled;
}

float pidGetPreviousSetpoint(int axis)
{
    return pidRuntime.previousPidSetpoint[axis];
}

float pidGetDT()
{
    return pidRuntime.dT;
}

float pidGetPidFrequency()
{
    return pidRuntime.pidFrequency;
}
