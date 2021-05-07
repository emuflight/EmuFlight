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

#pragma once

#include <stdbool.h>
#include "common/time.h"
#include "common/filter.h"
#include "common/axis.h"

#include "pg/pg.h"

#define MAX_PID_PROCESS_DENOM       16
#define PID_CONTROLLER_BETAFLIGHT   1
#define PID_MIXER_SCALING           1000.0f
#define PID_SERVO_MIXER_SCALING     0.7f
#define PIDSUM_LIMIT                1000
#define PIDSUM_LIMIT_YAW            1000
#define PIDSUM_LIMIT_MIN            100
#define PIDSUM_LIMIT_MAX            1000

// Scaling factors for Pids for better tunable range in configurator for betaflight pid controller. The scaling is based on legacy pid controller or previous float
#define PTERM_SCALE 0.032029f
#define ITERM_SCALE 0.244381f
#define DTERM_SCALE 0.000529f

// The constant scale factor to replace the Kd component of the feedforward calculation.
// This value gives the same "feel" as the previous Kd default of 26 (26 * DTERM_SCALE)
#define FEEDFORWARD_SCALE 0.013754f
#define DIRECT_FF_SCALE 0.005f

// Anti gravity I constant
#define AG_KI 21.586988f;

#define ITERM_ACCELERATOR_GAIN_OFF 0
#define ITERM_ACCELERATOR_GAIN_MAX 30000
typedef enum {
    PID_ROLL,
    PID_PITCH,
    PID_YAW,
    PID_LEVEL_LOW, //pid controller for low errorAngle
    PID_LEVEL_HIGH, //pid controller for high errorAngle
    PID_MAG,
    PID_ITEM_COUNT
} pidIndex_e;

typedef enum {
    SUPEREXPO_YAW_OFF = 0,
    SUPEREXPO_YAW_ON,
    SUPEREXPO_YAW_ALWAYS
} pidSuperExpoYaw_e;

typedef enum {
    PID_STABILISATION_OFF = 0,
    PID_STABILISATION_ON
} pidStabilisationState_e;

typedef enum {
    PID_CRASH_RECOVERY_OFF = 0,
    PID_CRASH_RECOVERY_DISARM,
} pidCrashRecovery_e;

typedef struct pidf_s {
    uint8_t P;
    uint8_t I;
    uint8_t D;
    uint16_t F;
    uint8_t DF;
} pidf_t;

typedef enum {
    ANTI_GRAVITY_SMOOTH,
    ANTI_GRAVITY_STEP
} antiGravityMode_e;

typedef enum ffInterpolationType_e {
    FF_INTERPOLATE_OFF,
    FF_INTERPOLATE_ON,
    FF_INTERPOLATE_AVG2,
    FF_INTERPOLATE_AVG3,
    FF_INTERPOLATE_AVG4
} ffInterpolationType_t;

#define MAX_PROFILE_NAME_LENGTH 8u

typedef struct pidProfile_s {
    uint16_t yaw_lowpass_hz;                // Additional yaw filter when yaw axis too noisy
    uint16_t dterm_notch_hz;                // Biquad dterm notch hz
    uint16_t dterm_notch_cutoff;            // Biquad dterm notch low cutoff

    pidf_t  pid[PID_ITEM_COUNT];

    uint8_t dterm_filter_type;              // Filter selection for dterm
    uint8_t itermWindupPointPercent;        // iterm windup threshold, percent motor saturation
    uint16_t pidSumLimit;
    uint16_t pidSumLimitYaw;
    uint8_t pidAtMinThrottle;               // Disable/Enable pids on zero throttle. Normally even without airmode P and D would be active.
    uint8_t levelAngleLimit;                // Max angle in degrees in level mode

    uint8_t angleExpo;                      // How much expo to add to angle mode
    uint8_t horizonTransition;              // horizonTransition
    uint8_t horizon_tilt_effect;           // inclination factor for Horizon mode
    uint8_t horizon_strength;               // multiplier to the angle pids to determine the strength of angle pids in horizon mode

    // EmuFlight PID controller parameters
    uint8_t  antiGravityMode;               // type of anti gravity method
    uint16_t itermThrottleThreshold;        // max allowed throttle delta before iterm accelerated in ms
    uint16_t itermAcceleratorGain;          // Iterm Accelerator Gain when itermThrottlethreshold is hit
    uint16_t crash_dthreshold;              // dterm crash value
    uint16_t crash_gthreshold;              // gyro crash value
    uint16_t crash_setpoint_threshold;      // setpoint must be below this value to detect crash, so flips and rolls are not interpreted as crashes
    uint8_t crash_recovery;                 // off, disarm, only works in gps mode
    uint8_t feedForwardTransition;          // Feed forward weight transition
    uint16_t itermLimit;
    uint16_t dterm_lowpass2_hz;             // Extra PT1 Filter on D in hz
    uint8_t throttle_boost;                 // how much should throttle be boosted during transient changes 0-100, 100 adds 10x hpf filtered throttle
    uint8_t throttle_boost_cutoff;          // Which cutoff frequency to use for throttle boost. higher cutoffs keep the boost on for shorter. Specified in hz.
    uint8_t iterm_rotation;                 // rotates iterm to translate world errors to local coordinate system
    uint8_t iterm_relax_cutoff;
    uint8_t iterm_relax_cutoff_yaw;
    uint8_t iterm_relax_threshold;          // This cutoff frequency specifies a low pass filter which predicts average response of the quad to setpoint
    uint8_t iterm_relax_threshold_yaw;      // This cutoff frequency specifies a low pass filter which predicts average response of the quad to setpoint
    uint8_t dterm_filter2_type;             // Filter selection for 2nd dterm
    uint16_t dyn_lpf_dterm_min_hz;
    uint8_t dyn_lpf_dterm_width;
    uint8_t dyn_lpf_dterm_gain;
    uint8_t launchControlMode;              // Whether launch control is limited to pitch only (launch stand or top-mount) or all axes (on battery)
    uint8_t launchControlThrottlePercent;   // Throttle percentage to trigger launch for launch control
    uint8_t launchControlAngleLimit;        // Optional launch control angle limit (requires ACC)
    uint8_t launchControlGain;              // Iterm gain used while launch control is active
    uint8_t launchControlAllowTriggerReset; // Controls trigger behavior and whether the trigger can be reset
    uint8_t thrustLinearization;            // Compensation factor for pid linearization
    uint8_t motor_output_limit;             // Upper limit of the motor output (percent)
    int8_t auto_profile_cell_count;         // Cell count for this profile to be used with if auto PID profile switching is used
    uint8_t ff_boost;                       // amount of high-pass filtered FF to add to FF, 100 means 100% added
    char profileName[MAX_PROFILE_NAME_LENGTH + 1]; // Descriptive name for profile

    uint8_t dyn_idle_min_rpm;                   // minimum motor speed enforced by the dynamic idle controller
    uint8_t dyn_idle_p_gain;                // P gain during active control of rpm
    uint8_t dyn_idle_i_gain;                // I gain during active control of rpm
    uint8_t dyn_idle_d_gain;                // D gain for corrections around rapid changes in rpm
    uint8_t dyn_idle_max_increase;          // limit on maximum possible increase in motor idle drive during active control

    uint8_t ff_interpolate_sp;              // Calculate FF from interpolated setpoint
    uint8_t ff_max_rate_limit;              // Maximum setpoint rate percentage for FF
    uint8_t ff_smooth_factor;               // Amount of smoothing for interpolated FF steps
    uint8_t ff_jitter_factor;               // Number of RC steps below which to attenuate FF
    uint8_t dyn_lpf_curve_expo;             // set the curve for dynamic dterm lowpass filter
    uint8_t vbat_sag_compensation;          // Reduce motor output by this percentage of the maximum compensation amount

    uint8_t  dtermMeasurementSlider;

    uint16_t emuBoostPR;
    uint16_t emuBoostY;
    uint16_t dtermBoost;

    uint8_t i_decay;
    uint8_t i_decay_cutoff;

    uint8_t stickTransition[3][XYZ_AXIS_COUNT];         // SPA p transition

    uint8_t dynThr[XYZ_AXIS_COUNT];
    uint16_t tpa_breakpoint;                // Breakpoint where TPA is activated

    uint16_t dtermAlpha;
    uint16_t dterm_abg_boost;
    uint16_t dterm_abg_half_life;

    uint8_t axis_lock_hz;                   // filter for the axis lock
    uint8_t axis_lock_multiplier;           // multplier for the axis lock effect
    uint8_t axis_smooth_multiplier;         // decreases pidsum on the axis you move
} pidProfile_t;

PG_DECLARE_ARRAY(pidProfile_t, PID_PROFILE_COUNT, pidProfiles);

typedef struct pidConfig_s {
    uint8_t pid_process_denom;              // Processing denominator for PID controller vs gyro sampling rate
    uint8_t runaway_takeoff_prevention;          // off, on - enables pidsum runaway disarm logic
    uint16_t runaway_takeoff_deactivate_delay;   // delay in ms for "in-flight" conditions before deactivation (successful flight)
    uint8_t runaway_takeoff_deactivate_throttle; // minimum throttle percent required during deactivation phase
} pidConfig_t;

PG_DECLARE(pidConfig_t, pidConfig);

union rollAndPitchTrims_u;
void pidController(const pidProfile_t *pidProfile);

typedef struct pidAxisData_s {
    float P;
    float I;
    float D;
    float F;

    float Sum;
} pidAxisData_t;

typedef union dtermLowpass_u {
    ptnFilter_t ptnFilter;
} dtermLowpass_t;

typedef struct pidCoefficient_s {
    float Kp;
    float Ki;
    float Kd;
    float Kf;
    float Kdf;
} pidCoefficient_t;

typedef struct pidRuntime_s {
    float dT;
    float pidFrequency;
    bool pidStabilisationEnabled;
    float previousPidSetpoint[XYZ_AXIS_COUNT];
    float previousRcDeflection[XYZ_AXIS_COUNT];
    float filteredStickMovement[XYZ_AXIS_COUNT];
    pt1Filter_t stickMovementLpf[XYZ_AXIS_COUNT];
    filterApplyFnPtr dtermNotchApplyFn;
    biquadFilter_t dtermNotch[XYZ_AXIS_COUNT];
    filterApplyFnPtr dtermLowpassApplyFn;
    ptnFilter_t dtermLowpass[XYZ_AXIS_COUNT];
    filterApplyFnPtr dtermLowpass2ApplyFn;
    ptnFilter_t dtermLowpass2[XYZ_AXIS_COUNT];
    filterApplyFnPtr dtermABGApplyFn;
    alphaBetaGammaFilter_t dtermABG[XYZ_AXIS_COUNT];
    filterApplyFnPtr ptermYawLowpassApplyFn;
    pt1Filter_t ptermYawLowpass;
    bool antiGravityEnabled;
    uint8_t antiGravityMode;
    pt1Filter_t antiGravityThrottleLpf;
    pt1Filter_t antiGravitySmoothLpf;
    float antiGravityOsdCutoff;
    float antiGravityThrottleHpf;
    float antiGravityPBoost;
    float ffBoostFactor;
    float itermAccelerator;
    uint16_t itermAcceleratorGain;
    float feedForwardTransition;
    pidCoefficient_t pidCoefficient[XYZ_AXIS_COUNT];

    float P_angle_low;
    float D_angle_low;
    float DF_angle_low;
    float P_angle_high;
    float D_angle_high;
    float DF_angle_high;
    float F_angle;
    float angle_yaw_correction;
    float horizonTransition;
    float horizonCutoffDegrees;
    float horizonStrength;
    float racemodeHorizonTransitionFactor;
    float previousAngle[XYZ_AXIS_COUNT];
    float attitudePrevious[XYZ_AXIS_COUNT];

    float itermWindupPointInv;
    float crashGyroThreshold;
    float crashDtermThreshold;
    float crashSetpointThreshold;
    float itermLimit;
    bool itermRotation;
    bool zeroThrottleItermReset;
    float dtermMeasurementSlider;
    float dtermMeasurementSliderInverse;

    float emuBoostPR;
    float emuBoostY;
    float emuBoostLimitPR;
    float emuBoostLimitY;
    float dtermBoost;
    float dtermBoostLimit;

    uint8_t iDecay;
    uint8_t iDecayCutoff;

    float stickPositionTransition[3][XYZ_AXIS_COUNT];

    uint8_t dynThr[XYZ_AXIS_COUNT];
    uint16_t tpaBreakpoint;                // Breakpoint where TPA is activated

#ifdef USE_ITERM_RELAX
    pt1Filter_t windupLpf[XYZ_AXIS_COUNT];
    uint8_t itermRelaxCutoff;
    uint8_t itermRelaxCutoffYaw;
    uint8_t itermRelaxThreshold;
    uint8_t itermRelaxThresholdYaw;
#endif

#ifdef USE_RC_SMOOTHING_FILTER
    ptnFilter_t setpointDerivativePt3[XYZ_AXIS_COUNT];
    bool setpointDerivativeLpfInitialized;
    uint8_t rcSmoothingDebugAxis;
    uint8_t rcSmoothingFilterType;
#endif // USE_RC_SMOOTHING_FILTER

#ifdef USE_DYN_LPF
    uint8_t dynLpfFilter;
    uint16_t dynLpfMin;
    uint16_t dynLpfMax;
    uint8_t dynLpfCurveExpo;
    uint16_t dynLpf2Gain;
    uint16_t dynLpf2Max;
#endif

#ifdef USE_LAUNCH_CONTROL
    uint8_t launchControlMode;
    uint8_t launchControlAngleLimit;
    float launchControlKi;
#endif

#ifdef USE_THRUST_LINEARIZATION
    float thrustLinearization;
    float throttleCompensateAmount;
#endif

#ifdef USE_INTERPOLATED_SP
    ffInterpolationType_t ffFromInterpolatedSetpoint;
    float ffSmoothFactor;
    float ffJitterFactor;
#endif

    float axisLockMultiplier;
    float axisSmoothMultiplier;
    float axisLockScaler[XYZ_AXIS_COUNT];
} pidRuntime_t;

extern pidRuntime_t pidRuntime;

extern const char pidNames[];

extern pidAxisData_t pidData[3];

extern uint32_t targetPidLooptime;

extern float throttleBoost;
extern pt1Filter_t throttleLpf;

void pidResetIterm(void);
void pidStabilisationState(pidStabilisationState_e pidControllerState);
void pidSetItermAccelerator(float newItermAccelerator);
void pidAcroTrainerInit(void);
void pidSetAcroTrainerState(bool newState);
void pidUpdateAntiGravityThrottleFilter(float throttle);
bool pidOsdAntiGravityActive(void);
bool pidOsdAntiGravityMode(void);
void pidSetAntiGravityState(bool newState);
bool pidAntiGravityEnabled(void);

#ifdef USE_THRUST_LINEARIZATION
float pidApplyThrustLinearization(float motorValue);
float pidCompensateThrustLinearization(float throttle);
#endif

#ifdef UNIT_TEST
#include "sensors/acceleration.h"
extern float axisError[XYZ_AXIS_COUNT];
void applyItermRelax(const int axis, const float iterm,
    const float gyroRate, float *itermErrorRate, float *currentPidSetpoint);
void applyAbsoluteControl(const int axis, const float gyroRate, float *currentPidSetpoint, float *itermErrorRate);
void rotateItermAndAxisError();
float pidLevel(int axis, const pidProfile_t *pidProfile,
    const rollAndPitchTrims_t *angleTrim, float currentPidSetpoint);
float calcHorizonLevelStrength(const pidProfile_t *pidProfile);
#endif
void dynLpfDTermUpdate(float cutoff[XYZ_AXIS_COUNT]);
uint16_t dynLpfDtermThrCut(float throttle);
float dynLpfDtermCutoff(uint16_t throttle, float dynlpf2_cutoff);
void pidSetItermReset(bool enabled);
float pidGetPreviousSetpoint(int axis);
float pidGetDT();
float pidGetPidFrequency();
float pidGetFfBoostFactor();
float pidGetFfSmoothFactor();
float pidGetFfJitterFactor();
float dynLpfCutoffFreq(float throttle, uint16_t dynLpfMin, uint16_t dynLpfMax, uint8_t expo);
