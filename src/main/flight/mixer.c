/*
 * This file is part of Cleanflight and Betaflight and Emuflight.
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
#include "common/maths.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"
#include "pg/pinio.h"
#include "pg/piniobox.h"

#include "interface/msp_box.h"

#include "drivers/pwm_output.h"
#include "drivers/pwm_esc_detect.h"
#include "drivers/time.h"
#include "drivers/io.h"

#include "io/motors.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "fc/fc_core.h"
#include "fc/fc_rc.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/gps_rescue.h"
#include "flight/mixer.h"
#include "flight/mixer_tricopter.h"
#include "flight/pid.h"

#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

PG_REGISTER_WITH_RESET_TEMPLATE(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);

#ifndef TARGET_DEFAULT_MIXER
#define TARGET_DEFAULT_MIXER    MIXER_QUADX
#endif
PG_RESET_TEMPLATE(mixerConfig_t, mixerConfig,
                  .mixerMode = TARGET_DEFAULT_MIXER,
                  .yaw_motors_reversed = false,
                  .crashflip_motor_percent = 0,
                  .crashflip_power_percent = 70,
                 );

PG_REGISTER_WITH_RESET_FN(motorConfig_t, motorConfig, PG_MOTOR_CONFIG, 1);

float calculatePredictiveAirModeAuthorityMultiplier();

void pgResetFn_motorConfig(motorConfig_t *motorConfig) {
#ifdef BRUSHED_MOTORS
    motorConfig->minthrottle = 1000;
    motorConfig->dev.motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
    motorConfig->dev.motorPwmProtocol = PWM_TYPE_BRUSHED;
    motorConfig->dev.useUnsyncedPwm = true;
#else
#ifdef USE_BRUSHED_ESC_AUTODETECT
    if (hardwareMotorType == MOTOR_BRUSHED) {
        motorConfig->minthrottle = 1000;
        motorConfig->dev.motorPwmRate = BRUSHED_MOTORS_PWM_RATE;
        motorConfig->dev.motorPwmProtocol = PWM_TYPE_BRUSHED;
        motorConfig->dev.useUnsyncedPwm = true;
    } else
#endif
    {
        motorConfig->minthrottle = 1070;
        motorConfig->dev.motorPwmRate = BRUSHLESS_MOTORS_PWM_RATE;
        motorConfig->dev.motorPwmProtocol = PWM_TYPE_DSHOT600;
    }
#endif
    motorConfig->maxthrottle = 2000;
    motorConfig->mincommand = 1000;
    motorConfig->digitalIdleOffsetValue = 450;
#ifdef USE_DSHOT_DMAR
    motorConfig->dev.useBurstDshot = ENABLE_DSHOT_DMAR;
#endif
    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS; motorIndex++) {
        motorConfig->dev.ioTags[motorIndex] = timerioTagGetByUsage(TIM_USE_MOTOR, motorIndex);
    }
    motorConfig->motorPoleCount = 14;   // Most brushes motors that we use are 14 poles
}

PG_REGISTER_ARRAY(motorMixer_t, MAX_SUPPORTED_MOTORS, customMotorMixer, PG_MOTOR_MIXER, 0);

#define PWM_RANGE_MID 1500

static FAST_RAM_ZERO_INIT uint8_t motorCount;
static FAST_RAM_ZERO_INIT float motorMixRange;

float FAST_RAM_ZERO_INIT motor[MAX_SUPPORTED_MOTORS];
//float FAST_RAM_ZERO_INIT previousMotor[MAX_SUPPORTED_MOTORS];
float motor_disarmed[MAX_SUPPORTED_MOTORS];

mixerMode_e currentMixerMode;
static motorMixer_t currentMixer[MAX_SUPPORTED_MOTORS];

static float minAuthorityZeroThrottle;
static float minAuthorityFullThrottle;
static float predictiveAirModeAuthorityMultiplier;
static float axisLockMultiplier;

static FAST_RAM_ZERO_INIT int throttleAngleCorrection;


static const motorMixer_t mixerQuadX[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
};
#ifndef USE_QUAD_MIXER_ONLY
static const motorMixer_t mixerTricopter[] = {
    { 1.0f,  0.0f,  1.333333f,  0.0f },     // REAR
    { 1.0f, -1.0f, -0.666667f,  0.0f },     // RIGHT
    { 1.0f,  1.0f, -0.666667f,  0.0f },     // LEFT
};

static const motorMixer_t mixerQuadP[] = {
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR
    { 1.0f, -1.0f,  0.0f,  1.0f },          // RIGHT
    { 1.0f,  1.0f,  0.0f,  1.0f },          // LEFT
    { 1.0f,  0.0f, -1.0f, -1.0f },          // FRONT
};

#if defined(USE_UNCOMMON_MIXERS)
static const motorMixer_t mixerBicopter[] = {
    { 1.0f,  1.0f,  0.0f,  0.0f },          // LEFT
    { 1.0f, -1.0f,  0.0f,  0.0f },          // RIGHT
};
#else
#define mixerBicopter NULL
#endif

static const motorMixer_t mixerY4[] = {
    { 1.0f,  0.0f,  1.0f, -1.0f },          // REAR_TOP CW
    { 1.0f, -1.0f, -1.0f,  0.0f },          // FRONT_R CCW
    { 1.0f,  0.0f,  1.0f,  1.0f },          // REAR_BOTTOM CCW
    { 1.0f,  1.0f, -1.0f,  0.0f },          // FRONT_L CW
};


#if (MAX_SUPPORTED_MOTORS >= 6)
static const motorMixer_t mixerHex6X[] = {
    { 1.0f, -0.5f,  0.866025f,  1.0f },     // REAR_R
    { 1.0f, -0.5f, -0.866025f,  1.0f },     // FRONT_R
    { 1.0f,  0.5f,  0.866025f, -1.0f },     // REAR_L
    { 1.0f,  0.5f, -0.866025f, -1.0f },     // FRONT_L
    { 1.0f, -1.0f,  0.0f,      -1.0f },     // RIGHT
    { 1.0f,  1.0f,  0.0f,       1.0f },     // LEFT
};

#if defined(USE_UNCOMMON_MIXERS)
static const motorMixer_t mixerHex6H[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },     // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },     // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },     // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },     // FRONT_L
    { 1.0f,  0.0f,  0.0f,  0.0f },     // RIGHT
    { 1.0f,  0.0f,  0.0f,  0.0f },     // LEFT
};

static const motorMixer_t mixerHex6P[] = {
    { 1.0f, -0.866025f,  0.5f,  1.0f },     // REAR_R
    { 1.0f, -0.866025f, -0.5f, -1.0f },     // FRONT_R
    { 1.0f,  0.866025f,  0.5f,  1.0f },     // REAR_L
    { 1.0f,  0.866025f, -0.5f, -1.0f },     // FRONT_L
    { 1.0f,  0.0f,      -1.0f,  1.0f },     // FRONT
    { 1.0f,  0.0f,       1.0f, -1.0f },     // REAR
};
static const motorMixer_t mixerY6[] = {
    { 1.0f,  0.0f,  1.333333f,  1.0f },     // REAR
    { 1.0f, -1.0f, -0.666667f, -1.0f },     // RIGHT
    { 1.0f,  1.0f, -0.666667f, -1.0f },     // LEFT
    { 1.0f,  0.0f,  1.333333f, -1.0f },     // UNDER_REAR
    { 1.0f, -1.0f, -0.666667f,  1.0f },     // UNDER_RIGHT
    { 1.0f,  1.0f, -0.666667f,  1.0f },     // UNDER_LEFT
};
#else
#define mixerHex6H NULL
#define mixerHex6P NULL
#define mixerY6 NULL
#endif // USE_UNCOMMON_MIXERS
#else
#define mixerHex6X NULL
#endif // MAX_SUPPORTED_MOTORS >= 6

#if defined(USE_UNCOMMON_MIXERS) && (MAX_SUPPORTED_MOTORS >= 8)
static const motorMixer_t mixerOctoX8[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
    { 1.0f, -1.0f,  1.0f,  1.0f },          // UNDER_REAR_R
    { 1.0f, -1.0f, -1.0f, -1.0f },          // UNDER_FRONT_R
    { 1.0f,  1.0f,  1.0f, -1.0f },          // UNDER_REAR_L
    { 1.0f,  1.0f, -1.0f,  1.0f },          // UNDER_FRONT_L
};

static const motorMixer_t mixerOctoFlatP[] = {
    { 1.0f,  0.707107f, -0.707107f,  1.0f },    // FRONT_L
    { 1.0f, -0.707107f, -0.707107f,  1.0f },    // FRONT_R
    { 1.0f, -0.707107f,  0.707107f,  1.0f },    // REAR_R
    { 1.0f,  0.707107f,  0.707107f,  1.0f },    // REAR_L
    { 1.0f,  0.0f, -1.0f, -1.0f },              // FRONT
    { 1.0f, -1.0f,  0.0f, -1.0f },              // RIGHT
    { 1.0f,  0.0f,  1.0f, -1.0f },              // REAR
    { 1.0f,  1.0f,  0.0f, -1.0f },              // LEFT
};

static const motorMixer_t mixerOctoFlatX[] = {
    { 1.0f,  1.0f, -0.414178f,  1.0f },      // MIDFRONT_L
    { 1.0f, -0.414178f, -1.0f,  1.0f },      // FRONT_R
    { 1.0f, -1.0f,  0.414178f,  1.0f },      // MIDREAR_R
    { 1.0f,  0.414178f,  1.0f,  1.0f },      // REAR_L
    { 1.0f,  0.414178f, -1.0f, -1.0f },      // FRONT_L
    { 1.0f, -1.0f, -0.414178f, -1.0f },      // MIDFRONT_R
    { 1.0f, -0.414178f,  1.0f, -1.0f },      // REAR_R
    { 1.0f,  1.0f,  0.414178f, -1.0f },      // MIDREAR_L
};
#else
#define mixerOctoX8 NULL
#define mixerOctoFlatP NULL
#define mixerOctoFlatX NULL
#endif

static const motorMixer_t mixerVtail4[] = {
    { 1.0f,  -0.58f,  0.58f, 1.0f },        // REAR_R
    { 1.0f,  -0.46f, -0.39f, -0.5f },       // FRONT_R
    { 1.0f,  0.58f,  0.58f, -1.0f },        // REAR_L
    { 1.0f,  0.46f, -0.39f, 0.5f },         // FRONT_L
};

static const motorMixer_t mixerAtail4[] = {
    { 1.0f, -0.58f,  0.58f, -1.0f },          // REAR_R
    { 1.0f, -0.46f, -0.39f,  0.5f },          // FRONT_R
    { 1.0f,  0.58f,  0.58f,  1.0f },          // REAR_L
    { 1.0f,  0.46f, -0.39f, -0.5f },          // FRONT_L
};

#if defined(USE_UNCOMMON_MIXERS)
static const motorMixer_t mixerDualcopter[] = {
    { 1.0f,  0.0f,  0.0f, -1.0f },          // LEFT
    { 1.0f,  0.0f,  0.0f,  1.0f },          // RIGHT
};
#else
#define mixerDualcopter NULL
#endif

static const motorMixer_t mixerSingleProp[] = {
    { 1.0f,  0.0f,  0.0f, 0.0f },
};

static const motorMixer_t mixerQuadX1234[] = {
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L
};

// Keep synced with mixerMode_e
const mixer_t mixers[] = {
    // motors, use servo, motor mixer
    { 0, false, NULL },                // entry 0
    { 3, true,  mixerTricopter },      // MIXER_TRI
    { 4, false, mixerQuadP },          // MIXER_QUADP
    { 4, false, mixerQuadX },          // MIXER_QUADX
    { 2, true,  mixerBicopter },       // MIXER_BICOPTER
    { 0, true,  NULL },                // * MIXER_GIMBAL
    { 6, false, mixerY6 },             // MIXER_Y6
    { 6, false, mixerHex6P },          // MIXER_HEX6
    { 1, true,  mixerSingleProp },     // * MIXER_FLYING_WING
    { 4, false, mixerY4 },             // MIXER_Y4
    { 6, false, mixerHex6X },          // MIXER_HEX6X
    { 8, false, mixerOctoX8 },         // MIXER_OCTOX8
    { 8, false, mixerOctoFlatP },      // MIXER_OCTOFLATP
    { 8, false, mixerOctoFlatX },      // MIXER_OCTOFLATX
    { 1, true,  mixerSingleProp },     // * MIXER_AIRPLANE
    { 1, true,  mixerSingleProp },     // * MIXER_HELI_120_CCPM
    { 0, true,  NULL },                // * MIXER_HELI_90_DEG
    { 4, false, mixerVtail4 },         // MIXER_VTAIL4
    { 6, false, mixerHex6H },          // MIXER_HEX6H
    { 0, true,  NULL },                // * MIXER_PPM_TO_SERVO
    { 2, true,  mixerDualcopter },     // MIXER_DUALCOPTER
    { 1, true,  NULL },                // MIXER_SINGLECOPTER
    { 4, false, mixerAtail4 },         // MIXER_ATAIL4
    { 0, false, NULL },                // MIXER_CUSTOM
    { 2, true,  NULL },                // MIXER_CUSTOM_AIRPLANE
    { 3, true,  NULL },                // MIXER_CUSTOM_TRI
    { 4, false, mixerQuadX1234 },
};
#endif // !USE_QUAD_MIXER_ONLY

FAST_RAM_ZERO_INIT float motorOutputHigh, motorOutputLow;

static FAST_RAM_ZERO_INIT float disarmMotorOutput, deadbandMotor3dHigh, deadbandMotor3dLow;
static FAST_RAM_ZERO_INIT float rcCommandThrottleRange;

uint8_t getMotorCount(void) {
    return motorCount;
}

float getMotorMixRange(void) {
    return motorMixRange;
}

bool areMotorsRunning(void) {
    bool motorsRunning = false;
    if (ARMING_FLAG(ARMED)) {
        motorsRunning = true;
    } else {
        for (int i = 0; i < motorCount; i++) {
            if (motor_disarmed[i] != disarmMotorOutput) {
                motorsRunning = true;
                break;
            }
        }
    }
    return motorsRunning;
}

bool mixerIsTricopter(void) {
#ifdef USE_SERVOS
    return (currentMixerMode == MIXER_TRI || currentMixerMode == MIXER_CUSTOM_TRI);
#else
    return false;
#endif
}

bool mixerIsOutputSaturated(int axis, float errorRate) {
    if (axis == FD_YAW && mixerIsTricopter()) {
        return mixerTricopterIsServoSaturated(errorRate);
    }
    return motorMixRange >= 1.0f;
}

// All PWM motor scaling is done to standard PWM range of 1000-2000 for easier tick conversion with legacy code / configurator
// DSHOT scaling is done to the actual dshot range
void initEscEndpoints(void) {
    float motorOutputLimit = 1.0f;
    if (currentPidProfile->motor_output_limit < 100) {
        motorOutputLimit = currentPidProfile->motor_output_limit / 100.0f;
    }
    // Can't use 'isMotorProtocolDshot()' here since motors haven't been initialised yet
    switch (motorConfig()->dev.motorPwmProtocol) {
#ifdef USE_DSHOT
    case PWM_TYPE_PROSHOT1000:
    case PWM_TYPE_DSHOT4800:
    case PWM_TYPE_DSHOT2400:
    case PWM_TYPE_DSHOT1200:
    case PWM_TYPE_DSHOT600:
    case PWM_TYPE_DSHOT300:
    case PWM_TYPE_DSHOT150: {
        float outputLimitOffset = (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE) * (1 - motorOutputLimit);
        disarmMotorOutput = DSHOT_CMD_MOTOR_STOP;
        if (feature(FEATURE_3D)) {
            motorOutputLow = DSHOT_MIN_THROTTLE + ((DSHOT_3D_FORWARD_MIN_THROTTLE - 1 - DSHOT_MIN_THROTTLE) / 100.0f) * CONVERT_PARAMETER_TO_PERCENT(motorConfig()->digitalIdleOffsetValue);
            motorOutputHigh = DSHOT_MAX_THROTTLE - outputLimitOffset / 2;
            deadbandMotor3dHigh = DSHOT_3D_FORWARD_MIN_THROTTLE + ((DSHOT_MAX_THROTTLE - DSHOT_3D_FORWARD_MIN_THROTTLE) / 100.0f) * CONVERT_PARAMETER_TO_PERCENT(motorConfig()->digitalIdleOffsetValue);
            deadbandMotor3dLow = DSHOT_3D_FORWARD_MIN_THROTTLE - 1 - outputLimitOffset / 2;
        } else {
            motorOutputLow = DSHOT_MIN_THROTTLE + ((DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE) / 100.0f) * CONVERT_PARAMETER_TO_PERCENT(motorConfig()->digitalIdleOffsetValue);
            motorOutputHigh = DSHOT_MAX_THROTTLE - outputLimitOffset;
        }
    }
    break;
#endif
    default:
        if (feature(FEATURE_3D)) {
            float outputLimitOffset = (flight3DConfig()->limit3d_high - flight3DConfig()->limit3d_low) * (1 - motorOutputLimit) / 2;
            disarmMotorOutput = flight3DConfig()->neutral3d;
            motorOutputLow = flight3DConfig()->limit3d_low + outputLimitOffset;
            motorOutputHigh = flight3DConfig()->limit3d_high - outputLimitOffset;
            deadbandMotor3dHigh = flight3DConfig()->deadband3d_high;
            deadbandMotor3dLow = flight3DConfig()->deadband3d_low;
        } else {
            disarmMotorOutput = motorConfig()->mincommand;
            motorOutputLow = motorConfig()->minthrottle;
            motorOutputHigh = motorConfig()->maxthrottle - ((motorConfig()->maxthrottle - motorConfig()->minthrottle) * (1 - motorOutputLimit));
        }
        break;
    }
    rcCommandThrottleRange = PWM_RANGE_MAX - PWM_RANGE_MIN;
}

// Initialize pidProfile related mixer settings
void mixerInitProfile(void)
{
    minAuthorityZeroThrottle = CONVERT_PARAMETER_TO_PERCENT(currentPidProfile->min_authority_zero_throttle);
    minAuthorityFullThrottle = CONVERT_PARAMETER_TO_PERCENT(currentPidProfile->min_authority_full_throttle);
    predictiveAirModeAuthorityMultiplier = CONVERT_PARAMETER_TO_PERCENT(currentPidProfile->predictiveAirModeMultiplier);
    axisLockMultiplier = CONVERT_PARAMETER_TO_PERCENT(currentPidProfile->axisLockMultiplier);
}

void mixerInit(mixerMode_e mixerMode) {
    currentMixerMode = mixerMode;
    initEscEndpoints();
    if (mixerIsTricopter()) {
        mixerTricopterInit();
    }
    mixerInitProfile();
}

#ifndef USE_QUAD_MIXER_ONLY

void mixerConfigureOutput(void) {
    motorCount = 0;
    if (currentMixerMode == MIXER_CUSTOM || currentMixerMode == MIXER_CUSTOM_TRI || currentMixerMode == MIXER_CUSTOM_AIRPLANE) {
        // load custom mixer into currentMixer
        for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
            // check if done
            if (customMotorMixer(i)->throttle == 0.0f) {
                break;
            }
            currentMixer[i] = *customMotorMixer(i);
            motorCount++;
        }
    } else {
        motorCount = mixers[currentMixerMode].motorCount;
        if (motorCount > MAX_SUPPORTED_MOTORS) {
            motorCount = MAX_SUPPORTED_MOTORS;
        }
        // copy motor-based mixers
        if (mixers[currentMixerMode].motor) {
            for (int i = 0; i < motorCount; i++)
                currentMixer[i] = mixers[currentMixerMode].motor[i];
        }
    }
    mixerResetDisarmedMotors();
}

void mixerLoadMix(int index, motorMixer_t *customMixers) {
    // we're 1-based
    index++;
    // clear existing
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        customMixers[i].throttle = 0.0f;
    }
    // do we have anything here to begin with?
    if (mixers[index].motor != NULL) {
        for (int i = 0; i < mixers[index].motorCount; i++) {
            customMixers[i] = mixers[index].motor[i];
        }
    }
}
#else
void mixerConfigureOutput(void) {
    motorCount = QUAD_MOTOR_COUNT;
    for (int i = 0; i < motorCount; i++) {
        currentMixer[i] = mixerQuadX[i];
    }
    mixerResetDisarmedMotors();
}
#endif // USE_QUAD_MIXER_ONLY

void mixerResetDisarmedMotors(void) {
    // set disarmed motor values
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        motor_disarmed[i] = disarmMotorOutput;
    }
}

void writeMotors(void) {
    if (pwmAreMotorsEnabled()) {
        for (int i = 0; i < motorCount; i++) {
            pwmWriteMotor(i, motor[i]);
        }
        pwmCompleteMotorUpdate(motorCount);
    }
}

static void writeAllMotors(int16_t mc) {
    // Sends commands to all motors
    for (int i = 0; i < motorCount; i++) {
        motor[i] = mc;
    }
    writeMotors();
}

void stopMotors(void) {
    writeAllMotors(disarmMotorOutput);
    delay(50); // give the timers and ESCs a chance to react.
}

void stopPwmAllMotors(void) {
    pwmShutdownPulsesForAllMotors(motorCount);
    delayMicroseconds(1500);
}

static FAST_RAM_ZERO_INIT float throttle = 0;
static FAST_RAM_ZERO_INIT float loggingThrottle = 0;
static FAST_RAM_ZERO_INIT float motorOutputMin;
static FAST_RAM_ZERO_INIT float motorRangeMin;
static FAST_RAM_ZERO_INIT float motorRangeMax;
static FAST_RAM_ZERO_INIT float motorOutputRange;
static FAST_RAM_ZERO_INIT int8_t motorOutputMixSign;

static void calculateThrottleAndCurrentMotorEndpoints(timeUs_t currentTimeUs) {
    static uint16_t rcThrottlePrevious = 0;   // Store the last throttle direction for deadband transitions
    static timeUs_t reversalTimeUs = 0; // time when motors last reversed in 3D mode
    float currentThrottleInputRange = 0;
    if (feature(FEATURE_3D)) {
        uint16_t rcCommand3dDeadBandLow;
        uint16_t rcCommand3dDeadBandHigh;
        if (!ARMING_FLAG(ARMED)) {
            rcThrottlePrevious = rxConfig()->midrc; // When disarmed set to mid_rc. It always results in positive direction after arming.
        }
        if (IS_RC_MODE_ACTIVE(BOX3D) || flight3DConfig()->switched_mode3d) {
            // The min_check range is halved because the output throttle is scaled to 500us.
            // So by using half of min_check we maintain the same low-throttle deadband
            // stick travel as normal non-3D mode.
            const int mincheckOffset = (rxConfig()->mincheck - PWM_RANGE_MIN) / 2;
            rcCommand3dDeadBandLow = rxConfig()->midrc - mincheckOffset;
            rcCommand3dDeadBandHigh = rxConfig()->midrc + mincheckOffset;
        } else {
            rcCommand3dDeadBandLow = rxConfig()->midrc - flight3DConfig()->deadband3d_throttle;
            rcCommand3dDeadBandHigh = rxConfig()->midrc + flight3DConfig()->deadband3d_throttle;
        }
        const float rcCommandThrottleRange3dLow = rcCommand3dDeadBandLow - PWM_RANGE_MIN;
        const float rcCommandThrottleRange3dHigh = PWM_RANGE_MAX - rcCommand3dDeadBandHigh;
        if (rcCommand[THROTTLE] <= rcCommand3dDeadBandLow) {
            // INVERTED
            motorRangeMin = motorOutputLow;
            motorRangeMax = deadbandMotor3dLow;
            if (isMotorProtocolDshot()) {
                motorOutputMin = motorOutputLow;
                motorOutputRange = deadbandMotor3dLow - motorOutputLow;
            } else {
                motorOutputMin = deadbandMotor3dLow;
                motorOutputRange = motorOutputLow - deadbandMotor3dLow;
            }
            if (motorOutputMixSign != -1) {
                reversalTimeUs = currentTimeUs;
            }
            motorOutputMixSign = -1;
            rcThrottlePrevious = rcCommand[THROTTLE];
            throttle = rcCommand3dDeadBandLow - rcCommand[THROTTLE];
            currentThrottleInputRange = rcCommandThrottleRange3dLow;
        } else if (rcCommand[THROTTLE] >= rcCommand3dDeadBandHigh) {
            // NORMAL
            motorRangeMin = deadbandMotor3dHigh;
            motorRangeMax = motorOutputHigh;
            motorOutputMin = deadbandMotor3dHigh;
            motorOutputRange = motorOutputHigh - deadbandMotor3dHigh;
            if (motorOutputMixSign != 1) {
                reversalTimeUs = currentTimeUs;
            }
            motorOutputMixSign = 1;
            rcThrottlePrevious = rcCommand[THROTTLE];
            throttle = rcCommand[THROTTLE] - rcCommand3dDeadBandHigh;
            currentThrottleInputRange = rcCommandThrottleRange3dHigh;
        } else if ((rcThrottlePrevious <= rcCommand3dDeadBandLow &&
                    !flight3DConfigMutable()->switched_mode3d) ||
                   isMotorsReversed()) {
            // INVERTED_TO_DEADBAND
            motorRangeMin = motorOutputLow;
            motorRangeMax = deadbandMotor3dLow;
            if (isMotorProtocolDshot()) {
                motorOutputMin = motorOutputLow;
                motorOutputRange = deadbandMotor3dLow - motorOutputLow;
            } else {
                motorOutputMin = deadbandMotor3dLow;
                motorOutputRange = motorOutputLow - deadbandMotor3dLow;
            }
            if (motorOutputMixSign != -1) {
                reversalTimeUs = currentTimeUs;
            }
            motorOutputMixSign = -1;
            throttle = 0;
            currentThrottleInputRange = rcCommandThrottleRange3dLow;
        } else {
            // NORMAL_TO_DEADBAND
            motorRangeMin = deadbandMotor3dHigh;
            motorRangeMax = motorOutputHigh;
            motorOutputMin = deadbandMotor3dHigh;
            motorOutputRange = motorOutputHigh - deadbandMotor3dHigh;
            if (motorOutputMixSign != 1) {
                reversalTimeUs = currentTimeUs;
            }
            motorOutputMixSign = 1;
            throttle = 0;
            currentThrottleInputRange = rcCommandThrottleRange3dHigh;
        }
        if (currentTimeUs - reversalTimeUs < 250000) {
            // keep ITerm zero for 250ms after motor reversal
            pidResetITerm();
        }
    } else {
        throttle = rcCommand[THROTTLE] - PWM_RANGE_MIN + throttleAngleCorrection;
        currentThrottleInputRange = rcCommandThrottleRange;
        motorRangeMin = motorOutputLow;
        motorRangeMax = motorOutputHigh;
        motorOutputMin = motorOutputLow;
        motorOutputRange = motorOutputHigh - motorOutputLow;
        if (getBoxIdState(BOXUSER4)) {
            motorOutputMixSign = -1;
        } else {
            motorOutputMixSign = 1;
        }
    }
    throttle = constrainf(throttle / currentThrottleInputRange, 0.0f, 1.0f);
}

#define CRASH_FLIP_DEADBAND 20
#define CRASH_FLIP_STICK_MINF 0.15f

static float mapThrustToMotorOutput(float thrust)
{
    float vbatCompFactor = currentControlRateProfile->vbat_comp_type != VBAT_COMP_TYPE_OFF ? calculateBatteryCompensationFactor() : 1.0f;

    float linearizedThrust = vbatCompFactor * scaleRangef(currentControlRateProfile->thrust_linearization_level, 0, 100, thrust, SIGN(thrust) * vbatCompFactor * sqrtf(ABS(thrust)));
    return motorOutputMin + linearizedThrust * motorOutputRange;
}

static void applyFlipOverAfterCrashModeToMotors(void) {
    if (ARMING_FLAG(ARMED)) {
        float stickDeflectionPitchAbs = getRcDeflectionAbs(FD_PITCH);
        float stickDeflectionRollAbs = getRcDeflectionAbs(FD_ROLL);
        float stickDeflectionYawAbs = getRcDeflectionAbs(FD_YAW);
        float signPitch = getRcDeflection(FD_PITCH) < 0 ? 1 : -1;
        float signRoll = getRcDeflection(FD_ROLL) < 0 ? 1 : -1;
        float signYaw = (getRcDeflection(FD_YAW) < 0 ? 1 : -1) * (mixerConfig()->yaw_motors_reversed ? 1 : -1);
        float stickDeflectionMax;
        float stickDeflectionLength = sqrtf(stickDeflectionPitchAbs * stickDeflectionPitchAbs + stickDeflectionRollAbs * stickDeflectionRollAbs);
        if (stickDeflectionPitchAbs > MAX(stickDeflectionRollAbs, stickDeflectionYawAbs)) {
            stickDeflectionMax = stickDeflectionPitchAbs;
            signYaw = 0;
        } else if (stickDeflectionRollAbs > stickDeflectionYawAbs) {
            stickDeflectionMax = stickDeflectionRollAbs;
            signYaw = 0;
        } else {
            stickDeflectionMax = stickDeflectionYawAbs;
            signRoll = 0;
            signPitch = 0;
        }
        float cosPhi = (stickDeflectionPitchAbs + stickDeflectionRollAbs) / (sqrtf(2.0f) * stickDeflectionLength);
        const float cosThreshold = sqrtf(3.0f) / 2.0f; // cos(PI/6.0f)
        if (cosPhi < cosThreshold) {
            // Enforce either roll or pitch exclusively, if not on diagonal
            if (stickDeflectionRollAbs > stickDeflectionPitchAbs) {
                signPitch = 0;
            } else {
                signRoll = 0;
            }
        }
        // Apply a reasonable amount of stick deadband
        const float flipStickRange = 1.0f - CRASH_FLIP_STICK_MINF;
        float flipPower = MAX(0.0f, stickDeflectionMax - CRASH_FLIP_STICK_MINF) / flipStickRange;
        for (int i = 0; i < motorCount; ++i) {
            float motorOutput =
                signPitch * currentMixer[i].pitch +
                signRoll * currentMixer[i].roll +
                signYaw * currentMixer[i].yaw;
            if (motorOutput < 0) {
                if (mixerConfig()->crashflip_motor_percent > 0) {
                    motorOutput = -motorOutput * (float)mixerConfig()->crashflip_motor_percent / 100.0f;
                } else {
                    motorOutput = disarmMotorOutput;
                }
            }
            motorOutput = MIN(1.0f, flipPower * motorOutput * (mixerConfig()->crashflip_power_percent * 1.414f) / 100.0f);
            motorOutput = motorOutputMin + motorOutput * motorOutputRange;
            // Add a little bit to the motorOutputMin so props aren't spinning when sticks are centered
            motorOutput = (motorOutput < motorOutputMin + CRASH_FLIP_DEADBAND) ? disarmMotorOutput : (motorOutput - CRASH_FLIP_DEADBAND);
            motor[i] = motorOutput;
        }
    } else {
        // Disarmed mode
        for (int i = 0; i < motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }
}

static float applyThrottleCurve(float throttle) {
    if (currentControlRateProfile->thrust_linearization_level) {
        if (currentControlRateProfile->use_throttle_linearization) {
            return throttle - CONVERT_PARAMETER_TO_PERCENT(motorConfig()->minthrottle) / 100.0f;
        } else {
            // counter compensating thrust linearization on throttle
            return throttle *
                   scaleRangef(currentControlRateProfile->thrust_linearization_level, 0, 100, 1.0f, ABS(throttle));
        }
    }
    return throttle;
}

static void applyMixToMotors(float motorMix[MAX_SUPPORTED_MOTORS]) {
    // Now add in the desired throttle, but keep in a range that doesn't clip adjusted
    // roll/pitch/yaw. This could move throttle down, but also up for those low throttle flips.
    float curvedThrottle = applyThrottleCurve(throttle);
    for (int i = 0; i < motorCount; i++) {
        float motorOutput = mapThrustToMotorOutput(
                motorOutputMixSign * motorMix[i] + curvedThrottle * currentMixer[i].throttle);
        if (mixerIsTricopter()) {
            motorOutput += mixerTricopterMotorCorrection(i);
        }
        if (failsafeIsActive()) {
            if (isMotorProtocolDshot()) {
                motorOutput = (motorOutput < motorRangeMin) ? disarmMotorOutput : motorOutput; // Prevent getting into special reserved range
            }
            motorOutput = constrainf(motorOutput, disarmMotorOutput, motorRangeMax);
        } else {
            motorOutput = constrainf(motorOutput, motorRangeMin, motorRangeMax);
        }
        // Motor stop handling
        if (feature(FEATURE_MOTOR_STOP) && ARMING_FLAG(ARMED) && !feature(FEATURE_3D) && !isAirmodeActive()
                && !FLIGHT_MODE(GPS_RESCUE_MODE)) {   // disable motor_stop while GPS Rescue is active
            if (((rcData[THROTTLE]) < rxConfig()->mincheck)) {
                motorOutput = disarmMotorOutput;
            }
        }
        motor[i] = motorOutput;
    }
// float difference;
// float looptimeAccounter;
// looptimeAccounter = gyro.targetLooptime * pidConfig()->pid_process_denom;
//     for (int motorNum = 0; motorNum < motorCount; motorNum++)
// {
//   difference = fabsf(motor[motorNum] - previousMotor[motorNum]);
//   if (difference <= (looptimeAccounter * motorOutputRange * 0.00002f))
//   {
//     motor[motorNum] = previousMotor[motorNum];
//   }
//   else
//   {
//     if (difference > (looptimeAccounter * motorOutputRange * 0.00040f))
//     {
//       if (motor[motorNum] > previousMotor[motorNum])
//       {
//         motor[motorNum] = previousMotor[motorNum] + (looptimeAccounter * motorOutputRange * 0.00040f); /* increase by max 5% every ms */
//         previousMotor[motorNum] = motor[motorNum];
//       }
//       else
//       {
//         motor[motorNum] = previousMotor[motorNum] - (looptimeAccounter * motorOutputRange * 0.00040f); /* decrease by max 5% every ms */
//         previousMotor[motorNum] = motor[motorNum];
//       }
//     }
//     else
//     {
//       previousMotor[motorNum] = motor[motorNum];
//     }
//   }
// }
    // Disarmed mode
    if (!ARMING_FLAG(ARMED)) {
        for (int i = 0; i < motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }
}

float applyThrottleLimit(float throttle) {
    if (currentControlRateProfile->throttle_limit_percent < 100) {
        const float throttleLimitFactor = currentControlRateProfile->throttle_limit_percent / 100.0f;
        switch (currentControlRateProfile->throttle_limit_type) {
        case THROTTLE_LIMIT_TYPE_SCALE:
            return throttle * throttleLimitFactor;
        case THROTTLE_LIMIT_TYPE_CLIP:
            return MIN(throttle, throttleLimitFactor);
        }
    }
    return throttle;
}

FAST_RAM_ZERO_INIT float lastRcDeflection[3], stickMovement[3];

void applyAirMode(float *motorMix, float motorMixMax) {
    float motorMixNormalizationFactor = motorMixRange > 1.0f && hardwareMotorType != MOTOR_BRUSHED ? motorMixRange : 1.0f;
    float motorMixDelta = 0.5f * motorMixRange;
    float authorityZeroThrottle, authorityFullThrottle;

    if (isAirmodeActive()) {
        authorityZeroThrottle = authorityFullThrottle = 1.0f;
    } else {
        float authorityMultiplier = predictiveAirModeAuthorityMultiplier ? calculatePredictiveAirModeAuthorityMultiplier() : 1.0f;
        authorityZeroThrottle = MAX(minAuthorityZeroThrottle * authorityMultiplier, 1.0f);
        authorityFullThrottle = MAX(minAuthorityFullThrottle * authorityMultiplier, 1.0f);
    }

    DEBUG_SET(DEBUG_AIRMODE_PERCENT, 0, lrintf(authorityZeroThrottle * 100.0f));
    DEBUG_SET(DEBUG_AIRMODE_PERCENT, 1, lrintf(authorityFullThrottle * 100.0f));

    bool useAirMode2_0 = currentControlRateProfile->use_airmode_2_0 && currentControlRateProfile->thrust_linearization_level; // 2.0 mode works (well) only with thrust linearization

    for (int i = 0; i < motorCount; ++i) {
        motorMix[i] += motorMixDelta - motorMixMax; // let's center motorMix values around the zero
        if (useAirMode2_0) {
            motorMix[i] = scaleRangef(throttle, 0.0f, 1.0f, authorityZeroThrottle * (motorMix[i] + ABS(motorMix[i])), authorityFullThrottle * (motorMix[i] - ABS(motorMix[i])));
        } else {
            motorMix[i] = scaleRangef(throttle, 0.0f, 1.0f, authorityZeroThrottle * (motorMix[i] + motorMixDelta), authorityFullThrottle * (motorMix[i] - motorMixDelta));
        }
        motorMix[i] /= motorMixNormalizationFactor;
    }
}

// Predictive AirMode actually predicts the need of greater authority based on stick movements
float calculatePredictiveAirModeAuthorityMultiplier() {
    float maxStickMovement = MAX(stickMovement[ROLL], MAX(stickMovement[PITCH], stickMovement[YAW])); // [0, 1], the max r/p/y stick movement
    float multiplier = pt1FilterApply(&predictiveAirmodeLpf, maxStickMovement) * predictiveAirModeAuthorityMultiplier;
    multiplier = MIN(multiplier, 1.0f);
    return multiplier;
}

extern float pidFrequency;

FAST_CODE_NOINLINE void mixTable(timeUs_t currentTimeUs)
{
    if (isFlipOverAfterCrashMode()) {
        applyFlipOverAfterCrashModeToMotors();
        return;
    }
    // Find min and max throttle based on conditions. Throttle has to be known before mixing
    calculateThrottleAndCurrentMotorEndpoints(currentTimeUs);
    // Calculate and Limit the PIDsum
    float scaledAxisPidRoll =
        constrainf(pidData[FD_ROLL].Sum, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;
    float scaledAxisPidPitch =
        constrainf(pidData[FD_PITCH].Sum, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;
    uint16_t yawPidSumLimit = currentPidProfile->pidSumLimitYaw;
#ifdef USE_YAW_SPIN_RECOVERY
    const bool yawSpinDetected = gyroYawSpinDetected();
    if (yawSpinDetected) {
        yawPidSumLimit = PIDSUM_LIMIT_MAX;   // Set to the maximum limit during yaw spin recovery to prevent limiting motor authority
    }
#endif // USE_YAW_SPIN_RECOVERY
    float scaledAxisPidYaw = constrainf(pidData[FD_YAW].Sum, -yawPidSumLimit, yawPidSumLimit) / PID_MIXER_SCALING;
    if (!mixerConfig()->yaw_motors_reversed) {
        scaledAxisPidYaw = -scaledAxisPidYaw;
    }

    //apply axis lock (make a function later)
    for (int i = 0; i < 3; ++i) {
        stickMovement[i] = fabsf(getRcDeflectionAbs(i) - lastRcDeflection[i]) * pidFrequency;
        lastRcDeflection[i] = getRcDeflectionAbs(i);
    }
    float rollLock = pt1FilterApply(&axisLockLpf[ROLL], stickMovement[ROLL]) * axisLockMultiplier;
    float pitchLock = pt1FilterApply(&axisLockLpf[PITCH], stickMovement[PITCH]) * axisLockMultiplier;
    float yawLock = pt1FilterApply(&axisLockLpf[YAW], stickMovement[YAW]) * axisLockMultiplier;
    scaledAxisPidRoll = scaledAxisPidRoll * constrainf(1 - pitchLock - yawLock + rollLock, 0.0f, 1.0f);
    scaledAxisPidPitch = scaledAxisPidPitch * constrainf(1 - rollLock - yawLock + pitchLock, 0.0f, 1.0f);
    scaledAxisPidYaw = scaledAxisPidYaw * constrainf(1 - rollLock - pitchLock + yawLock, 0.0f, 1.0f);

    DEBUG_SET(DEBUG_AXIS_LOCK, 0, lrintf(scaledAxisPidRoll * 1000.0f));
    DEBUG_SET(DEBUG_AXIS_LOCK, 1, lrintf(scaledAxisPidPitch * 1000.0f));
    DEBUG_SET(DEBUG_AXIS_LOCK, 2, lrintf(scaledAxisPidYaw * 1000.0f));
    DEBUG_SET(DEBUG_AXIS_LOCK, 3, lrintf(rollLock * 100.0f));
    //finish with the currently gross axis lock

    // Apply the throttle_limit_percent to scale or limit the throttle based on throttle_limit_type
    if (currentControlRateProfile->throttle_limit_type != THROTTLE_LIMIT_TYPE_OFF) {
        throttle = applyThrottleLimit(throttle);
    }
    // Find roll/pitch/yaw desired output
    float motorMix[MAX_SUPPORTED_MOTORS];
    float motorMixMax = 0, motorMixMin = 0;
    for (int i = 0; i < motorCount; i++) {
        float mix =
            scaledAxisPidRoll  * currentMixer[i].roll +
            scaledAxisPidPitch * currentMixer[i].pitch +
            scaledAxisPidYaw   * currentMixer[i].yaw;
        if (mix > motorMixMax) {
            motorMixMax = mix;
        } else if (mix < motorMixMin) {
            motorMixMin = mix;
        }
        motorMix[i] = mix;
    }
#if defined(USE_THROTTLE_BOOST)
    if (throttleBoost > 0.0f) {
        const float throttleHpf = throttle - pt1FilterApply(&throttleLpf, throttle);
        throttle = constrainf(throttle + throttleBoost * throttleHpf, 0.0f, 1.0f);
    }
#endif
#ifdef USE_GPS_RESCUE
    // If gps rescue is active then override the throttle. This prevents things
    // like throttle boost or throttle limit from negatively affecting the throttle.
    if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
        throttle = gpsRescueGetThrottle();
    }
#endif
    loggingThrottle = throttle;
    motorMixRange = motorMixMax - motorMixMin;

    applyAirMode(motorMix, motorMixMax);

    // Apply the mix to motor endpoints
    applyMixToMotors(motorMix);
}

float convertExternalToMotor(uint16_t externalValue) {
    float motorValue;
    switch ((int)isMotorProtocolDshot()) {
#ifdef USE_DSHOT
    case true:
        externalValue = constrain(externalValue, PWM_RANGE_MIN, PWM_RANGE_MAX);
        if (feature(FEATURE_3D)) {
            if (externalValue == PWM_RANGE_MID) {
                motorValue = DSHOT_DISARM_COMMAND;
            } else if (externalValue < PWM_RANGE_MID) {
                motorValue = scaleRangef(externalValue, PWM_RANGE_MIN, PWM_RANGE_MID - 1, DSHOT_3D_DEADBAND_LOW, DSHOT_MIN_THROTTLE);
            } else {
                motorValue = scaleRangef(externalValue, PWM_RANGE_MID + 1, PWM_RANGE_MAX, DSHOT_3D_DEADBAND_HIGH, DSHOT_MAX_THROTTLE);
            }
        } else {
            motorValue = (externalValue == PWM_RANGE_MIN) ? DSHOT_DISARM_COMMAND : scaleRangef(externalValue, PWM_RANGE_MIN + 1, PWM_RANGE_MAX, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
        }
        break;
    case false:
#endif
    default:
        motorValue = externalValue;
        break;
    }
    return motorValue;
}

uint16_t convertMotorToExternal(float motorValue) {
    uint16_t externalValue;
    switch ((int)isMotorProtocolDshot()) {
#ifdef USE_DSHOT
    case true:
        if (feature(FEATURE_3D)) {
            if (motorValue == DSHOT_DISARM_COMMAND || motorValue < DSHOT_MIN_THROTTLE) {
                externalValue = PWM_RANGE_MID;
            } else if (motorValue <= DSHOT_3D_DEADBAND_LOW) {
                externalValue = scaleRangef(motorValue, DSHOT_MIN_THROTTLE, DSHOT_3D_DEADBAND_LOW, PWM_RANGE_MID - 1, PWM_RANGE_MIN);
            } else {
                externalValue = scaleRangef(motorValue, DSHOT_3D_DEADBAND_HIGH, DSHOT_MAX_THROTTLE, PWM_RANGE_MID + 1, PWM_RANGE_MAX);
            }
        } else {
            externalValue = (motorValue < DSHOT_MIN_THROTTLE) ? PWM_RANGE_MIN : scaleRangef(motorValue, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE, PWM_RANGE_MIN + 1, PWM_RANGE_MAX);
        }
        break;
    case false:
#endif
    default:
        externalValue = motorValue;
        break;
    }
    return externalValue;
}

void mixerSetThrottleAngleCorrection(int correctionValue) {
    throttleAngleCorrection = correctionValue;
}

float mixerGetLoggingThrottle(void) {
    return loggingThrottle;
}
