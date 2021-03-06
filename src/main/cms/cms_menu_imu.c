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

// Menu contents for PID, RATES, RC preview, misc
// Should be part of the relevant .c file.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef USE_CMS

#include "build/version.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_imu.h"

#include "common/utils.h"

#include "drivers/pwm_output.h"

#include "config/feature.h"
#include "pg/pg.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"
#include "flight/pid.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

//
// PID
//
static uint8_t tmpPidProfileIndex;
static uint8_t pidProfileIndex;
static char pidProfileIndexString[] = " p";
static uint8_t feathered_pids;
static uint8_t i_decay;
static uint8_t i_decay_cutoff;
static uint16_t errorBoost;
static uint8_t errorBoostLimit;
static uint16_t errorBoostYaw;
static uint8_t errorBoostLimitYaw;
static uint8_t itermRelaxCutoff;
static uint8_t itermRelaxCutoffYaw;
static uint8_t itermRelaxThreshold;
static uint8_t itermRelaxThresholdYaw;
static uint8_t itermWindup;
static uint16_t dtermBoost;
static uint8_t dtermBoostLimit;
static uint8_t emuGravityGain;
static uint8_t tempPid[3][3];
static uint8_t tempPidWc[3];
static uint8_t directYawFF;
static uint8_t linear_thrust_low_output;
static uint8_t linear_thrust_high_output;
static uint8_t linear_throttle;
static mixerImplType_e mixer_impl;
static uint8_t mixer_laziness;
static uint8_t tmpRateProfileIndex;
static uint8_t rateProfileIndex;
static char rateProfileIndexString[] = " p-r";
static controlRateConfig_t rateProfile;

static const char * const cms_offOnLabels[] = {
    "OFF", "ON"
};

static const char * const cms_throttleVbatCompTypeLabels[] = {
    "OFF", "BOOST", "LIMIT", "BOTH"
};

static const char * const cms_mixerImplTypeLabels[] = {
    "LEGACY", "SMOOTH", "2PASS"
};

static long cmsx_menuImu_onEnter(void) {
    pidProfileIndex = getCurrentPidProfileIndex();
    tmpPidProfileIndex = pidProfileIndex + 1;
    rateProfileIndex = getCurrentControlRateProfileIndex();
    tmpRateProfileIndex = rateProfileIndex + 1;
    return 0;
}

static long cmsx_menuImu_onExit(const OSD_Entry *self) {
    UNUSED(self);
    changePidProfile(pidProfileIndex);
    changeControlRateProfile(rateProfileIndex);
    return 0;
}

static long cmsx_profileIndexOnChange(displayPort_t *displayPort, const void *ptr) {
    UNUSED(displayPort);
    UNUSED(ptr);
    pidProfileIndex = tmpPidProfileIndex - 1;
    changePidProfile(pidProfileIndex);
    return 0;
}

static long cmsx_rateProfileIndexOnChange(displayPort_t *displayPort, const void *ptr) {
    UNUSED(displayPort);
    UNUSED(ptr);
    rateProfileIndex = tmpRateProfileIndex - 1;
    changeControlRateProfile(rateProfileIndex);
    return 0;
}

static long cmsx_PidAdvancedRead(void) {
    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);
    feathered_pids = pidProfile->feathered_pids;
    i_decay = pidProfile->i_decay;
    i_decay_cutoff = pidProfile->i_decay_cutoff;
    errorBoost = pidProfile->errorBoost;
    errorBoostLimit = pidProfile->errorBoostLimit;
    errorBoostYaw = pidProfile->errorBoostYaw;
    errorBoostLimitYaw = pidProfile->errorBoostLimitYaw;
    dtermBoost = pidProfile->dtermBoost;
    dtermBoostLimit = pidProfile->dtermBoostLimit;
    itermRelaxCutoff = pidProfile->iterm_relax_cutoff;
    itermRelaxCutoffYaw = pidProfile->iterm_relax_cutoff_yaw;
    itermRelaxThreshold = pidProfile->iterm_relax_threshold;
    itermRelaxThresholdYaw = pidProfile->iterm_relax_threshold_yaw;
    itermWindup = pidProfile->itermWindupPointPercent;
    emuGravityGain = pidProfile->emuGravityGain;
    linear_thrust_low_output = pidProfile->linear_thrust_low_output;
    linear_thrust_high_output = pidProfile->linear_thrust_high_output;
    linear_throttle = pidProfile->linear_throttle;
    mixer_impl = pidProfile->mixer_impl;
    mixer_laziness = pidProfile->mixer_laziness;
    return 0;
}

static long cmsx_PidAdvancedOnEnter(void) {
    pidProfileIndexString[1] = '0' + tmpPidProfileIndex;
    cmsx_PidAdvancedRead();
    return 0;
}

static long cmsx_PidAdvancedWriteback(const OSD_Entry *self) {
    UNUSED(self);
    pidProfile_t *pidProfile = currentPidProfile;
    pidProfile->feathered_pids = feathered_pids;
    pidProfile->errorBoost = errorBoost;
    pidProfile->errorBoostLimit = errorBoostLimit;
    pidProfile->errorBoostYaw = errorBoostYaw;
    pidProfile->errorBoostLimitYaw = errorBoostLimitYaw;
    pidProfile->dtermBoost = dtermBoost;
    pidProfile->dtermBoostLimit = dtermBoostLimit;
    pidProfile->i_decay = i_decay;
    pidProfile->i_decay_cutoff = i_decay_cutoff;
    pidProfile->iterm_relax_cutoff = itermRelaxCutoff;
    pidProfile->iterm_relax_cutoff_yaw = itermRelaxCutoffYaw;
    pidProfile->iterm_relax_threshold = itermRelaxThreshold;
    pidProfile->iterm_relax_threshold_yaw = itermRelaxThresholdYaw;
    pidProfile->itermWindupPointPercent = itermWindup;
    pidProfile->emuGravityGain = emuGravityGain;
    pidProfile->linear_thrust_low_output = linear_thrust_low_output;
    pidProfile->linear_thrust_high_output = linear_thrust_high_output;
    pidProfile->linear_throttle = linear_throttle;
    pidProfile->mixer_impl = mixer_impl;
    pidProfile->mixer_laziness = mixer_laziness;
    pidInitConfig(currentPidProfile);
    return 0;
}

static OSD_Entry cmsx_menuPidAdvancedEntries[] = {
    { "-- ADVANCED PIDS --", OME_Label, NULL, pidProfileIndexString, 0},

    { "FEATHERED",         OME_UINT8, NULL, &(OSD_UINT8_t){ &feathered_pids,           0, 100,   1}, 0 },

    { "EMU BOOST",         OME_UINT16, NULL, &(OSD_UINT16_t){ &errorBoost,             0,  2000, 5}, 0 },
    { "BOOST LIMIT",       OME_UINT8, NULL, &(OSD_UINT8_t){ &errorBoostLimit,          0,  250,  1}, 0 },
    { "EMU BOOST YAW",     OME_UINT16, NULL, &(OSD_UINT16_t){ &errorBoostYaw,          0,  2000, 5}, 0 },
    { "BOOST LIMIT YAW",   OME_UINT8, NULL, &(OSD_UINT8_t){ &errorBoostLimitYaw,       0,  250,  1}, 0 },

    { "DTERM BOOST",       OME_UINT16, NULL, &(OSD_UINT16_t){ &dtermBoost,             0,  2000, 5}, 0 },
    { "DTERM LIMIT",       OME_UINT8, NULL, &(OSD_UINT8_t){ &dtermBoostLimit,          0,  250,  1}, 0 },

    { "I DECAY",           OME_UINT8, NULL, &(OSD_UINT8_t){ &i_decay,                  1, 10,  1 }, 0 },
    { "I DECAY CUTOFF",    OME_UINT8, NULL, &(OSD_UINT8_t){ &i_decay_cutoff,           1, 250, 1 }, 0 },
    { "I RELAX CUTOFF",    OME_UINT8, NULL, &(OSD_UINT8_t){ &itermRelaxCutoff,         10, 100, 1 }, 0 },
    { "I RELAX CUTOFF YAW", OME_UINT8, NULL, &(OSD_UINT8_t){ &itermRelaxCutoffYaw,      10, 100, 1 }, 0 },
    { "I RELAX THRESH",    OME_UINT8, NULL, &(OSD_UINT8_t){ &itermRelaxThreshold,      0, 100, 1 }, 0 },
    { "I RELAX THRESH YAW", OME_UINT8, NULL, &(OSD_UINT8_t){ &itermRelaxThresholdYaw,   0, 100, 1 }, 0 },
    { "I WINDUP",          OME_UINT8, NULL, &(OSD_UINT8_t){ &itermWindup,              0, 100, 1 }, 0 },

    { "EMU GRAVITY",       OME_UINT8, NULL, &(OSD_UINT8_t){ &emuGravityGain,         0,  255, 1}, 0 },

    { "LINEAR THRUST LOW",  OME_UINT8, NULL, &(OSD_UINT8_t) { &linear_thrust_low_output, 0, 100,  1}, 0 },
    { "LINEAR THRUST HIGH", OME_UINT8, NULL, &(OSD_UINT8_t) { &linear_thrust_high_output, 0, 100,  1}, 0 },
    { "LINEAR THROTTLE",   OME_TAB,   NULL, &(OSD_TAB_t)   { (uint8_t *) &linear_throttle, 1, cms_offOnLabels }, 0 },
    { "MIXER IMPL",        OME_TAB,   NULL, &(OSD_TAB_t)   { &mixer_impl, MIXER_IMPL_COUNT - 1, cms_mixerImplTypeLabels }, 0 },
    { "MIXER LAZINESS",    OME_TAB,   NULL, &(OSD_TAB_t)   { (uint8_t *) &mixer_laziness, 1, cms_offOnLabels }, 0 },

    { "SAVE&EXIT",         OME_OSD_Exit, cmsMenuExit,   (void *)CMS_EXIT_SAVE, 0},
    { "BACK",              OME_Back, NULL, NULL, 0 },
    { NULL,                OME_END, NULL, NULL, 0 }
};

static CMS_Menu cmsx_menuPidAdvanced = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XPIDADVANCED",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_PidAdvancedOnEnter,
    .onExit = cmsx_PidAdvancedWriteback,
    .entries = cmsx_menuPidAdvancedEntries
};

static long cmsx_PidRead(void) {
    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);
    for (uint8_t i = 0; i < 3; i++) {
        tempPid[i][0] = pidProfile->pid[i].P;
        tempPid[i][1] = pidProfile->pid[i].I;
        tempPid[i][2] = pidProfile->pid[i].D;
    }
    directYawFF = pidProfile->directFF_yaw;
    return 0;
}

static long cmsx_PidOnEnter(void) {
    pidProfileIndexString[1] = '0' + tmpPidProfileIndex;
    cmsx_PidRead();
    return 0;
}

static long cmsx_PidWriteback(const OSD_Entry *self) {
    UNUSED(self);
    pidProfile_t *pidProfile = currentPidProfile;
    for (uint8_t i = 0; i < 3; i++) {
        pidProfile->pid[i].P = tempPid[i][0];
        pidProfile->pid[i].I = tempPid[i][1];
        pidProfile->pid[i].D = tempPid[i][2];
    }
    pidProfile->directFF_yaw = directYawFF;
    pidInitConfig(currentPidProfile);
    return 0;
}

static OSD_Entry cmsx_menuPidEntries[] = {
    { "-- EMUFLIGHT PIDS --", OME_Label, NULL, pidProfileIndexString, 0},
    { "PID ADVANCED", OME_Submenu, cmsMenuChange, &cmsx_menuPidAdvanced, 0},

    { "ROLL  P", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_ROLL][0],  0, 200, 1 }, 0 },
    { "ROLL  I", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_ROLL][1],  0, 200, 1 }, 0 },
    { "ROLL  D", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_ROLL][2],  0, 200, 1 }, 0 },

    { "PITCH P", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_PITCH][0], 0, 200, 1 }, 0 },
    { "PITCH I", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_PITCH][1], 0, 200, 1 }, 0 },
    { "PITCH D", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_PITCH][2], 0, 200, 1 }, 0 },

    { "YAW   P", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_YAW][0],   0, 200, 1 }, 0 },
    { "YAW   I", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_YAW][1],   0, 200, 1 }, 0 },
    { "YAW   D", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_YAW][2],   0, 200, 1 }, 0 },
    { "YAW DIR FF", OME_UINT8, NULL, &(OSD_UINT8_t){ &directYawFF,         0, 200, 1 }, 0 },

    { "SAVE&EXIT",   OME_OSD_Exit, cmsMenuExit,   (void *)CMS_EXIT_SAVE, 0},
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu cmsx_menuPid = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XPID",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_PidOnEnter,
    .onExit = cmsx_PidWriteback,
    .entries = cmsx_menuPidEntries
};

//
// Rate & Expo
//

static long cmsx_RateProfileRead(void) {
    memcpy(&rateProfile, controlRateProfiles(rateProfileIndex), sizeof(controlRateConfig_t));
    return 0;
}

static long cmsx_RateProfileWriteback(const OSD_Entry *self) {
    UNUSED(self);
    memcpy(controlRateProfilesMutable(rateProfileIndex), &rateProfile, sizeof(controlRateConfig_t));
    return 0;
}

static long cmsx_RateProfileOnEnter(void) {
    rateProfileIndexString[1] = '0' + tmpPidProfileIndex;
    rateProfileIndexString[3] = '0' + tmpRateProfileIndex;
    cmsx_RateProfileRead();
    return 0;
}

static OSD_Entry cmsx_menuRateProfileEntries[] = {
    { "-- RATE --", OME_Label, NULL, rateProfileIndexString, 0 },

    { "RC CENTER SENS",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rateDynamics.rateSensCenter,        25, 175, 1, 10 }, 0 },
    { "RC END SENS",      OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rateDynamics.rateSensEnd,           25, 175, 1, 10 }, 0 },
    { "RC CENTER CORR",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rateDynamics.rateCorrectionCenter,  0,   95, 1, 10 }, 0 },
    { "RC END CORR",      OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rateDynamics.rateCorrectionEnd,     0,   95, 1, 10 }, 0 },
    { "RC CENTER WEIGHT", OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rateDynamics.rateWeightCenter,      0,   95, 1, 10 }, 0 },
    { "RC END WEIGHT",    OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rateDynamics.rateWeightEnd,         0,   95, 1, 10 }, 0 },

    { "RC R RATE",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcRates[FD_ROLL],     0, 255, 1, 10 }, 0 },
    { "RC P RATE",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcRates[FD_PITCH],    0, 255, 1, 10 }, 0 },
    { "RC Y RATE",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcRates[FD_YAW],      0, 255, 1, 10 }, 0 },

    { "ROLL SUPER",  OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rates[FD_ROLL],       0, 100, 1, 10 }, 0 },
    { "PITCH SUPER", OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rates[FD_PITCH],      0, 100, 1, 10 }, 0 },
    { "YAW SUPER",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rates[FD_YAW],        0, 100, 1, 10 }, 0 },

    { "RC R EXPO",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcExpo[FD_ROLL],      0, 100, 1, 10 }, 0 },
    { "RC P EXPO",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcExpo[FD_PITCH],     0, 100, 1, 10 }, 0 },
    { "RC Y EXPO",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcExpo[FD_YAW],       0, 100, 1, 10 }, 0 },

    { "THR MID",     OME_UINT8,  NULL, &(OSD_UINT8_t) { &rateProfile.thrMid8,              0,  100,  1}, 0 },
    { "THR EXPO",    OME_UINT8,  NULL, &(OSD_UINT8_t) { &rateProfile.thrExpo8,             0,  100,  1}, 0 },
    { "TPA RATE P",  OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.dynThrP,              0,  250,  1, 10}, 0 },
    { "TPA RATE I",  OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.dynThrI,              0,  250,  1, 10}, 0 },
    { "TPA RATE D",  OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.dynThrD,              0,  250,  1, 10}, 0 },
    { "TPA BREAKPOINT",   OME_UINT16, NULL, &(OSD_UINT16_t){ &rateProfile.tpa_breakpoint,  1000, 2000, 10}, 0 },

    { "VBAT COMP TYPE",  OME_TAB,   NULL, &(OSD_TAB_t)    { &rateProfile.vbat_comp_type, VBAT_COMP_TYPE_COUNT - 1, cms_throttleVbatCompTypeLabels}, 0 },
    { "VBAT COMP REF",   OME_UINT8, NULL, &(OSD_UINT8_t)  { &rateProfile.vbat_comp_ref, VBAT_CELL_VOTAGE_RANGE_MIN, VBAT_CELL_VOTAGE_RANGE_MAX,  1}, 0 },
    { "THROTTLE LIMIT %", OME_UINT8, NULL, &(OSD_UINT8_t)  { &rateProfile.throttle_limit_percent, 25,  100,  1}, 0 },

    { "SAVE&EXIT",   OME_OSD_Exit, cmsMenuExit,   (void *)CMS_EXIT_SAVE, 0},
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu cmsx_menuRateProfile = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENURATE",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_RateProfileOnEnter,
    .onExit = cmsx_RateProfileWriteback,
    .entries = cmsx_menuRateProfileEntries
};

static uint8_t  cmsx_axis_lock_hz;
static uint8_t  cmsx_axis_lock_mult;
static uint8_t  cmsx_setPointPTransition[3];
static uint8_t  cmsx_setPointITransition[3];
static uint8_t  cmsx_setPointDTransition[3];
static uint8_t  cmsx_P_angle_low;
static uint8_t  cmsx_D_angle_low;
static uint8_t  cmsx_P_angle_high;
static uint8_t  cmsx_D_angle_high;
static uint16_t cmsx_F_angle;
static uint8_t  cmsx_horizonTransition;
static uint8_t  cmsx_throttleBoost;
static uint8_t  cmsx_motorOutputLimit;
static int8_t   cmsx_autoProfileCellCount;

static long cmsx_profileOtherOnEnter(void) {
    pidProfileIndexString[1] = '0' + tmpPidProfileIndex;
    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);
    cmsx_axis_lock_hz = pidProfile->axis_lock_hz;
    cmsx_axis_lock_mult = pidProfile->axis_lock_multiplier;
    cmsx_setPointPTransition[ROLL]  =    pidProfile->setPointPTransition[ROLL];
    cmsx_setPointITransition[ROLL]  =    pidProfile->setPointITransition[ROLL];
    cmsx_setPointDTransition[ROLL]  =    pidProfile->setPointDTransition[ROLL];
    cmsx_setPointPTransition[PITCH]  =   pidProfile->setPointPTransition[PITCH];
    cmsx_setPointITransition[PITCH]  =   pidProfile->setPointITransition[PITCH];
    cmsx_setPointDTransition[PITCH]  =   pidProfile->setPointDTransition[PITCH];
    cmsx_setPointPTransition[YAW]  =     pidProfile->setPointPTransition[YAW];
    cmsx_setPointITransition[YAW]  =     pidProfile->setPointITransition[YAW];
    cmsx_setPointDTransition[YAW]  =     pidProfile->setPointDTransition[YAW];
    cmsx_P_angle_low =       pidProfile->pid[PID_LEVEL_LOW].P;
    cmsx_D_angle_low =       pidProfile->pid[PID_LEVEL_LOW].D;
    cmsx_P_angle_high =      pidProfile->pid[PID_LEVEL_HIGH].P;
    cmsx_D_angle_high =      pidProfile->pid[PID_LEVEL_HIGH].D;
    cmsx_F_angle =       pidProfile->pid[PID_LEVEL_LOW].F;
    cmsx_horizonTransition = pidProfile->horizonTransition;
    cmsx_throttleBoost = pidProfile->throttle_boost;
    cmsx_motorOutputLimit = pidProfile->motor_output_limit;
    cmsx_autoProfileCellCount = pidProfile->auto_profile_cell_count;
    return 0;
}

static long cmsx_profileOtherOnExit(const OSD_Entry *self) {
    UNUSED(self);
    pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);
    pidProfile->axis_lock_hz = cmsx_axis_lock_hz;
    pidProfile->axis_lock_multiplier = cmsx_axis_lock_mult;
    pidProfile->setPointPTransition[ROLL] = cmsx_setPointPTransition[ROLL];
    pidProfile->setPointITransition[ROLL] = cmsx_setPointITransition[ROLL];
    pidProfile->setPointDTransition[ROLL] = cmsx_setPointDTransition[ROLL];
    pidProfile->setPointPTransition[PITCH] = cmsx_setPointPTransition[PITCH];
    pidProfile->setPointITransition[PITCH] = cmsx_setPointITransition[PITCH];
    pidProfile->setPointDTransition[PITCH] = cmsx_setPointDTransition[PITCH];
    pidProfile->setPointPTransition[YAW] = cmsx_setPointPTransition[YAW];
    pidProfile->setPointITransition[YAW] = cmsx_setPointITransition[YAW];
    pidProfile->setPointDTransition[YAW] = cmsx_setPointDTransition[YAW];
    pidProfile->pid[PID_LEVEL_LOW].P = cmsx_P_angle_low;
    pidProfile->pid[PID_LEVEL_LOW].D = cmsx_D_angle_low;
    pidProfile->pid[PID_LEVEL_HIGH].P = cmsx_P_angle_high;
    pidProfile->pid[PID_LEVEL_HIGH].D = cmsx_D_angle_high;
    pidProfile->pid[PID_LEVEL_LOW].F = cmsx_F_angle;
    pidProfile->horizonTransition = cmsx_horizonTransition;
    pidProfile->throttle_boost = cmsx_throttleBoost;
    pidProfile->motor_output_limit = cmsx_motorOutputLimit;
    pidProfile->auto_profile_cell_count = cmsx_autoProfileCellCount;
    pidInitConfig(currentPidProfile);
    initEscEndpoints();
    return 0;
}

static OSD_Entry cmsx_menuProfileOtherEntries[] = {
    { "-- OTHER PP --", OME_Label, NULL, pidProfileIndexString, 0 },

    { "AXIS LOCK HZ",     OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_axis_lock_hz,                 1,    50,   1  }, 0 },
    { "AXIS LOCK MULT",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_axis_lock_mult,               0,    10,   1  }, 0 },

    { "SPA ROLL P",      OME_FLOAT,  NULL, &(OSD_FLOAT_t)  { &cmsx_setPointPTransition[ROLL], 0,    250,   1, 10 }, 0 },
    { "SPA ROLL I",      OME_FLOAT,  NULL, &(OSD_FLOAT_t)  { &cmsx_setPointITransition[ROLL], 0,    250,   1, 10 }, 0 },
    { "SPA ROLL D",      OME_FLOAT,  NULL, &(OSD_FLOAT_t)  { &cmsx_setPointDTransition[ROLL], 0,    250,   1, 10 }, 0 },
    { "SPA PITCH P",     OME_FLOAT,  NULL, &(OSD_FLOAT_t)  { &cmsx_setPointPTransition[PITCH], 0,    250,   1, 10 }, 0 },
    { "SPA PITCH I",     OME_FLOAT,  NULL, &(OSD_FLOAT_t)  { &cmsx_setPointITransition[PITCH], 0,    250,   1, 10 }, 0 },
    { "SPA PITCH D",     OME_FLOAT,  NULL, &(OSD_FLOAT_t)  { &cmsx_setPointDTransition[PITCH], 0,    250,   1, 10 }, 0 },
    { "SPA YAW P",       OME_FLOAT,  NULL, &(OSD_FLOAT_t)  { &cmsx_setPointPTransition[YAW],  0,    250,   1, 10 }, 0 },
    { "SPA YAW I",       OME_FLOAT,  NULL, &(OSD_FLOAT_t)  { &cmsx_setPointITransition[YAW],  0,    250,   1, 10 }, 0 },
    { "SPA YAW D",       OME_FLOAT,  NULL, &(OSD_FLOAT_t)  { &cmsx_setPointDTransition[YAW],  0,    250,   1, 10 }, 0 },
    { "ANGLE P LOW",     OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_P_angle_low,               0,    200,   1  }, 0 },
    { "ANGLE D LOW",     OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_D_angle_low,               0,    200,   1  }, 0 },
    { "ANGLE P HIGH",    OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_P_angle_high,              0,    200,   1  }, 0 },
    { "ANGLE D HIGH",    OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_D_angle_high,              0,    200,   1  }, 0 },
    { "ANGLE F",         OME_UINT16, NULL, &(OSD_UINT16_t) { &cmsx_F_angle,                   0,    2000,  1  }, 0 },
    { "HORZN TRS",       OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_horizonTransition,         0,    200,   1  }, 0 },
#ifdef USE_THROTTLE_BOOST
    { "THR BOOST",       OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_throttleBoost,             0,    100,   1  }, 0 },
#endif
    { "MTR OUT LIM %",   OME_UINT8, NULL, &(OSD_UINT8_t) { &cmsx_motorOutputLimit, MOTOR_OUTPUT_LIMIT_PERCENT_MIN,  MOTOR_OUTPUT_LIMIT_PERCENT_MAX,  1}, 0 },
    { "AUTO CELL CNT",   OME_INT8, NULL, &(OSD_INT8_t) { &cmsx_autoProfileCellCount, AUTO_PROFILE_CELL_COUNT_CHANGE, MAX_AUTO_DETECT_CELL_COUNT, 1}, 0 },

    { "SAVE&EXIT",   OME_OSD_Exit, cmsMenuExit,   (void *)CMS_EXIT_SAVE, 0},
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu cmsx_menuProfileOther = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XPROFOTHER",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_profileOtherOnEnter,
    .onExit = cmsx_profileOtherOnExit,
    .entries = cmsx_menuProfileOtherEntries,
};


static uint16_t gyroConfig_gyro_lowpass_hz_roll;
static uint16_t gyroConfig_gyro_lowpass_hz_pitch;
static uint16_t gyroConfig_gyro_lowpass_hz_yaw;
static uint16_t gyroConfig_gyro_lowpass2_hz_roll;
static uint16_t gyroConfig_gyro_lowpass2_hz_pitch;
static uint16_t gyroConfig_gyro_lowpass2_hz_yaw;
static uint16_t gyroConfig_gyro_soft_notch_hz_1;
static uint16_t gyroConfig_gyro_soft_notch_cutoff_1;
static uint16_t gyroConfig_gyro_soft_notch_hz_2;
static uint16_t gyroConfig_gyro_soft_notch_cutoff_2;
static uint16_t gyroConfig_gyro_matrix_q;
static uint16_t gyroConfig_gyro_matrix_min_hz;
static uint16_t gyroConfig_gyro_matrix_max_hz;
static uint16_t gyroConfig_gyro_abg_alpha;
static uint16_t gyroConfig_gyro_abg_boost;
static uint8_t gyroConfig_gyro_abg_half_life;
#ifndef USE_GYRO_IMUF9001
static uint16_t gyroConfig_imuf_roll_q;
static uint16_t gyroConfig_imuf_pitch_q;
static uint16_t gyroConfig_imuf_yaw_q;
static uint16_t gyroConfig_imuf_w;
static uint16_t gyroConfig_imuf_sharpness;
#endif

static long cmsx_menuGyro_onEnter(void) {
    gyroConfig_gyro_lowpass_hz_roll =  gyroConfig()->gyro_lowpass_hz[ROLL];
    gyroConfig_gyro_lowpass_hz_pitch =  gyroConfig()->gyro_lowpass_hz[PITCH];
    gyroConfig_gyro_lowpass_hz_yaw =  gyroConfig()->gyro_lowpass_hz[YAW];
    gyroConfig_gyro_lowpass2_hz_roll =  gyroConfig()->gyro_lowpass2_hz[ROLL];
    gyroConfig_gyro_lowpass2_hz_pitch =  gyroConfig()->gyro_lowpass2_hz[PITCH];
    gyroConfig_gyro_lowpass2_hz_yaw =  gyroConfig()->gyro_lowpass2_hz[YAW];
    gyroConfig_gyro_soft_notch_hz_1 = gyroConfig()->gyro_soft_notch_hz_1;
    gyroConfig_gyro_soft_notch_cutoff_1 = gyroConfig()->gyro_soft_notch_cutoff_1;
    gyroConfig_gyro_soft_notch_hz_2 = gyroConfig()->gyro_soft_notch_hz_2;
    gyroConfig_gyro_soft_notch_cutoff_2 = gyroConfig()->gyro_soft_notch_cutoff_2;
    gyroConfig_gyro_matrix_q = gyroConfig()->dyn_notch_q_factor;
    gyroConfig_gyro_matrix_min_hz = gyroConfig()->dyn_notch_min_hz;
    gyroConfig_gyro_matrix_max_hz = gyroConfig()->dyn_notch_max_hz;
    gyroConfig_gyro_abg_alpha = gyroConfig()->gyro_ABG_alpha;
    gyroConfig_gyro_abg_boost = gyroConfig()->gyro_ABG_boost;
    gyroConfig_gyro_abg_half_life = gyroConfig()->gyro_ABG_half_life;

#ifndef USE_GYRO_IMUF9001
    gyroConfig_imuf_roll_q = gyroConfig()->imuf_roll_q;
    gyroConfig_imuf_pitch_q = gyroConfig()->imuf_pitch_q;
    gyroConfig_imuf_yaw_q = gyroConfig()->imuf_yaw_q;
    gyroConfig_imuf_w = gyroConfig()->imuf_w;
    gyroConfig_imuf_sharpness = gyroConfig()->imuf_sharpness;
#endif
    return 0;
}

static long cmsx_menuGyro_onExit(const OSD_Entry *self) {
    UNUSED(self);
    gyroConfigMutable()->gyro_lowpass_hz[ROLL] =  gyroConfig_gyro_lowpass_hz_roll;
    gyroConfigMutable()->gyro_lowpass_hz[PITCH] =  gyroConfig_gyro_lowpass_hz_pitch;
    gyroConfigMutable()->gyro_lowpass_hz[YAW] =  gyroConfig_gyro_lowpass_hz_yaw;
    gyroConfigMutable()->gyro_lowpass2_hz[ROLL] =  gyroConfig_gyro_lowpass2_hz_roll;
    gyroConfigMutable()->gyro_lowpass2_hz[PITCH] =  gyroConfig_gyro_lowpass2_hz_pitch;
    gyroConfigMutable()->gyro_lowpass2_hz[YAW] =  gyroConfig_gyro_lowpass2_hz_yaw;
    gyroConfigMutable()->gyro_soft_notch_hz_1 = gyroConfig_gyro_soft_notch_hz_1;
    gyroConfigMutable()->gyro_soft_notch_cutoff_1 = gyroConfig_gyro_soft_notch_cutoff_1;
    gyroConfigMutable()->gyro_soft_notch_hz_2 = gyroConfig_gyro_soft_notch_hz_2;
    gyroConfigMutable()->gyro_soft_notch_cutoff_2 = gyroConfig_gyro_soft_notch_cutoff_2;
    gyroConfigMutable()->dyn_notch_q_factor = gyroConfig_gyro_matrix_q;
    gyroConfigMutable()->dyn_notch_min_hz = gyroConfig_gyro_matrix_min_hz;
    gyroConfigMutable()->dyn_notch_max_hz = gyroConfig_gyro_matrix_max_hz;
    gyroConfigMutable()->gyro_ABG_alpha = gyroConfig_gyro_abg_alpha;
    gyroConfigMutable()->gyro_ABG_boost = gyroConfig_gyro_abg_boost;
    gyroConfigMutable()->gyro_ABG_half_life = gyroConfig_gyro_abg_half_life;
#ifndef USE_GYRO_IMUF9001
    gyroConfigMutable()->imuf_roll_q = gyroConfig_imuf_roll_q;
    gyroConfigMutable()->imuf_pitch_q = gyroConfig_imuf_pitch_q;
    gyroConfigMutable()->imuf_yaw_q = gyroConfig_imuf_yaw_q;
    gyroConfigMutable()->imuf_w = gyroConfig_imuf_w;
    gyroConfigMutable()->imuf_sharpness = gyroConfig_imuf_sharpness;
#endif
    return 0;
}

static OSD_Entry cmsx_menuFilterGlobalEntries[] = {
    { "-- FILTER GLB  --", OME_Label, NULL, NULL, 0 },

    { "GYRO LPF ROLL",    OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_lowpass_hz_roll,     0, 16000, 1 }, 0 },
    { "GYRO LPF PITCH",   OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_lowpass_hz_pitch,    0, 16000, 1 }, 0 },
    { "GYRO LPF YAW",     OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_lowpass_hz_yaw,      0, 16000, 1 }, 0 },
#ifdef USE_GYRO_LPF2
    { "GYRO LPF2 ROLL",   OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_lowpass2_hz_roll,    0, 16000, 1 }, 0 },
    { "GYRO LPF2 PITCH",  OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_lowpass2_hz_pitch,   0, 16000, 1 }, 0 },
    { "GYRO LPF2 YAW",    OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_lowpass2_hz_yaw,     0, 16000, 1 }, 0 },
#endif
    { "MATRIX Q",         OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_matrix_q,            0, 1000, 1 }, 0 },
    { "MATRIX MIN HZ",    OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_matrix_min_hz,       30, 1000, 1 }, 0 },
    { "MATRIX MAX HZ",    OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_matrix_max_hz,       200, 1000, 1 }, 0 },
#ifndef USE_GYRO_IMUF9001
    { "IMUF W",           OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_imuf_w,                   0, 512,     1 }, 0 },
    { "ROLL Q",           OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_imuf_roll_q,              100, 16000, 100 }, 0 },
    { "PITCH Q",          OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_imuf_pitch_q,             100, 16000, 100 }, 0 },
    { "YAW Q",            OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_imuf_yaw_q,               100, 16000, 100 }, 0 },
    { "IMUF SHARPNESS",   OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_imuf_sharpness,           0, 16000,   5 }, 0 },

#endif
    { "GYRO NF1",         OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_soft_notch_hz_1,     0, 500, 1 }, 0 },
    { "GYRO NF1C",        OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_soft_notch_cutoff_1, 0, 500, 1 }, 0 },
    { "GYRO NF2",         OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_soft_notch_hz_2,     0, 500, 1 }, 0 },
    { "GYRO NF2C",        OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_soft_notch_cutoff_2, 0, 500, 1 }, 0 },

    { "GYRO ABG ALPHA",   OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_abg_alpha,           0, 1000, 1 }, 0 },
    { "GYRO ABG BOOST",   OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_gyro_abg_boost,           0, 2000, 1 }, 0 },
    { "GYRO ABG HL",      OME_UINT16, NULL, &(OSD_UINT8_t) { &gyroConfig_gyro_abg_half_life,       0, 250, 1 }, 0 },

    { "SAVE&EXIT",   OME_OSD_Exit, cmsMenuExit,   (void *)CMS_EXIT_SAVE, 0},
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu cmsx_menuFilterGlobal = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XFLTGLB",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuGyro_onEnter,
    .onExit = cmsx_menuGyro_onExit,
    .entries = cmsx_menuFilterGlobalEntries,
};

//
// SPRING Imuf
//

#if defined(USE_GYRO_IMUF9001)
static uint16_t gyroConfig_imuf_roll_q;
static uint16_t gyroConfig_imuf_pitch_q;
static uint16_t gyroConfig_imuf_yaw_q;
static uint16_t gyroConfig_imuf_w;
static uint16_t gyroConfig_imuf_pitch_lpf_cutoff_hz;
static uint16_t gyroConfig_imuf_roll_lpf_cutoff_hz;
static uint16_t gyroConfig_imuf_yaw_lpf_cutoff_hz;
static uint16_t gyroConfig_imuf_acc_lpf_cutoff_hz;
static uint16_t gyroConfig_imuf_sharpness;
#endif

#if defined(USE_GYRO_IMUF9001)
static long cmsx_menuImuf_onEnter(void) {
    gyroConfig_imuf_roll_q = gyroConfig()->imuf_roll_q;
    gyroConfig_imuf_pitch_q = gyroConfig()->imuf_pitch_q;
    gyroConfig_imuf_yaw_q = gyroConfig()->imuf_yaw_q;
    gyroConfig_imuf_w = gyroConfig()->imuf_w;
    gyroConfig_imuf_pitch_lpf_cutoff_hz = gyroConfig()->imuf_pitch_lpf_cutoff_hz;
    gyroConfig_imuf_roll_lpf_cutoff_hz = gyroConfig()->imuf_roll_lpf_cutoff_hz;
    gyroConfig_imuf_yaw_lpf_cutoff_hz = gyroConfig()->imuf_yaw_lpf_cutoff_hz;
    gyroConfig_imuf_acc_lpf_cutoff_hz = gyroConfig()->imuf_acc_lpf_cutoff_hz;
    gyroConfig_imuf_sharpness = gyroConfig()->imuf_sharpness;
    return 0;
}
#endif

#if defined(USE_GYRO_IMUF9001)
static long cmsx_menuImuf_onExit(const OSD_Entry *self) {
    UNUSED(self);
    gyroConfigMutable()->imuf_roll_q = gyroConfig_imuf_roll_q;
    gyroConfigMutable()->imuf_pitch_q = gyroConfig_imuf_pitch_q;
    gyroConfigMutable()->imuf_yaw_q = gyroConfig_imuf_yaw_q;
    gyroConfigMutable()->imuf_w = gyroConfig_imuf_w;
    gyroConfigMutable()->imuf_roll_lpf_cutoff_hz = gyroConfig_imuf_roll_lpf_cutoff_hz;
    gyroConfigMutable()->imuf_pitch_lpf_cutoff_hz = gyroConfig_imuf_pitch_lpf_cutoff_hz;
    gyroConfigMutable()->imuf_yaw_lpf_cutoff_hz = gyroConfig_imuf_yaw_lpf_cutoff_hz;
    gyroConfigMutable()->imuf_acc_lpf_cutoff_hz = gyroConfig_imuf_acc_lpf_cutoff_hz;
    gyroConfigMutable()->imuf_sharpness = gyroConfig_imuf_sharpness;
    return 0;
}
#endif

#if defined(USE_GYRO_IMUF9001)
static OSD_Entry cmsx_menuImufEntries[] = {
    { "-- SPRING IMU-F --", OME_Label, NULL, NULL, 0 },
    { "-- CHANGES REQUIRE REBOOT --", OME_Label, NULL, NULL, 0 },
    { "IMUF W",          OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_imuf_w,                   3, 512,     1 }, 0 },
    { "ROLL Q",          OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_imuf_roll_q,              0, 16000, 100 }, 0 },
    { "PITCH Q",         OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_imuf_pitch_q,             0, 16000, 100 }, 0 },
    { "YAW Q",           OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_imuf_yaw_q,               0, 16000, 100 }, 0 },
    { "IMUF SHARPNESS",  OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_imuf_sharpness,           0, 16000,    5 }, 0 },
    { "ROLL LPF",        OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_imuf_roll_lpf_cutoff_hz,  0, 450,     1 }, 0 },
    { "PITCH LPF",       OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_imuf_pitch_lpf_cutoff_hz, 0, 450,     1 }, 0 },
    { "YAW LPF",         OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_imuf_yaw_lpf_cutoff_hz,   0, 450,     1 }, 0 },
    { "IMUF ACC",        OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_imuf_acc_lpf_cutoff_hz,   0, 450,     1 }, 0 },

    { "SAVE&REBOOT",   OME_OSD_Exit, cmsMenuExit,   (void *)CMS_EXIT_SAVEREBOOT, 0},
    { "BACK",        OME_Back,            NULL,   NULL,             0},
    { NULL, OME_END, NULL, NULL, 0 }
};
#endif

#if defined(USE_GYRO_IMUF9001)
static CMS_Menu cmsx_menuImuf = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XIMUF",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuImuf_onEnter,
    .onExit = cmsx_menuImuf_onExit,
    .entries = cmsx_menuImufEntries,
};
#endif

static uint16_t cmsx_dterm_lowpass_hz_roll;
static uint16_t cmsx_dterm_lowpass_hz_pitch;
static uint16_t cmsx_dterm_lowpass_hz_yaw;
static uint16_t cmsx_dterm_lowpass2_hz_roll;
static uint16_t cmsx_dterm_lowpass2_hz_pitch;
static uint16_t cmsx_dterm_lowpass2_hz_yaw;
static uint8_t cmsx_smart_dterm_smoothing_roll;
static uint8_t cmsx_smart_dterm_smoothing_pitch;
static uint8_t cmsx_smart_dterm_smoothing_yaw;
static uint16_t cmsx_dterm_abg_alpha;
static uint16_t cmsx_dterm_abg_boost;
static uint8_t cmsx_dterm_abg_half_life;

static long cmsx_FilterPerProfileRead(void) {
    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);
    for (uint8_t i = 0; i < 3; i++) {
        tempPidWc[i] = pidProfile->dFilter[i].Wc;
    }
    cmsx_dterm_lowpass_hz_roll   = pidProfile->dFilter[ROLL].dLpf;
    cmsx_dterm_lowpass_hz_pitch  = pidProfile->dFilter[PITCH].dLpf;
    cmsx_dterm_lowpass_hz_yaw    = pidProfile->dFilter[YAW].dLpf;
    cmsx_dterm_lowpass2_hz_roll  = pidProfile->dFilter[ROLL].dLpf2;
    cmsx_dterm_lowpass2_hz_pitch = pidProfile->dFilter[PITCH].dLpf2;
    cmsx_dterm_lowpass2_hz_yaw   = pidProfile->dFilter[YAW].dLpf2;
    cmsx_smart_dterm_smoothing_roll   = pidProfile->dFilter[ROLL].smartSmoothing;
    cmsx_smart_dterm_smoothing_pitch  = pidProfile->dFilter[PITCH].smartSmoothing;
    cmsx_smart_dterm_smoothing_yaw    = pidProfile->dFilter[YAW].smartSmoothing;
    cmsx_dterm_abg_alpha = pidProfile->dterm_ABG_alpha;
    cmsx_dterm_abg_boost = pidProfile->dterm_ABG_boost;
    cmsx_dterm_abg_half_life = pidProfile->dterm_ABG_half_life;
    return 0;
}

static long cmsx_FilterPerProfileWriteback(const OSD_Entry *self) {
    UNUSED(self);
    pidProfile_t *pidProfile = currentPidProfile;
    for (uint8_t i = 0; i < 3; i++) {
        pidProfile->dFilter[i].Wc = tempPidWc[i];
    }
    pidProfile->dFilter[ROLL].dLpf   = cmsx_dterm_lowpass_hz_roll;
    pidProfile->dFilter[PITCH].dLpf  = cmsx_dterm_lowpass_hz_pitch;
    pidProfile->dFilter[YAW].dLpf    = cmsx_dterm_lowpass_hz_yaw;
    pidProfile->dFilter[ROLL].dLpf2  = cmsx_dterm_lowpass2_hz_roll;
    pidProfile->dFilter[PITCH].dLpf2 = cmsx_dterm_lowpass2_hz_pitch;
    pidProfile->dFilter[YAW].dLpf2   = cmsx_dterm_lowpass2_hz_yaw;
    pidProfile->dFilter[ROLL].smartSmoothing   = cmsx_smart_dterm_smoothing_roll;
    pidProfile->dFilter[PITCH].smartSmoothing  = cmsx_smart_dterm_smoothing_pitch;
    pidProfile->dFilter[YAW].smartSmoothing    = cmsx_smart_dterm_smoothing_yaw;
    pidProfile->dterm_ABG_alpha = cmsx_dterm_abg_alpha;
    pidProfile->dterm_ABG_boost = cmsx_dterm_abg_boost;
    pidProfile->dterm_ABG_half_life = cmsx_dterm_abg_half_life;
    return 0;
}

static OSD_Entry cmsx_menuFilterPerProfileEntries[] = {
    { "-- FILTER PP  --", OME_Label, NULL, NULL, 0 },

    { "DTERM LPF ROLL",  OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_dterm_lowpass_hz_roll,     0, 500, 1 }, 0 },
    { "DTERM LPF PITCH",  OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_dterm_lowpass_hz_pitch,     0, 500, 1 }, 0 },
    { "DTERM LPF YAW",  OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_dterm_lowpass_hz_yaw,     0, 500, 1 }, 0 },
    { "DTERM LPF2 ROLL", OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_dterm_lowpass2_hz_roll,    0, 500, 1 }, 0 },
    { "DTERM LPF2 PITCH", OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_dterm_lowpass2_hz_pitch,    0, 500, 1 }, 0 },
    { "DTERM LPF2 YAW", OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_dterm_lowpass2_hz_yaw,    0, 500, 1 }, 0 },

    { "SMART SMOOTHING ROLL",    OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_smart_dterm_smoothing_roll,       0, 250, 1 }, 0 },
    { "SMART SMOOTHING PITCH",    OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_smart_dterm_smoothing_pitch,       0, 250, 1 }, 0 },
    { "SMART SMOOTHING YAW",    OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_smart_dterm_smoothing_yaw,       0, 250, 1 }, 0 },
    { "ROLL WITCHCRAFT",    OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPidWc[ROLL], 0, 10, 1 }, 0 },
    { "PITCH WITCHCRAFT",   OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPidWc[PITCH], 0, 10, 1 }, 0 },
    { "YAW WITCHCRAFT",     OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPidWc[YAW], 0, 10, 1 }, 0 },

    { "DTERM ABG ALPHA",    OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_dterm_abg_alpha,       0, 1000, 1 }, 0 },
    { "DTERM ABG BOOST",    OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_dterm_abg_boost,       0, 2000, 1 }, 0 },
    { "DTERM ABG HL",       OME_UINT16, NULL, &(OSD_UINT8_t){ &cmsx_dterm_abg_half_life,   0, 250, 1 }, 0 },

    { "SAVE&EXIT",   OME_OSD_Exit, cmsMenuExit,   (void *)CMS_EXIT_SAVE, 0},
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu cmsx_menuFilterPerProfile = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XFLTPP",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_FilterPerProfileRead,
    .onExit = cmsx_FilterPerProfileWriteback,
    .entries = cmsx_menuFilterPerProfileEntries,
};

#ifdef USE_EXTENDED_CMS_MENUS

static uint8_t cmsx_dstPidProfile;
static uint8_t cmsx_dstControlRateProfile;

static const char * const cmsx_ProfileNames[] = {
    "-",
    "1",
    "2",
    "3"
};

static OSD_TAB_t cmsx_PidProfileTable = { &cmsx_dstPidProfile, 3, cmsx_ProfileNames };
static OSD_TAB_t cmsx_ControlRateProfileTable = { &cmsx_dstControlRateProfile, 3, cmsx_ProfileNames };

static long cmsx_menuCopyProfile_onEnter(void) {
    cmsx_dstPidProfile = 0;
    cmsx_dstControlRateProfile = 0;
    return 0;
}

static long cmsx_CopyPidProfile(displayPort_t *pDisplay, const void *ptr) {
    UNUSED(pDisplay);
    UNUSED(ptr);
    if (cmsx_dstPidProfile > 0) {
        pidCopyProfile(cmsx_dstPidProfile - 1, getCurrentPidProfileIndex());
    }
    return 0;
}

static long cmsx_CopyControlRateProfile(displayPort_t *pDisplay, const void *ptr) {
    UNUSED(pDisplay);
    UNUSED(ptr);
    if (cmsx_dstControlRateProfile > 0) {
        copyControlRateProfile(cmsx_dstControlRateProfile - 1, getCurrentControlRateProfileIndex());
    }
    return 0;
}

static OSD_Entry cmsx_menuCopyProfileEntries[] = {
    { "-- COPY PROFILE --", OME_Label, NULL, NULL, 0},

    { "CPY PID PROF TO",   OME_TAB,      NULL,                        &cmsx_PidProfileTable, 0 },
    { "COPY PP",           OME_Funcall,  cmsx_CopyPidProfile,         NULL, 0 },
    { "CPY RATE PROF TO",  OME_TAB,      NULL,                        &cmsx_ControlRateProfileTable, 0 },
    { "COPY RP",           OME_Funcall,  cmsx_CopyControlRateProfile, NULL, 0 },

    { "SAVE&EXIT",   OME_OSD_Exit, cmsMenuExit,   (void *)CMS_EXIT_SAVE, 0},
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

CMS_Menu cmsx_menuCopyProfile = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XCPY",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuCopyProfile_onEnter,
    .onExit = NULL,
    .entries = cmsx_menuCopyProfileEntries,
};

#endif

static OSD_Entry cmsx_menuImuEntries[] = {
    { "-- PROFILE --", OME_Label, NULL, NULL, 0},

    {"PID PROF",  OME_UINT8,   cmsx_profileIndexOnChange,     &(OSD_UINT8_t){ &tmpPidProfileIndex, 1, PID_PROFILE_COUNT, 1},    0},
    {"PID",       OME_Submenu, cmsMenuChange,                 &cmsx_menuPid,                                                 0},
    {"MISC PP",   OME_Submenu, cmsMenuChange,                 &cmsx_menuProfileOther,                                        0},
    {"FILT PP",   OME_Submenu, cmsMenuChange,                 &cmsx_menuFilterPerProfile,                                    0},

    {"RATE PROF", OME_UINT8,   cmsx_rateProfileIndexOnChange, &(OSD_UINT8_t){ &tmpRateProfileIndex, 1, CONTROL_RATE_PROFILE_COUNT, 1}, 0},
    {"RATE",      OME_Submenu, cmsMenuChange,                 &cmsx_menuRateProfile,                                         0},

    {"FILT GLB",  OME_Submenu, cmsMenuChange,                 &cmsx_menuFilterGlobal,                                        0},
#ifdef USE_EXTENDED_CMS_MENUS
#if defined(USE_GYRO_IMUF9001)
    {"IMUF",      OME_Submenu, cmsMenuChange,                 &cmsx_menuImuf,                                                0},
#endif
#ifdef USE_COPY_PROFILE_CMS_MENU
    {"COPY PROF", OME_Submenu, cmsMenuChange,                 &cmsx_menuCopyProfile,                                         0},
#endif
#endif /* USE_EXTENDED_CMS_MENUS */

    { "SAVE&EXIT",   OME_OSD_Exit, cmsMenuExit,   (void *)CMS_EXIT_SAVE, 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuImu = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XIMU",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_menuImu_onEnter,
    .onExit = cmsx_menuImu_onExit,
    .entries = cmsx_menuImuEntries,
};

#endif // CMS
