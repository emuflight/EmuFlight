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

// Menu contents for PID, RATES, RC preview, misc
// Should be part of the relevant .c file.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#include "platform.h"

#ifdef USE_CMS

#include "build/version.h"
#include "build/build_config.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_imu.h"

#include "common/utils.h"

#include "config/feature.h"

#include "drivers/pwm_output.h"

#include "config/config.h"
#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/pid_init.h"

#include "pg/pg.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "cli/settings.h"

//
// PID
//

static uint8_t tmpPidProfileIndex;
static uint8_t pidProfileIndex;
static char pidProfileIndexString[MAX_PROFILE_NAME_LENGTH + 5];
static uint8_t tempPid[3][3];
static uint16_t tempPidF[3];
static uint8_t tempPidDF;

static uint8_t tmpRateProfileIndex;
static uint8_t rateProfileIndex;
static char rateProfileIndexString[MAX_RATE_PROFILE_NAME_LENGTH + 5];
static controlRateConfig_t rateProfile;

static const char * const osdTableThrottleLimitType[] = {
    "OFF", "SCALE", "CLIP"
};

#ifdef USE_MULTI_GYRO
static const char * const osdTableGyroToUse[] = {
    "FIRST", "SECOND", "BOTH"
};
#endif

static const char * const osdTableLpfType[] = {
    "PT", "BUTTERWORTH"
};

static void setProfileIndexString(char *profileString, int profileIndex, char *profileName)
{
    int charIndex = 0;
    profileString[charIndex++] = '1' + profileIndex;

#ifdef USE_PROFILE_NAMES
    const int profileNameLen = strlen(profileName);

    if (profileNameLen > 0) {
        profileString[charIndex++] = ' ';
        profileString[charIndex++] = '(';
        for (int i = 0; i < profileNameLen; i++) {
            profileString[charIndex++] = toupper(profileName[i]);
        }
        profileString[charIndex++] = ')';
    }
#else
    UNUSED(profileName);
#endif

    profileString[charIndex] = '\0';
}

static const void *cmsx_menuImu_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    pidProfileIndex = getCurrentPidProfileIndex();
    tmpPidProfileIndex = pidProfileIndex + 1;

    rateProfileIndex = getCurrentControlRateProfileIndex();
    tmpRateProfileIndex = rateProfileIndex + 1;

    return NULL;
}

static const void *cmsx_menuImu_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    changePidProfile(pidProfileIndex);
    changeControlRateProfile(rateProfileIndex);

    return NULL;
}

static const void *cmsx_profileIndexOnChange(displayPort_t *displayPort, const void *ptr)
{
    UNUSED(displayPort);
    UNUSED(ptr);

    pidProfileIndex = tmpPidProfileIndex - 1;
    changePidProfile(pidProfileIndex);

    return NULL;
}

static const void *cmsx_rateProfileIndexOnChange(displayPort_t *displayPort, const void *ptr)
{
    UNUSED(displayPort);
    UNUSED(ptr);

    rateProfileIndex = tmpRateProfileIndex - 1;
    changeControlRateProfile(rateProfileIndex);

    return NULL;
}

static uint8_t  cmsx_dterm_measurement_slider;
static uint16_t cmsx_emuboost_pr;
static uint16_t cmsx_emuboost_y;
static uint16_t cmsx_dboost;
static uint8_t  cmsx_i_decay;
static uint8_t  cmsx_iterm_relax_cutoff;
static uint8_t  cmsx_iterm_relax_cutoff_yaw;
static uint8_t emuBoost2;
static uint8_t emuBoost2_filter;
static uint8_t emuBoost2_cutoff;
static uint8_t emuBoost2_expo;

static const void *cmsx_PidAdvancedOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);

    cmsx_dterm_measurement_slider = pidProfile->dtermMeasurementSlider;
    cmsx_emuboost_pr =              pidProfile->emuBoostPR;
    cmsx_emuboost_y =               pidProfile->emuBoostY;
    cmsx_dboost =                   pidProfile->dtermBoost;
    cmsx_i_decay =                  pidProfile->i_decay;

#ifdef USE_ITERM_RELAX
    cmsx_iterm_relax_cutoff = pidProfile->iterm_relax_cutoff;
    cmsx_iterm_relax_cutoff_yaw = pidProfile->iterm_relax_cutoff_yaw;
#endif

    emuBoost2 = pidProfile->emuBoost2;
    emuBoost2_filter = pidProfile->emuBoost2_filter;
    emuBoost2_cutoff = pidProfile->emuBoost2_cutoff;
    emuBoost2_expo = pidProfile->emuBoost2_expo;

    return NULL;
}

static const void *cmsx_PidAdvancedWriteback(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);

    pidProfile->dtermMeasurementSlider = cmsx_dterm_measurement_slider;
    pidProfile->emuBoostPR =             cmsx_emuboost_pr;
    pidProfile->emuBoostY =              cmsx_emuboost_y;
    pidProfile->dtermBoost =             cmsx_dboost;
    pidProfile->i_decay =                cmsx_i_decay;

#ifdef USE_ITERM_RELAX
    pidProfile->iterm_relax_cutoff = cmsx_iterm_relax_cutoff;
    pidProfile->iterm_relax_cutoff_yaw = cmsx_iterm_relax_cutoff_yaw;
#endif

    pidProfile->emuBoost2 = emuBoost2;
    pidProfile->emuBoost2_filter = emuBoost2_filter;
    pidProfile->emuBoost2_cutoff = emuBoost2_cutoff;
    pidProfile->emuBoost2_expo = emuBoost2_expo;

    return NULL;
}

static const OSD_Entry cmsx_menuPidAdvancedEntries[] =
{
    { "-- PID ADVANCED --", OME_Label, NULL, pidProfileIndexString, 0},

    { "D MEASUREMENT",   OME_UINT8,  NULL, &(OSD_UINT8_t){ &cmsx_dterm_measurement_slider,  0, 100, 1 }, 0 },

    { "EMUBOOST",        OME_UINT16, NULL, &(OSD_UINT16_t){&cmsx_emuboost_pr,       0, 1000, 5 }, 0 },
    { "EMUBOOST YAW",    OME_UINT16, NULL, &(OSD_UINT16_t){&cmsx_emuboost_y,        0, 1000, 5 }, 0 },

    { "DBOOST",          OME_UINT16, NULL, &(OSD_UINT16_t){&cmsx_dboost,            0, 1000, 5 }, 0 },

    { "I DECAY",         OME_UINT8,  NULL, &(OSD_UINT8_t){ &cmsx_i_decay,           1, 10,   1 }, 0 },
#ifdef USE_ITERM_RELAX
    { "I RELAX CUTOFF",    OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_iterm_relax_cutoff,      10, 100, 1 }, 0 },
    { "I RELAX CUTOFF YAW", OME_UINT8, NULL, &(OSD_UINT8_t){ &cmsx_iterm_relax_cutoff_yaw,  10, 100, 1 }, 0 },
#endif
    { "EMUBOOST 2.0",      OME_UINT8, NULL, &(OSD_UINT8_t){ &emuBoost2,             0,  250,  1}, 0 },
    { "EMUBOOST 2.0 FILT", OME_UINT8, NULL, &(OSD_UINT8_t){ &emuBoost2_filter,      1,  250,  1}, 0 },
    { "EMUBOOST 2.0 CUT",  OME_UINT8, NULL, &(OSD_UINT8_t){ &emuBoost2_cutoff,      5,  250,  1}, 0 },
    { "EMUBOOST 2.0 EXPO", OME_UINT8, NULL, &(OSD_UINT8_t){ &emuBoost2_expo,       10,  100,  1}, 0 },

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu cmsx_menuPidAdvanced = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XPIDADV",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_PidAdvancedOnEnter,
    .onExit = cmsx_PidAdvancedWriteback,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuPidAdvancedEntries
};

static const void *cmsx_PidRead(void)
{

    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);
    for (uint8_t i = 0; i < 3; i++) {
        float tempP = pidProfile->pid[i].P;
        float tempI = pidProfile->pid[i].I;
        float tempD = pidProfile->pid[i].D;
        tempI = tempI / tempP * 100.0f;
        tempD = tempD / tempP * 100.0f;
        tempPid[i][0] = tempP;
        tempPid[i][1] = round(tempI);
        tempPid[i][2] = round(tempD);
        tempPidF[i] = pidProfile->pid[i].F;
    }
    tempPidDF = pidProfile->pid[YAW].DF;

    return NULL;
}

static const void *cmsx_PidOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    setProfileIndexString(pidProfileIndexString, pidProfileIndex, currentPidProfile->profileName);
    cmsx_PidRead();

    return NULL;
}

static const void *cmsx_PidWriteback(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    pidProfile_t *pidProfile = currentPidProfile;
    for (uint8_t i = 0; i < 3; i++) {
        float tempP = tempPid[i][0];
        float tempI = tempPid[i][1];
        float tempD = tempPid[i][2];
        tempI = tempI / 100.0f * tempP;
        tempD = tempD / 100.0f * tempP;
        pidProfile->pid[i].P = tempP;
        pidProfile->pid[i].I = round(tempI);
        pidProfile->pid[i].D = round(tempD);
        pidProfile->pid[i].F = tempPidF[i];
    }
    pidProfile->pid[YAW].DF = tempPidDF;
    pidInitConfig(currentPidProfile);

    return NULL;
}

static const OSD_Entry cmsx_menuPidEntries[] =
{
    { "-- PID RATIO --", OME_Label, NULL, pidProfileIndexString, 0},
    { "PID ADVANCED", OME_Submenu, cmsMenuChange, &cmsx_menuPidAdvanced, 0 },

    { "ROLL GAIN",  OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_ROLL][0],  0, 255, 1 }, 0 },
    { "ROLL PD",    OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_ROLL][2],  0, 255, 1 }, 0 },
    { "ROLL PI",    OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_ROLL][1],  0, 255, 1 }, 0 },
    { "ROLL F",     OME_UINT16, NULL, &(OSD_UINT16_t){ &tempPidF[PID_ROLL],  0, 2000, 1 }, 0 },

    { "PITCH GAIN", OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_PITCH][0], 0, 255, 1 }, 0 },
    { "PITCH PD",   OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_PITCH][2], 0, 255, 1 }, 0 },
    { "PITCH PI",   OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_PITCH][1], 0, 255, 1 }, 0 },
    { "PITCH F",    OME_UINT16, NULL, &(OSD_UINT16_t){ &tempPidF[PID_PITCH], 0, 2000, 1 }, 0 },

    { "YAW GAIN",   OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_YAW][0],   0, 255, 1 }, 0 },
    { "YAW PD",     OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_YAW][2],   0, 255, 1 }, 0 },
    { "YAW PI",     OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPid[PID_YAW][1],   0, 255, 1 }, 0 },
    { "YAW F",      OME_UINT16, NULL, &(OSD_UINT16_t){ &tempPidF[PID_YAW],   0, 2000, 1 }, 0 },
    { "YAW DF",     OME_UINT8, NULL, &(OSD_UINT8_t){ &tempPidDF,             0, 255, 1 }, 0 },

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
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuPidEntries
};

static uint8_t  cmsx_P_angle_low;
static uint8_t  cmsx_D_angle_low;
static uint8_t  cmsx_DF_angle_low;
static uint8_t  cmsx_P_angle_high;
static uint8_t  cmsx_D_angle_high;
static uint8_t  cmsx_DF_angle_high;
static uint16_t cmsx_F_angle;
static uint8_t  cmsx_angle_limit;
static uint8_t  cmsx_angle_expo;
static uint8_t  cmsx_horizon_strength;
static uint8_t  cmsx_horizon_transition;

static const void *cmsx_AngleOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);

    cmsx_P_angle_low =           pidProfile->pid[PID_LEVEL_LOW].P;
    cmsx_D_angle_low =           pidProfile->pid[PID_LEVEL_LOW].D;
    cmsx_DF_angle_low =          pidProfile->pid[PID_LEVEL_LOW].DF;
    cmsx_P_angle_high =          pidProfile->pid[PID_LEVEL_HIGH].P;
    cmsx_D_angle_high =          pidProfile->pid[PID_LEVEL_HIGH].D;
    cmsx_DF_angle_high =         pidProfile->pid[PID_LEVEL_HIGH].DF;
    cmsx_F_angle =               pidProfile->pid[PID_LEVEL_LOW].F;
    cmsx_angle_limit =           pidProfile->levelAngleLimit;
    cmsx_angle_expo =            pidProfile->angleExpo;
    cmsx_horizon_strength =      pidProfile->horizon_strength;
    cmsx_horizon_transition =    pidProfile->horizonTransition;

    return NULL;
}

static const void *cmsx_AngleWriteback(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);

    pidProfile->pid[PID_LEVEL_LOW].P =    cmsx_P_angle_low;
    pidProfile->pid[PID_LEVEL_LOW].D =    cmsx_D_angle_low;
    pidProfile->pid[PID_LEVEL_LOW].DF =   cmsx_DF_angle_low;
    pidProfile->pid[PID_LEVEL_HIGH].P =   cmsx_P_angle_high;
    pidProfile->pid[PID_LEVEL_HIGH].D =   cmsx_D_angle_high;
    pidProfile->pid[PID_LEVEL_HIGH].DF =  cmsx_DF_angle_high;
    pidProfile->pid[PID_LEVEL_LOW].F =    cmsx_F_angle;
    pidProfile->levelAngleLimit =         cmsx_angle_limit;
    pidProfile->angleExpo =               cmsx_angle_expo;
    pidProfile->horizon_strength =        cmsx_horizon_strength;
    pidProfile->horizonTransition =       cmsx_horizon_transition;

    pidInitConfig(currentPidProfile);

    return NULL;
}

static const OSD_Entry cmsx_menuAngleEntries[] =
{
    { "-- ANGLE PID --", OME_Label, NULL, pidProfileIndexString, 0},

    { "ANGLE P  LOW", OME_UINT8,  NULL, &(OSD_UINT8_t){ &cmsx_P_angle_low,  0, 200, 1 }, 0 },
    { "ANGLE D  LOW", OME_UINT8,  NULL, &(OSD_UINT8_t){ &cmsx_D_angle_low,  0, 200, 1 }, 0 },
    { "ANGLE DF LOW", OME_UINT8,  NULL, &(OSD_UINT8_t){ &cmsx_DF_angle_low,  0, 200, 1 }, 0 },
    { "ANGLE P  HIGH",OME_UINT8,  NULL, &(OSD_UINT8_t){ &cmsx_P_angle_high, 0, 200, 1 }, 0 },
    { "ANGLE D  HIGH",OME_UINT8,  NULL, &(OSD_UINT8_t){ &cmsx_D_angle_high, 0, 200, 1 }, 0 },
    { "ANGLE DF HIGH",OME_UINT8,  NULL, &(OSD_UINT8_t){ &cmsx_DF_angle_high, 0, 200, 1 }, 0 },
    { "ANGLE F",      OME_UINT16, NULL, &(OSD_UINT16_t){&cmsx_F_angle,     0, 2000, 1 }, 0 },

    { "ANGLE LIMIT",  OME_UINT8,  NULL, &(OSD_UINT8_t){ &cmsx_angle_limit, 10,  90, 1 }, 0 },
    { "ANGLE EXPO",   OME_UINT8,  NULL, &(OSD_UINT8_t){ &cmsx_angle_expo,   0, 100, 1 }, 0 },

    { "HORZN STR",    OME_UINT8,  NULL, &(OSD_UINT8_t){ &cmsx_horizon_strength,     0, 200, 1 }, 0 },
    { "HORZN TRANS",  OME_UINT8,  NULL, &(OSD_UINT8_t){ &cmsx_horizon_transition,   0, 200, 1 }, 0 },

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu cmsx_menuAngle = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XANGLE",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_AngleOnEnter,
    .onExit = cmsx_AngleWriteback,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuAngleEntries
};


//
// Rate & Expo
//

static const void *cmsx_RateProfileRead(void)
{
    memcpy(&rateProfile, controlRateProfiles(rateProfileIndex), sizeof(controlRateConfig_t));

    return NULL;
}

static const void *cmsx_RateProfileWriteback(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    memcpy(controlRateProfilesMutable(rateProfileIndex), &rateProfile, sizeof(controlRateConfig_t));

    return NULL;
}

static const void *cmsx_RateProfileOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    setProfileIndexString(rateProfileIndexString, rateProfileIndex, controlRateProfilesMutable(rateProfileIndex)->profileName);
    cmsx_RateProfileRead();

    return NULL;
}

static const OSD_Entry cmsx_menuRateProfileEntries[] =
{
    { "-- RATE --", OME_Label, NULL, rateProfileIndexString, 0 },

    { "RC CENTER SENS",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rateDynamics.rateSensCenter,        25, 175, 1, 10 }, 0 },
    { "RC END SENS",      OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rateDynamics.rateSensEnd,           25, 175, 1, 10 }, 0 },
    { "RC CENTER CORR",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rateDynamics.rateCorrectionCenter,  0,   95, 1, 10 }, 0 },
    { "RC END CORR",      OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rateDynamics.rateCorrectionEnd,     0,   95, 1, 10 }, 0 },
    { "RC CENTER WEIGHT", OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rateDynamics.rateWeightCenter,      0,   95, 1, 10 }, 0 },
    { "RC END WEIGHT",    OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rateDynamics.rateWeightEnd,         0,   95, 1, 10 }, 0 },

    { "RC R RATE",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcRates[FD_ROLL],    1, CONTROL_RATE_CONFIG_RC_RATES_MAX, 1, 10 }, 0 },
    { "RC P RATE",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcRates[FD_PITCH],    1, CONTROL_RATE_CONFIG_RC_RATES_MAX, 1, 10 }, 0 },
    { "RC Y RATE",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcRates[FD_YAW], 1, CONTROL_RATE_CONFIG_RC_RATES_MAX, 1, 10 }, 0 },

    { "ROLL SUPER",  OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rates[FD_ROLL],   0, CONTROL_RATE_CONFIG_RATE_MAX, 1, 10 }, 0 },
    { "PITCH SUPER", OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rates[FD_PITCH],   0, CONTROL_RATE_CONFIG_RATE_MAX, 1, 10 }, 0 },
    { "YAW SUPER",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rates[FD_YAW],   0, CONTROL_RATE_CONFIG_RATE_MAX, 1, 10 }, 0 },

    { "RC R EXPO",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcExpo[FD_ROLL],    0, 100, 1, 10 }, 0 },
    { "RC P EXPO",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcExpo[FD_PITCH],    0, 100, 1, 10 }, 0 },
    { "RC Y EXPO",   OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &rateProfile.rcExpo[FD_YAW], 0, 100, 1, 10 }, 0 },

    { "THR MID",     OME_UINT8,  NULL, &(OSD_UINT8_t) { &rateProfile.thrMid8,           0,  100,  1}, 0 },
    { "THR EXPO",    OME_UINT8,  NULL, &(OSD_UINT8_t) { &rateProfile.thrExpo8,          0,  100,  1}, 0 },

    { "THR LIM TYPE",OME_TAB,    NULL, &(OSD_TAB_t)   { &rateProfile.throttle_limit_type, THROTTLE_LIMIT_TYPE_COUNT - 1, osdTableThrottleLimitType}, 0 },
    { "THR LIM %",   OME_UINT8,  NULL, &(OSD_UINT8_t) { &rateProfile.throttle_limit_percent, 25,  100,  1}, 0 },

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
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuRateProfileEntries
};

#ifdef USE_LAUNCH_CONTROL
static uint8_t cmsx_launchControlMode;
static uint8_t cmsx_launchControlAllowTriggerReset;
static uint8_t cmsx_launchControlThrottlePercent;
static uint8_t cmsx_launchControlAngleLimit;
static uint8_t cmsx_launchControlGain;

static const void *cmsx_launchControlOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);

    cmsx_launchControlMode  = pidProfile->launchControlMode;
    cmsx_launchControlAllowTriggerReset  = pidProfile->launchControlAllowTriggerReset;
    cmsx_launchControlThrottlePercent  = pidProfile->launchControlThrottlePercent;
    cmsx_launchControlAngleLimit  = pidProfile->launchControlAngleLimit;
    cmsx_launchControlGain  = pidProfile->launchControlGain;

    return NULL;
}

static const void *cmsx_launchControlOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);

    pidProfile->launchControlMode = cmsx_launchControlMode;
    pidProfile->launchControlAllowTriggerReset = cmsx_launchControlAllowTriggerReset;
    pidProfile->launchControlThrottlePercent = cmsx_launchControlThrottlePercent;
    pidProfile->launchControlAngleLimit = cmsx_launchControlAngleLimit;
    pidProfile->launchControlGain = cmsx_launchControlGain;

    return NULL;
}

static const OSD_Entry cmsx_menuLaunchControlEntries[] = {
    { "-- LAUNCH CONTROL --", OME_Label, NULL, pidProfileIndexString, 0 },

    { "MODE",             OME_TAB,   NULL, &(OSD_TAB_t)   { &cmsx_launchControlMode, LAUNCH_CONTROL_MODE_COUNT - 1, osdLaunchControlModeNames}, 0 },
    { "ALLOW RESET",      OME_Bool,  NULL, &cmsx_launchControlAllowTriggerReset, 0 },
    { "TRIGGER THROTTLE", OME_UINT8, NULL, &(OSD_UINT8_t) { &cmsx_launchControlThrottlePercent, 0,  LAUNCH_CONTROL_THROTTLE_TRIGGER_MAX, 1 } , 0 },
    { "ANGLE LIMIT",      OME_UINT8, NULL, &(OSD_UINT8_t) { &cmsx_launchControlAngleLimit,      0,  80, 1 } , 0 },
    { "ITERM GAIN",       OME_UINT8, NULL, &(OSD_UINT8_t) { &cmsx_launchControlGain,            0, 200, 1 } , 0 },

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu cmsx_menuLaunchControl = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "LAUNCH",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_launchControlOnEnter,
    .onExit = cmsx_launchControlOnExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuLaunchControlEntries,
};
#endif

static uint8_t  cmsx_axis_lock_hz;
static uint8_t  cmsx_axis_lock_mult;
static uint8_t  cmsx_axis_smooth_mult;
static uint8_t  cmsx_dynThrP;
static uint8_t  cmsx_dynThrI;
static uint8_t  cmsx_dynThrD;
static uint16_t cmsx_tpaBreakpoint;
static uint8_t  cmsx_throttleBoost;
static uint8_t  cmsx_thrustLinearization;
static uint16_t cmsx_itermAcceleratorGain;
static uint8_t  cmsx_motorOutputLimit;
static int8_t   cmsx_autoProfileCellCount;

#ifdef USE_BATTERY_VOLTAGE_SAG_COMPENSATION
static uint8_t  cmsx_vbat_sag_compensation;
#endif


#ifdef USE_FEEDFORWARD
static uint8_t cmsx_feedforward_transition;
static uint8_t cmsx_feedforward_boost;
static uint8_t cmsx_feedforward_averaging;
static uint8_t cmsx_feedforward_smooth_factor;
static uint8_t cmsx_feedforward_jitter_factor;
#endif

static const void *cmsx_profileOtherOnEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    setProfileIndexString(pidProfileIndexString, pidProfileIndex, currentPidProfile->profileName);

    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);
    cmsx_axis_lock_hz = pidProfile->axis_lock_hz;
    cmsx_axis_lock_mult = pidProfile->axis_lock_multiplier;
    cmsx_axis_smooth_mult = pidProfile->axis_smooth_multiplier;

    cmsx_dynThrP                = pidProfile->dynThr[0];
    cmsx_dynThrI                = pidProfile->dynThr[1];
    cmsx_dynThrD                = pidProfile->dynThr[2];
    cmsx_tpaBreakpoint          = pidProfile->tpa_breakpoint;

    cmsx_itermAcceleratorGain   = pidProfile->itermAcceleratorGain;

    cmsx_throttleBoost = pidProfile->throttle_boost;
    cmsx_thrustLinearization = pidProfile->thrustLinearization;
    cmsx_motorOutputLimit = pidProfile->motor_output_limit;
    cmsx_autoProfileCellCount = pidProfile->auto_profile_cell_count;


#ifdef USE_FEEDFORWARD
    cmsx_feedforward_transition  = pidProfile->feedforward_transition;
    cmsx_feedforward_averaging = pidProfile->feedforward_averaging;
    cmsx_feedforward_boost = pidProfile->feedforward_boost;
    cmsx_feedforward_smooth_factor = pidProfile->feedforward_smooth_factor;
    cmsx_feedforward_jitter_factor = pidProfile->feedforward_jitter_factor;
#endif

#ifdef USE_BATTERY_VOLTAGE_SAG_COMPENSATION
    cmsx_vbat_sag_compensation = pidProfile->vbat_sag_compensation;
#endif
    return NULL;
}

static const void *cmsx_profileOtherOnExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    pidProfile_t *pidProfile = pidProfilesMutable(pidProfileIndex);
    pidProfile->axis_lock_hz = cmsx_axis_lock_hz;
    pidProfile->axis_lock_multiplier = cmsx_axis_lock_mult;
    pidProfile->axis_smooth_multiplier = cmsx_axis_smooth_mult;

    pidProfile->dynThr[0] = cmsx_dynThrP;
    pidProfile->dynThr[1] = cmsx_dynThrI;
    pidProfile->dynThr[2] = cmsx_dynThrD;
    pidProfile->tpa_breakpoint = cmsx_tpaBreakpoint;

    pidProfile->itermAcceleratorGain   = cmsx_itermAcceleratorGain;

    pidProfile->throttle_boost = cmsx_throttleBoost;
    pidProfile->thrustLinearization = cmsx_thrustLinearization;
    pidProfile->motor_output_limit = cmsx_motorOutputLimit;
    pidProfile->auto_profile_cell_count = cmsx_autoProfileCellCount;


#ifdef USE_FEEDFORWARD
    pidProfile->feedforward_transition = cmsx_feedforward_transition;
    pidProfile->feedforward_averaging = cmsx_feedforward_averaging;
    pidProfile->feedforward_boost = cmsx_feedforward_boost;
    pidProfile->feedforward_smooth_factor = cmsx_feedforward_smooth_factor;
    pidProfile->feedforward_jitter_factor = cmsx_feedforward_jitter_factor;
#endif

#ifdef USE_BATTERY_VOLTAGE_SAG_COMPENSATION
    pidProfile->vbat_sag_compensation = cmsx_vbat_sag_compensation;
#endif

    pidInitConfig(currentPidProfile);
    initEscEndpoints();
    return NULL;
}

static const OSD_Entry cmsx_menuProfileOtherEntries[] = {
    { "-- OTHER PP --", OME_Label, NULL, pidProfileIndexString, 0 },

    { "AXIS LOCK HZ",     OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_axis_lock_hz,                 1,    50,   1  }, 0 },
    { "AXIS LOCK MULT",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_axis_lock_mult,               0,    50,   1  }, 0 },
    { "AXIS SMOOTH MULT", OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_axis_smooth_mult,             0,    50,   1  }, 0 },

    { "TPA P",       OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &cmsx_dynThrP,           0,  200,  1, 10}, 0 },
    { "TPA I",       OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &cmsx_dynThrI,           0,  200,  1, 10}, 0 },
    { "TPA D",       OME_FLOAT,  NULL, &(OSD_FLOAT_t) { &cmsx_dynThrD,           0,  200,  1, 10}, 0 },
    { "TPA BRKPT",   OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_tpaBreakpoint,      1000,  2000, 10}, 0 },

#ifdef USE_FEEDFORWARD
    { "FF TRANSITION", OME_FLOAT,  NULL, &(OSD_FLOAT_t)  { &cmsx_feedforward_transition,        0,    100,   1, 10 }, 0 },
    { "FF AVERAGING",  OME_TAB,    NULL, &(OSD_TAB_t)    { &cmsx_feedforward_averaging,         4, lookupTableFeedforwardAveraging}, 0 },
    { "FF SMOOTHNESS", OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_feedforward_smooth_factor,     0,     75,   1  }   , 0 },
    { "FF JITTER",     OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_feedforward_jitter_factor,     0,     20,   1  }   , 0 },
    { "FF BOOST",      OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_feedforward_boost,             0,     50,   1  }   , 0 },
#endif
    { "AG GAIN",     OME_UINT16, NULL, &(OSD_UINT16_t) { &cmsx_itermAcceleratorGain,   ITERM_ACCELERATOR_GAIN_OFF, ITERM_ACCELERATOR_GAIN_MAX, 10 }   , 0 },
#ifdef USE_THROTTLE_BOOST
    { "THR BOOST",   OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_throttleBoost,          0,    100,   1  }   , 0 },
#endif
#ifdef USE_THRUST_LINEARIZATION
    { "THR LINEAR",  OME_UINT8,  NULL, &(OSD_UINT8_t)  { &cmsx_thrustLinearization,    0,    150,   1  }   , 0 },
#endif
#ifdef USE_LAUNCH_CONTROL
    {"LAUNCH CONTROL", OME_Submenu, cmsMenuChange, &cmsx_menuLaunchControl, 0 },
#endif
    { "MTR OUT LIM %",OME_UINT8, NULL, &(OSD_UINT8_t) { &cmsx_motorOutputLimit, MOTOR_OUTPUT_LIMIT_PERCENT_MIN,  MOTOR_OUTPUT_LIMIT_PERCENT_MAX,  1}, 0 },

    { "AUTO CELL CNT", OME_INT8, NULL, &(OSD_INT8_t) { &cmsx_autoProfileCellCount, AUTO_PROFILE_CELL_COUNT_CHANGE, MAX_AUTO_DETECT_CELL_COUNT, 1}, 0 },

#ifdef USE_BATTERY_VOLTAGE_SAG_COMPENSATION
    { "VBAT_SAG_COMP", OME_UINT8,  NULL, &(OSD_UINT8_t) { &cmsx_vbat_sag_compensation, 0, 150, 1 }, 0 },
#endif

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
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuProfileOtherEntries,
};

#ifdef USE_DYN_NOTCH_FILTER
static uint16_t dynFiltNotchMaxHz;
static uint8_t  dynFiltNotchCount;
static uint16_t dynFiltNotchQ;
static uint16_t dynFiltNotchMinHz;
#endif
#ifdef USE_DYN_LPF
static uint16_t dynFiltGyroMin;
static uint8_t dynFiltGyroWidth;
static uint8_t dynFiltGyroExpo;
static uint8_t dynFiltGyroGain;
#endif
#ifndef USE_GYRO_IMUF9001
static uint16_t gyroConfig_imuf_roll_q;
static uint16_t gyroConfig_imuf_pitch_q;
static uint16_t gyroConfig_imuf_yaw_q;
static uint16_t gyroConfig_imuf_w;
#endif
static uint16_t gyroConfig_alpha;
static uint16_t gyroConfig_abg_boost;
static uint16_t gyroConfig_abg_half_life;

static uint8_t  gyroConfig_gyro_to_use;
static uint8_t  gyroConfig_lpf_type;
static uint8_t  gyroConfig_lpf_order;

#ifdef USE_SMITH_PREDICTOR
static uint8_t smithPredictor_strength;
static uint8_t smithPredictor_delay;
static uint16_t smithPredictor_filt_hz;
#endif // USE_SMITH_PREDICTOR

static const void *cmsx_menuGyro_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);
#ifdef USE_DYN_LPF
    dynFiltGyroMin   = gyroConfig()->dyn_lpf_gyro_min_hz;
    dynFiltGyroWidth = gyroConfig()->dyn_lpf_gyro_width;
    dynFiltGyroExpo  = gyroConfig()->dyn_lpf_curve_expo;
    dynFiltGyroGain  = gyroConfig()->dyn_lpf_gyro_gain;
#endif
#ifdef USE_DYN_NOTCH_FILTER
    dynFiltNotchMaxHz   = dynNotchConfig()->dyn_notch_max_hz;
    dynFiltNotchCount   = dynNotchConfig()->dyn_notch_count;
    dynFiltNotchQ       = dynNotchConfig()->dyn_notch_q;
    dynFiltNotchMinHz   = dynNotchConfig()->dyn_notch_min_hz;
#endif
#ifndef USE_GYRO_IMUF9001
    gyroConfig_imuf_roll_q    = gyroConfig()->imuf_roll_q;
    gyroConfig_imuf_pitch_q   = gyroConfig()->imuf_pitch_q;
    gyroConfig_imuf_yaw_q     = gyroConfig()->imuf_yaw_q;
    gyroConfig_imuf_w         = gyroConfig()->imuf_w;
#endif
    gyroConfig_alpha          = gyroConfig()->alpha;
    gyroConfig_abg_boost      = gyroConfig()->abg_boost;
    gyroConfig_abg_half_life  = gyroConfig()->abg_half_life;

    gyroConfig_gyro_to_use    = gyroConfig()->gyro_to_use;
    gyroConfig_lpf_type       = gyroConfig()->gyro_lpf_type;
    gyroConfig_lpf_order      = gyroConfig()->gyro_lpf_order;

#ifdef USE_SMITH_PREDICTOR
    smithPredictor_strength  = gyroConfig()->smithPredictorStrength;
    smithPredictor_delay     = gyroConfig()->smithPredictorDelay;
    smithPredictor_filt_hz   = gyroConfig()->smithPredictorFilterHz;
#endif
    return NULL;
}

static const void *cmsx_menuGyro_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);
#ifdef USE_DYN_LPF
    gyroConfigMutable()->dyn_lpf_gyro_min_hz = dynFiltGyroMin;
    gyroConfigMutable()->dyn_lpf_gyro_width = dynFiltGyroWidth;
    gyroConfigMutable()->dyn_lpf_curve_expo  = dynFiltGyroExpo;
    gyroConfigMutable()->dyn_lpf_gyro_gain = dynFiltGyroGain;
#endif
#ifdef USE_DYN_NOTCH_FILTER
    dynNotchConfigMutable()->dyn_notch_max_hz        = dynFiltNotchMaxHz;
    dynNotchConfigMutable()->dyn_notch_count         = dynFiltNotchCount;
    dynNotchConfigMutable()->dyn_notch_q             = dynFiltNotchQ;
    dynNotchConfigMutable()->dyn_notch_min_hz        = dynFiltNotchMinHz;
#endif
#ifndef USE_GYRO_IMUF9001
    gyroConfigMutable()->imuf_roll_q    = gyroConfig_imuf_roll_q;
    gyroConfigMutable()->imuf_pitch_q   = gyroConfig_imuf_pitch_q;
    gyroConfigMutable()->imuf_yaw_q     = gyroConfig_imuf_yaw_q;
    gyroConfigMutable()->imuf_w         = gyroConfig_imuf_w;
#endif
    gyroConfigMutable()->alpha          = gyroConfig_alpha;
    gyroConfigMutable()->abg_boost      = gyroConfig_abg_boost;
    gyroConfigMutable()->abg_half_life  = gyroConfig_abg_half_life;

    gyroConfigMutable()->gyro_to_use = gyroConfig_gyro_to_use;
    gyroConfigMutable()->gyro_lpf_type = gyroConfig_lpf_type;
    gyroConfigMutable()->gyro_lpf_order = gyroConfig_lpf_order;

#ifdef USE_SMITH_PREDICTOR
    gyroConfigMutable()->smithPredictorStrength = smithPredictor_strength;
    gyroConfigMutable()->smithPredictorDelay = smithPredictor_delay;
    gyroConfigMutable()->smithPredictorFilterHz = smithPredictor_filt_hz;
#endif
    return NULL;
}

static const OSD_Entry cmsx_menuFilterGlobalEntries[] =
{
    { "-- GYRO FILTERS  --", OME_Label, NULL, NULL, 0 },
#ifdef USE_DYN_LPF
    { "LPF MIN",    OME_UINT16, NULL, &(OSD_UINT16_t) { &dynFiltGyroMin,  0, 1000, 1 }, 0 },
    { "LPF WIDTH",  OME_UINT8, NULL, &(OSD_UINT8_t) { &dynFiltGyroWidth,  0, 255, 1 }, 0 },
    { "LPF EXPO",   OME_UINT8, NULL, &(OSD_UINT8_t) { &dynFiltGyroExpo,   0, 10, 1 }, 0 },
    { "LPF GAIN",   OME_UINT8, NULL, &(OSD_UINT8_t) { &dynFiltGyroGain,   0, 200, 1 }, 0 },
    { "LPF TYPE",   OME_TAB,   NULL, &(OSD_TAB_t)   { &gyroConfig_lpf_type,     1, osdTableLpfType}, 0 },
    { "LPF ORDER",  OME_UINT8, NULL, &(OSD_UINT8_t) { &gyroConfig_lpf_order,    1, 4, 1}, 0 },
#endif
#ifdef USE_DYN_NOTCH_FILTER
    { "NOTCH COUNT",    OME_UINT8,  NULL, &(OSD_UINT8_t)  { &dynFiltNotchCount,   1, DYN_NOTCH_COUNT_MAX, 1 }, 0 },
    { "NOTCH Q",        OME_UINT16, NULL, &(OSD_UINT16_t) { &dynFiltNotchQ,       1, 1000, 1 }, 0 },
    { "NOTCH MIN HZ",   OME_UINT16, NULL, &(OSD_UINT16_t) { &dynFiltNotchMinHz,   0, 1000, 1 }, 0 },
    { "NOTCH MAX HZ",   OME_UINT16, NULL, &(OSD_UINT16_t) { &dynFiltNotchMaxHz,   0, 1000, 1 }, 0 },
#endif
#ifndef USE_GYRO_IMUF9001
    { "IMUF W",          OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_imuf_w,              0,   300,   1 }, 0 },
    { "IMUF ROLL Q",     OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_imuf_roll_q,       100, 16000, 100 }, 0 },
    { "IMUF PITCH Q",    OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_imuf_pitch_q,      100, 16000, 100 }, 0 },
    { "IMUF YAW Q",      OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_imuf_yaw_q,        100, 16000, 100 }, 0 },
#endif
    { "ALPHA",           OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_alpha,           0, 1000, 1 }, 0 },
    { "ABG BOOST",       OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_abg_boost,       0, 2000, 5 }, 0 },
    { "ABG HALF LIFE",   OME_UINT16, NULL, &(OSD_UINT16_t) { &gyroConfig_abg_half_life,   0, 1000, 1 }, 0 },
#ifdef USE_SMITH_PREDICTOR
    { "SMITH STR",       OME_UINT8,  NULL, &(OSD_UINT8_t)  { &smithPredictor_strength,    0, 100, 1 }, 0 },
    { "SMITH DELAY",     OME_UINT8,  NULL, &(OSD_UINT8_t)  { &smithPredictor_delay,       0, 60, 1 }, 0 },
    { "SMITH FILT",      OME_UINT16,  NULL, &(OSD_UINT16_t)  { &smithPredictor_filt_hz,   1, 1000, 1 }, 0 },
#endif
#ifdef USE_MULTI_GYRO
    { "GYRO TO USE",  OME_TAB,  NULL, &(OSD_TAB_t)    { &gyroConfig_gyro_to_use,  2, osdTableGyroToUse}, REBOOT_REQUIRED },
#endif

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
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuFilterGlobalEntries,
};

#ifdef USE_DYN_LPF
static uint16_t dynFiltDtermMin;
static uint8_t dynFiltDtermWidth;
static uint8_t dynFiltDtermExpo;
static uint8_t dynFiltDtermGain;
#endif
static uint16_t cmsx_dterm_lpf2_hz;
static uint16_t dterm_alpha;
static uint16_t dterm_abg_boost;
static uint16_t dterm_abg_half_life;
static uint8_t cmsx_dterm_lpf_type;
static uint8_t cmsx_dterm_lpf2_type;
static uint8_t cmsx_dterm_lpf_order;
static uint8_t cmsx_dterm_lpf2_order;
static const void *cmsx_DtermFilterRead(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    const pidProfile_t *pidProfile = pidProfiles(pidProfileIndex);
#ifdef USE_DYN_LPF
    dynFiltDtermMin = pidProfile->dyn_lpf_dterm_min_hz;
    dynFiltDtermWidth = pidProfile->dyn_lpf_dterm_width;
    dynFiltDtermExpo = pidProfile->dyn_lpf_curve_expo;
    dynFiltDtermGain = pidProfile->dyn_lpf_dterm_gain;
#endif
    cmsx_dterm_lpf2_hz  = pidProfile->dterm_lpf2_hz;

    dterm_alpha               = pidProfile->dtermAlpha;
    dterm_abg_boost           = pidProfile->dterm_abg_boost;
    dterm_abg_half_life       = pidProfile->dterm_abg_half_life;
    cmsx_dterm_lpf_type       = pidProfile->dterm_filter_type;
    cmsx_dterm_lpf2_type      = pidProfile->dterm_filter2_type;
    cmsx_dterm_lpf_order      = pidProfile->dterm_filter_order;
    cmsx_dterm_lpf2_order     = pidProfile->dterm_filter2_order;

    return NULL;
}

static const void *cmsx_DtermFilterWriteback(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    pidProfile_t *pidProfile = currentPidProfile;
#ifdef USE_DYN_LPF
    pidProfile->dyn_lpf_dterm_min_hz         = dynFiltDtermMin;
    pidProfile->dyn_lpf_dterm_width         = dynFiltDtermWidth;
    pidProfile->dyn_lpf_curve_expo           = dynFiltDtermExpo;
    pidProfile->dyn_lpf_dterm_gain = dynFiltDtermGain;
#endif
    pidProfile->dterm_lpf2_hz  = cmsx_dterm_lpf2_hz;

    pidProfile->dtermAlpha              = dterm_alpha;
    pidProfile->dterm_abg_boost         = dterm_abg_boost;
    pidProfile->dterm_abg_half_life     = dterm_abg_half_life;

    pidProfile->dterm_filter_type       = cmsx_dterm_lpf_type;
    pidProfile->dterm_filter2_type      = cmsx_dterm_lpf2_type;
    pidProfile->dterm_filter_order      = cmsx_dterm_lpf_order;
    pidProfile->dterm_filter2_order     = cmsx_dterm_lpf2_order;
    return NULL;
}

static const OSD_Entry cmsx_menuDtermFilterEntries[] =
{
    { "-- DTERM FILTERS  --", OME_Label, NULL, NULL, 0 },
#ifdef USE_DYN_LPF
    { "LPF MIN",   OME_UINT16, NULL, &(OSD_UINT16_t) { &dynFiltDtermMin, 0, 1000, 1 }, 0 },
    { "LPF WIDTH", OME_UINT8, NULL, &(OSD_UINT8_t) { &dynFiltDtermWidth, 0, 255, 1 }, 0 },
    { "LPF EXPO",  OME_UINT8, NULL, &(OSD_UINT8_t) { &dynFiltDtermExpo, 0, 10,   1 }, 0 },
    { "LPF GAIN",  OME_UINT8, NULL, &(OSD_UINT8_t) { &dynFiltDtermGain, 0,  200, 1 }, 0 },
    { "LPF TYPE",  OME_TAB,   NULL, &(OSD_TAB_t)   { &cmsx_dterm_lpf_type,     1, osdTableLpfType}, 0 },
    { "LPF ORDER", OME_UINT8, NULL, &(OSD_UINT8_t) { &cmsx_dterm_lpf_order, 0, 4, 1 }, 0 },
#endif
    { "LPF2",      OME_UINT16, NULL, &(OSD_UINT16_t){ &cmsx_dterm_lpf2_hz,    0, FILTER_FREQUENCY_MAX, 1 }, 0 },
    { "LPF2 TYPE", OME_TAB,    NULL, &(OSD_TAB_t)   { &cmsx_dterm_lpf2_type,      1, osdTableLpfType}, 0 },
    { "LPF2 ORDER",OME_UINT8,  NULL, &(OSD_UINT8_t) { &cmsx_dterm_lpf2_order, 0, 4, 1 }, 0 },

    { "ALPHA",         OME_UINT16, NULL, &(OSD_UINT16_t) { &dterm_alpha,            0, 1000, 1 }, 0 },
    { "ABG BOOST",     OME_UINT16, NULL, &(OSD_UINT16_t) { &dterm_abg_boost,        0, 2000, 5 }, 0 },
    { "ABG HALF LIFE", OME_UINT16, NULL, &(OSD_UINT16_t) { &dterm_abg_half_life,    0, 1000, 1 }, 0 },
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

static CMS_Menu cmsx_menuDtermFilter = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "XDFLT",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_DtermFilterRead,
    .onExit = cmsx_DtermFilterWriteback,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuDtermFilterEntries,
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

static const void *cmsx_menuCopyProfile_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    cmsx_dstPidProfile = 0;
    cmsx_dstControlRateProfile = 0;

    return NULL;
}

static const void *cmsx_CopyPidProfile(displayPort_t *pDisplay, const void *ptr)
{
    UNUSED(pDisplay);
    UNUSED(ptr);

    if (cmsx_dstPidProfile > 0) {
        pidCopyProfile(cmsx_dstPidProfile - 1, getCurrentPidProfileIndex());
    }

    return NULL;
}

static const void *cmsx_CopyControlRateProfile(displayPort_t *pDisplay, const void *ptr)
{
    UNUSED(pDisplay);
    UNUSED(ptr);

    if (cmsx_dstControlRateProfile > 0) {
        copyControlRateProfile(cmsx_dstControlRateProfile - 1, getCurrentControlRateProfileIndex());
    }

    return NULL;
}

static const OSD_Entry cmsx_menuCopyProfileEntries[] =
{
    { "-- COPY PROFILE --", OME_Label, NULL, NULL, 0},

    { "CPY PID PROF TO",   OME_TAB,      NULL,                        &cmsx_PidProfileTable, 0 },
    { "COPY PP",           OME_Funcall,  cmsx_CopyPidProfile,         NULL, 0 },
    { "CPY RATE PROF TO",  OME_TAB,      NULL,                        &cmsx_ControlRateProfileTable, 0 },
    { "COPY RP",           OME_Funcall,  cmsx_CopyControlRateProfile, NULL, 0 },

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
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuCopyProfileEntries,
};

#endif

static const OSD_Entry cmsx_menuImuEntries[] =
{
    { "-- PROFILE --", OME_Label, NULL, NULL, 0},

    {"PID PROF",    OME_UINT8,   cmsx_profileIndexOnChange,     &(OSD_UINT8_t){ &tmpPidProfileIndex, 1, PID_PROFILE_COUNT, 1}, 0},
    {"PID RATIO",   OME_Submenu, cmsMenuChange,                 &cmsx_menuPid,                                                 0},
    {"DTERM FILT",  OME_Submenu, cmsMenuChange,                 &cmsx_menuDtermFilter,                                         0},
    {"GYRO FILT",   OME_Submenu, cmsMenuChange,                 &cmsx_menuFilterGlobal,                                        0},
    {"ANGLE HORZN", OME_Submenu, cmsMenuChange,                 &cmsx_menuAngle,                                               0},
    {"MISC PP",     OME_Submenu, cmsMenuChange,                 &cmsx_menuProfileOther,                                        0},
    {"RATE PROF",   OME_UINT8,   cmsx_rateProfileIndexOnChange, &(OSD_UINT8_t){ &tmpRateProfileIndex, 1, CONTROL_RATE_PROFILE_COUNT, 1}, 0},
    {"RATE",        OME_Submenu, cmsMenuChange,                 &cmsx_menuRateProfile,                                         0},

#ifdef USE_EXTENDED_CMS_MENUS
    {"COPY PROF",   OME_Submenu, cmsMenuChange,                 &cmsx_menuCopyProfile,                                         0},
#endif /* USE_EXTENDED_CMS_MENUS */

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
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuImuEntries,
};

#endif // CMS
