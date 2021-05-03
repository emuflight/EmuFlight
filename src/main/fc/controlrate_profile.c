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

#include "platform.h"

#include "common/axis.h"

#include "config/config_reset.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "config/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"

controlRateConfig_t *currentControlRateProfile;

PG_REGISTER_ARRAY_WITH_RESET_FN(controlRateConfig_t, CONTROL_RATE_PROFILE_COUNT, controlRateProfiles, PG_CONTROL_RATE_PROFILES, 1);

void pgResetFn_controlRateProfiles(controlRateConfig_t *controlRateConfig)
{
  for (int i = 0; i < CONTROL_RATE_PROFILE_COUNT; i++) {
    RESET_CONFIG(controlRateConfig_t, &controlRateConfig[i],
      // default rate dynamics do nothing to effect stick feels
        .rateDynamics = {100, 100, 10, 10, 0, 0,}, // SensitivityLow, SensitivityHigh, Correctionlow, CorrectionHigh, Weightlow, WeightHigh
        .thrMid8 = 50,
        .thrExpo8 = 0,
        .rates_type = RATES_TYPE_BETAFLIGHT,
        .rcRates[FD_ROLL] = 100,
        .rcRates[FD_PITCH] = 100,
        .rcRates[FD_YAW] = 100,
        .rcExpo[FD_ROLL] = 0,
        .rcExpo[FD_PITCH] = 0,
        .rcExpo[FD_YAW] = 0,
        .rates[FD_ROLL] = 70,
        .rates[FD_PITCH] = 70,
        .rates[FD_YAW] = 70,
        .throttle_limit_type = THROTTLE_LIMIT_TYPE_OFF,
        .throttle_limit_percent = 100,
        .addRollToYawRc = 0,
        .addYawToRollRc = 0,
        .rollPitchMagExpo = 0,
        .profileName = { 0 },
    );
  }
    // if (CONTROL_RATE_PROFILE_COUNT > 1)
    // {
    //     RESET_CONFIG(controlRateConfig_t, &controlRateConfig[1],
    //       // default rate dynamics do nothing to effect stick feels
    //         .rateDynamics = { 80, 120, 35, 35, 25, 25 }, // SensitivityLow, SensitivityHigh, Correctionlow, CorrectionHigh, Weightlow, WeightHigh
    //         .thrMid8 = 50,
    //         .thrExpo8 = 0,
    //         .rates_type = RATES_TYPE_BETAFLIGHT,
    //         .rcRates[FD_ROLL] = 217,
    //         .rcRates[FD_PITCH] = 217,
    //         .rcRates[FD_YAW] = 217,
    //         .rcExpo[FD_ROLL] = 71,
    //         .rcExpo[FD_PITCH] = 71,
    //         .rcExpo[FD_YAW] = 71,
    //         .rates[FD_ROLL] = 4,
    //         .rates[FD_PITCH] = 4,
    //         .rates[FD_YAW] = 4,
    //         .throttle_limit_type = THROTTLE_LIMIT_TYPE_OFF,
    //         .throttle_limit_percent = 100,
    //         .addRollToYawRc = 0,
    //         .addYawToRollRc = 0,
    //         .rollPitchMagExpo = 0,
    //         .profileName = { "QckFlsh" },
    //     );
    // }
    // if (CONTROL_RATE_PROFILE_COUNT > 2)
    // {
    //     RESET_CONFIG(controlRateConfig_t, &controlRateConfig[2],
    //       // default rate dynamics do nothing to effect stick feels
    //         .rateDynamics = { 80, 100, 25, 15, 50, 35 }, // SensitivityLow, SensitivityHigh, Correctionlow, CorrectionHigh, Weightlow, WeightHigh
    //         .thrMid8 = 50,
    //         .thrExpo8 = 0,
    //         .rates_type = RATES_TYPE_BETAFLIGHT,
    //         .rcRates[FD_ROLL] = 120,
    //         .rcRates[FD_PITCH] = 120,
    //         .rcRates[FD_YAW] = 120,
    //         .rcExpo[FD_ROLL] = 0,
    //         .rcExpo[FD_PITCH] = 0,
    //         .rcExpo[FD_YAW] = 0,
    //         .rates[FD_ROLL] = 70,
    //         .rates[FD_PITCH] = 70,
    //         .rates[FD_YAW] = 70,
    //         .throttle_limit_type = THROTTLE_LIMIT_TYPE_OFF,
    //         .throttle_limit_percent = 100,
    //         .addRollToYawRc = 0,
    //         .addYawToRollRc = 0,
    //         .rollPitchMagExpo = 0,
    //         .profileName = { "DrSchnk" },
    //     );
    // }
    // if (CONTROL_RATE_PROFILE_COUNT > 3)
    // {
    //     RESET_CONFIG(controlRateConfig_t, &controlRateConfig[3],
    //       // default rate dynamics do nothing to effect stick feels
    //         .rateDynamics = { 100, 100, 10, 10, 0, 0 }, // SensitivityLow, SensitivityHigh, Correctionlow, CorrectionHigh, Weightlow, WeightHigh
    //         .thrMid8 = 50,
    //         .thrExpo8 = 0,
    //         .rates_type = RATES_TYPE_BETAFLIGHT,
    //         .rcRates[FD_ROLL] = 131,
    //         .rcRates[FD_PITCH] = 131,
    //         .rcRates[FD_YAW] = 131,
    //         .rcExpo[FD_ROLL] = 0,
    //         .rcExpo[FD_PITCH] = 0,
    //         .rcExpo[FD_YAW] = 0,
    //         .rates[FD_ROLL] = 71,
    //         .rates[FD_PITCH] = 71,
    //         .rates[FD_YAW] = 71,
    //         .throttle_limit_type = THROTTLE_LIMIT_TYPE_OFF,
    //         .throttle_limit_percent = 100,
    //         .addRollToYawRc = 0,
    //         .addYawToRollRc = 0,
    //         .rollPitchMagExpo = 0,
    //         .profileName = { "Kore" },
    //     );
    // }
    // if (CONTROL_RATE_PROFILE_COUNT > 4)
    // {
    //     RESET_CONFIG(controlRateConfig_t, &controlRateConfig[4],
    //       // default rate dynamics do nothing to effect stick feels
    //         .rateDynamics = { 100, 100, 10, 10, 0, 0 }, // SensitivityLow, SensitivityHigh, Correctionlow, CorrectionHigh, Weightlow, WeightHigh
    //         .thrMid8 = 50,
    //         .thrExpo8 = 0,
    //         .rates_type = RATES_TYPE_BETAFLIGHT,
    //         .rcRates[FD_ROLL] = 92,
    //         .rcRates[FD_PITCH] = 92,
    //         .rcRates[FD_YAW] = 92,
    //         .rcExpo[FD_ROLL] = 30,
    //         .rcExpo[FD_PITCH] = 30,
    //         .rcExpo[FD_YAW] = 30,
    //         .rates[FD_ROLL] = 77,
    //         .rates[FD_PITCH] = 77,
    //         .rates[FD_YAW] = 77,
    //         .throttle_limit_type = THROTTLE_LIMIT_TYPE_OFF,
    //         .throttle_limit_percent = 100,
    //         .addRollToYawRc = 0,
    //         .addYawToRollRc = 0,
    //         .rollPitchMagExpo = 0,
    //         .profileName = { "NerdCopt" },
    //     );
    // }
    // if (CONTROL_RATE_PROFILE_COUNT > 5)
    // {
    //     RESET_CONFIG(controlRateConfig_t, &controlRateConfig[5],
    //       // default rate dynamics do nothing to effect stick feels
    //         .rateDynamics = { 100, 100, 10, 10, 0, 0 }, // SensitivityLow, SensitivityHigh, Correctionlow, CorrectionHigh, Weightlow, WeightHigh
    //         .thrMid8 = 50,
    //         .thrExpo8 = 0,
    //         .rates_type = RATES_TYPE_BETAFLIGHT,
    //         .rcRates[FD_ROLL] = 103,
    //         .rcRates[FD_PITCH] = 103,
    //         .rcRates[FD_YAW] = 103,
    //         .rcExpo[FD_ROLL] = 25,
    //         .rcExpo[FD_PITCH] = 20,
    //         .rcExpo[FD_YAW] = 20,
    //         .rates[FD_ROLL] = 72,
    //         .rates[FD_PITCH] = 72,
    //         .rates[FD_YAW] = 68,
    //         .throttle_limit_type = THROTTLE_LIMIT_TYPE_OFF,
    //         .throttle_limit_percent = 100,
    //         .addRollToYawRc = 0,
    //         .addYawToRollRc = 0,
    //         .rollPitchMagExpo = 0,
    //         .profileName = { "Shikijo" },
    //     );
    // }
}

const ratesSettingsLimits_t ratesSettingLimits[RATES_TYPE_COUNT] = {
    [RATES_TYPE_BETAFLIGHT] = { 255, 100, 100 },
    [RATES_TYPE_RACEFLIGHT] = { 200, 255, 100 },
    [RATES_TYPE_KISS]       = { 255,  99, 100 },
    [RATES_TYPE_ACTUAL]     = { 200, 200, 100 },
};

void loadControlRateProfile(void)
{
    currentControlRateProfile = controlRateProfilesMutable(systemConfig()->activeRateProfile);
}

void changeControlRateProfile(uint8_t controlRateProfileIndex)
{
    if (controlRateProfileIndex < CONTROL_RATE_PROFILE_COUNT) {
        systemConfigMutable()->activeRateProfile = controlRateProfileIndex;
    }

    loadControlRateProfile();
    initRcProcessing();
}

void copyControlRateProfile(const uint8_t dstControlRateProfileIndex, const uint8_t srcControlRateProfileIndex) {
    if ((dstControlRateProfileIndex < CONTROL_RATE_PROFILE_COUNT && srcControlRateProfileIndex < CONTROL_RATE_PROFILE_COUNT)
        && dstControlRateProfileIndex != srcControlRateProfileIndex
    ) {
        memcpy(controlRateProfilesMutable(dstControlRateProfileIndex), controlRateProfiles(srcControlRateProfileIndex), sizeof(controlRateConfig_t));
    }
}
