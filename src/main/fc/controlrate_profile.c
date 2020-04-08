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

#include "platform.h"

#include "common/axis.h"

#include "config/config_reset.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/fc_rc.h"

controlRateConfig_t *currentControlRateProfile;

PG_REGISTER_ARRAY_WITH_RESET_FN(controlRateConfig_t, CONTROL_RATE_PROFILE_COUNT, controlRateProfiles, PG_CONTROL_RATE_PROFILES, 2);

void pgResetFn_controlRateProfiles(controlRateConfig_t *controlRateConfig)
{
    for (int i = 0; i < CONTROL_RATE_PROFILE_COUNT; i++) {
        RESET_CONFIG(controlRateConfig_t, &controlRateConfig[i],
          .rateDynamics = { // defaults do nothing to effect stick feels
              100, 100, 10, 10, 0, 0,   // PLow, PHigh, Ilow, IHigh, Dlow, Dhigh
          },
            .thrMid8 = 50,
            .thrExpo8 = 0,
            .dynThrP = 75,
            .dynThrI = 125,
            .dynThrD = 65,
            .tpa_breakpoint = 1600,
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
            .vbat_comp_type = VBAT_COMP_TYPE_OFF,
            .vbat_comp_ref = 37,
            .vbat_comp_throttle_level = 75,
            .vbat_comp_pid_level = 75,
        );
    }
}

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
        memcpy(controlRateProfilesMutable(dstControlRateProfileIndex), controlRateProfilesMutable(srcControlRateProfileIndex), sizeof(controlRateConfig_t));
    }
}
