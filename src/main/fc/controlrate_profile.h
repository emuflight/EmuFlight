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

#pragma once

#include <stdint.h>

#include "pg/pg.h"

#define CONTROL_RATE_PROFILE_COUNT  6

typedef enum {
    RATES_TYPE_BETAFLIGHT = 0,
    RATES_TYPE_RACEFLIGHT,
} ratesType_e;

typedef enum {
    THROTTLE_LIMIT_TYPE_OFF = 0,
    THROTTLE_LIMIT_TYPE_SCALE,
    THROTTLE_LIMIT_TYPE_CLIP,
} throttleLimitType_e;

typedef enum {
    VBAT_COMP_TYPE_OFF = 0,
    VBAT_COMP_TYPE_BOOST,
    VBAT_COMP_TYPE_LIMIT,
    VBAT_COMP_TYPE_BOTH,
    VBAT_COMP_TYPE_COUNT   // must be the last entry
} throttleVbatCompType_e;

typedef struct controlRateConfig_s {
    uint8_t thrMid8;
    uint8_t thrExpo8;
    uint8_t rates_type;
    uint8_t rcRates[3];
    uint8_t rcExpo[3];
    uint8_t rates[3];
    uint8_t dynThrP;                        // TPA seperated into PID components
    uint8_t dynThrI;
    uint8_t dynThrD;
    uint16_t tpa_breakpoint;                // Breakpoint where TPA is activated
    uint8_t throttle_limit_type;            // Sets the throttle limiting type - off, scale or clip
    uint8_t throttle_limit_percent;         // Sets the maximum pilot commanded throttle limit
    uint8_t vbat_comp_type;                 // Sets the type of battery compensation: off, boost, limit or both
    uint8_t vbat_comp_ref;                  // Sets the voltage reference to calculate the battery compensation
    uint8_t vbat_comp_throttle_level;       // Sets the level of throttle battery compensation
    uint8_t vbat_comp_pid_level;            // Sets the level of PID battery compensation
    uint8_t vbat_comp_motor_output;         // Tells whether the battery compensation has to be applied to motor output or not
} controlRateConfig_t;

PG_DECLARE_ARRAY(controlRateConfig_t, CONTROL_RATE_PROFILE_COUNT, controlRateProfiles);

extern controlRateConfig_t *currentControlRateProfile;

void loadControlRateProfile(void);
void changeControlRateProfile(uint8_t controlRateProfileIndex);

void copyControlRateProfile(const uint8_t dstControlRateProfileIndex, const uint8_t srcControlRateProfileIndex);
