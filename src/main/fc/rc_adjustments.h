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

#include <stdbool.h>
#include "pg/pg.h"
#include "fc/rc_modes.h"

typedef enum {
    ADJUSTMENT_NONE = 0,
    ADJUSTMENT_RC_RATE,
    ADJUSTMENT_RC_EXPO,
    ADJUSTMENT_THROTTLE_EXPO,
    ADJUSTMENT_PITCH_ROLL_RATE,
    ADJUSTMENT_YAW_RATE,
    ADJUSTMENT_PITCH_ROLL_P,
    ADJUSTMENT_PITCH_ROLL_I,
    ADJUSTMENT_PITCH_ROLL_D,
    ADJUSTMENT_YAW_P,
    ADJUSTMENT_YAW_I,
    ADJUSTMENT_YAW_D,
    ADJUSTMENT_RATE_PROFILE,
    ADJUSTMENT_PITCH_RATE,
    ADJUSTMENT_ROLL_RATE,
    ADJUSTMENT_PITCH_P,
    ADJUSTMENT_PITCH_I,
    ADJUSTMENT_PITCH_D,
    ADJUSTMENT_ROLL_P,
    ADJUSTMENT_ROLL_I,
    ADJUSTMENT_ROLL_D,
    ADJUSTMENT_RC_RATE_YAW,
    ADJUSTMENT_PITCH_ROLL_F,
    ADJUSTMENT_HORIZON_STRENGTH,
    ADJUSTMENT_ROLL_RC_RATE,
    ADJUSTMENT_PITCH_RC_RATE,
    ADJUSTMENT_ROLL_RC_EXPO,
    ADJUSTMENT_PITCH_RC_EXPO,
    ADJUSTMENT_PID_AUDIO,
    ADJUSTMENT_FUNCTION_COUNT
} adjustmentFunction_e;

typedef enum {
    ADJUSTMENT_MODE_STEP,
    ADJUSTMENT_MODE_SELECT
} adjustmentMode_e;

typedef union adjustmentConfig_u {
    uint8_t step;
    uint8_t switchPositions;
} adjustmentData_t;

typedef struct adjustmentConfig_s {
    adjustmentFunction_e adjustmentFunction;
    adjustmentMode_e mode;
    adjustmentData_t data;
} adjustmentConfig_t;

#define MAX_ADJUSTMENT_RANGE_COUNT 15

typedef struct adjustmentRange_s {
    // when aux channel is in range...
    uint8_t auxChannelIndex;
    channelRange_t range;

    // ..then apply the adjustment function to the auxSwitchChannel ...
    uint8_t adjustmentFunction;
    uint8_t auxSwitchChannelIndex;

    // ... via slot
    uint8_t adjustmentIndex;
    uint16_t adjustmentCenter;
    uint16_t adjustmentScale;
} adjustmentRange_t;

PG_DECLARE_ARRAY(adjustmentRange_t, MAX_ADJUSTMENT_RANGE_COUNT, adjustmentRanges);

#define ADJUSTMENT_INDEX_OFFSET 1

typedef struct adjustmentState_s {
    uint8_t auxChannelIndex;
    const adjustmentConfig_t *config;
    uint32_t timeoutAt;
} adjustmentState_t;

#ifndef MAX_SIMULTANEOUS_ADJUSTMENT_COUNT
#define MAX_SIMULTANEOUS_ADJUSTMENT_COUNT 4 // enough for 4 x 3position switches / 4 aux channel
#endif

void resetAdjustmentStates(void);
void updateAdjustmentStates(void);
struct controlRateConfig_s;
void processRcAdjustments(struct controlRateConfig_s *controlRateConfig);
const char *getAdjustmentsRangeName(void);
int getAdjustmentsRangeValue(void);
