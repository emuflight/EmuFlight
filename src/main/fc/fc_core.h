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

#include "common/time.h"
#include "pg/pg.h"

#if defined(USE_GPS) || defined(USE_MAG)
extern int16_t magHold;
#endif

typedef struct throttleCorrectionConfig_s {
    uint16_t throttle_correction_angle;     // the angle when the throttle correction is maximal. in 0.1 degres, ex 225 = 22.5 ,30.0, 450 = 45.0 deg
    uint8_t throttle_correction_value;      // the correction that will be applied at throttle_correction_angle.
} throttleCorrectionConfig_t;

PG_DECLARE(throttleCorrectionConfig_t, throttleCorrectionConfig);

union rollAndPitchTrims_u;
void applyAndSaveAccelerometerTrimsDelta(union rollAndPitchTrims_u *rollAndPitchTrimsDelta);
void handleInflightCalibrationStickPosition(void);

void resetArmingDisabled(void);

void disarm(void);
void tryArm(void);

bool processRx(timeUs_t currentTimeUs);
void updateArmingStatus(void);

void taskMainPidLoop(timeUs_t currentTimeUs);
bool isFlipOverAfterCrashMode(void);
int8_t calculateThrottlePercent(void);
uint8_t calculateThrottlePercentAbs(void);

void runawayTakeoffTemporaryDisable(uint8_t disableFlag);
bool isAirmodeActivated();
timeUs_t getLastDisarmTimeUs(void);
bool isTryingToArm();
void resetTryingToArm();

void subTaskTelemetryPollSensors(timeUs_t currentTimeUs);
