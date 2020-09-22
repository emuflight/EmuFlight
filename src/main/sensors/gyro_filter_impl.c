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

#include "platform.h"
#include "common/kalman.h"

static FAST_CODE void GYRO_FILTER_FUNCTION_NAME(void)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // DEBUG_GYRO_RAW records the raw value read from the sensor (not zero offset, not scaled)
        GYRO_FILTER_DEBUG_SET(DEBUG_GYRO_RAW, axis, gyro.rawSensorDev->gyroADCRaw[axis]);

        // scale gyro output to degrees per second
        float gyroADCf = gyro.gyroADC[axis];

        // DEBUG_GYRO_SCALED records the unfiltered, scaled gyro output
        GYRO_FILTER_DEBUG_SET(DEBUG_GYRO_SCALED, axis, lrintf(gyro.gyroADC[axis]));

#ifdef USE_RPM_FILTER
        gyroADCf = rpmFilterGyro(axis, gyroADCf);
#endif

        // apply static notch filters and software lowpass filters
        gyroADCf = gyro.notchFilter1ApplyFn((filter_t *)&gyro.notchFilter1[axis], gyroADCf);
        gyroADCf = gyro.lowpassFilterApplyFn((filter_t *)&gyro.lowpassFilter[axis], gyroADCf);

#ifdef USE_GYRO_DATA_ANALYSE
        if (featureIsEnabled(FEATURE_DYNAMIC_FILTER)) {
            if (axis == gyro.gyroDebugAxis) {
                GYRO_FILTER_DEBUG_SET(DEBUG_FFT, 0, lrintf(gyroADCf));
                GYRO_FILTER_DEBUG_SET(DEBUG_FFT_FREQ, 3, lrintf(gyroADCf));
                GYRO_FILTER_DEBUG_SET(DEBUG_DYN_LPF, 0, lrintf(gyroADCf));
            }

            gyroDataAnalysePush(&gyro.gyroAnalyseState, axis, gyroADCf);
            for (uint8_t p = 0; p < gyro.notchFilterDynCount; p++) {
                gyroADCf = gyro.notchFilterDynApplyFn((filter_t*)&gyro.notchFilterDyn[axis][p], gyroADCf);
            }

            if (axis == gyro.gyroDebugAxis) {
                GYRO_FILTER_DEBUG_SET(DEBUG_FFT, 1, lrintf(gyroADCf));
            }
        }
#endif
        if (axis == FD_ROLL) {
            DEBUG_SET(DEBUG_ABG, 0, lrintf(gyroADCf));
        } else if (axis == FD_PITCH) {
            DEBUG_SET(DEBUG_ABG, 2, lrintf(gyroADCf));
        }
        gyroADCf = gyro.alphaBetaGammaApplyFn((filter_t *)&gyro.alphaBetaGamma[axis], gyroADCf);
        if (axis == FD_ROLL) {
            DEBUG_SET(DEBUG_ABG, 1, lrintf(gyroADCf));
            DEBUG_SET(DEBUG_ABG_STATE, 0, lrintf(ABGVelocity(&gyro.alphaBetaGamma[axis])));
            DEBUG_SET(DEBUG_ABG_STATE, 1, lrintf(ABGAcceleration(&gyro.alphaBetaGamma[axis])));
            DEBUG_SET(DEBUG_ABG_STATE, 2, lrintf(ABGJerk(&gyro.alphaBetaGamma[axis])));
            DEBUG_SET(DEBUG_ABG_STATE, 3, lrintf(ABGResidualError(&gyro.alphaBetaGamma[axis])));
        } else if (axis == FD_PITCH) {
            DEBUG_SET(DEBUG_ABG, 3, lrintf(gyroADCf));
        }

#ifndef USE_GYRO_IMUF9001
        update_kalman_covariance(&gyro.kalmanFilterStateRate[axis], gyroADCf);
#endif

#ifdef USE_SMITH_PREDICTOR
        gyroADCf = applySmithPredictor(&gyro.smithPredictor[axis], gyroADCf, axis);
#endif

        // DEBUG_GYRO_FILTERED records the scaled, filtered, after all software filtering has been applied.
        GYRO_FILTER_DEBUG_SET(DEBUG_GYRO_FILTERED, axis, lrintf(gyroADCf));

        gyro.gyroADCf[axis] = gyroADCf;
    }
}
