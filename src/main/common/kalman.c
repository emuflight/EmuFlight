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
#ifndef USE_GYRO_IMUF9001
#include <string.h>
#include "arm_math.h"

#include "kalman.h"
#include "fc/fc_rc.h"
#include "build/debug.h"

kalman_t    kalmanFilterStateRate[XYZ_AXIS_COUNT];

void init_kalman(kalman_t *filter, float q) {
    memset(filter, 0, sizeof(kalman_t));
    filter->q = q * 0.0001f;             //add multiplier to make tuning easier
    filter->r = 88.0f;                  //seeding R at 88.0f
    filter->p = 30.0f;                  //seeding P at 30.0f
    filter->e = 1.0f;
    filter->w = gyroConfig()->imuf_w;
    filter->inverseN = 1.0f / (float)(filter->w);

    pt1FilterInit(&filter->kFilter, pt1FilterGain(50, gyro.targetLooptime * 1e-6f));
}


void kalman_init(void) {
    isSetpointNew = 0;
    init_kalman(&kalmanFilterStateRate[X],  gyroConfig()->imuf_roll_q);
    init_kalman(&kalmanFilterStateRate[Y],  gyroConfig()->imuf_pitch_q);
    init_kalman(&kalmanFilterStateRate[Z],  gyroConfig()->imuf_yaw_q);
}

void update_kalman_covariance(float rate, int axis) {
    kalmanFilterStateRate[axis].axisWindow[kalmanFilterStateRate[axis].windex] = rate;
    kalmanFilterStateRate[axis].axisSumMean += kalmanFilterStateRate[axis].axisWindow[kalmanFilterStateRate[axis].windex];
    float varianceElement = kalmanFilterStateRate[axis].axisWindow[kalmanFilterStateRate[axis].windex] - kalmanFilterStateRate[axis].axisMean;
    varianceElement = varianceElement * varianceElement;
    kalmanFilterStateRate[axis].axisSumVar += varianceElement;
    kalmanFilterStateRate[axis].varianceWindow[kalmanFilterStateRate[axis].windex] = varianceElement;
    kalmanFilterStateRate[axis].windex++;
    if (kalmanFilterStateRate[axis].windex > kalmanFilterStateRate[axis].w) {
        kalmanFilterStateRate[axis].windex = 0;
    }
    kalmanFilterStateRate[axis].axisSumMean -= kalmanFilterStateRate[axis].axisWindow[kalmanFilterStateRate[axis].windex];
    kalmanFilterStateRate[axis].axisSumVar -= kalmanFilterStateRate[axis].varianceWindow[kalmanFilterStateRate[axis].windex];
    //New mean
    kalmanFilterStateRate[axis].axisMean = kalmanFilterStateRate[axis].axisSumMean * kalmanFilterStateRate[axis].inverseN;
    kalmanFilterStateRate[axis].axisVar = kalmanFilterStateRate[axis].axisSumVar * kalmanFilterStateRate[axis].inverseN;
    float squirt;
    arm_sqrt_f32(kalmanFilterStateRate[axis].axisVar, &squirt);
    kalmanFilterStateRate[axis].r = squirt * VARIANCE_SCALE;
}

FAST_CODE float kalman_process(kalman_t* kalmanState, float input) {
    //project the state ahead using acceleration
    kalmanState->x += (kalmanState->x - kalmanState->lastX) * kalmanState->k;
    //figure out how much to boost or reduce our error in the estimate based on setpoint target.
    //this should be close to 0 as we approach the sepoint and really high the futher away we are from the setpoint.
    //update last state
    kalmanState->lastX = kalmanState->x;

    float e = constrainf(kalmanState->r / 45.0f + 0.005f, 0.005f, 0.9f);
    //make the 1 a configurable value for testing purposes
    e = -powf(e - 1.0f, 2) * 0.7f + (e - 1.0f) * (1.0f - 0.7f) + 1.0f;
    kalmanState->e = e;

    //prediction update
    kalmanState->p = kalmanState->p + (kalmanState->q * kalmanState->e);

    //measurement update
    kalmanState->k = kalmanState->p / (kalmanState->p + 10.0f);
    kalmanState->k = pt1FilterApply(&kalmanState->kFilter, kalmanState->k);
    kalmanState->x += kalmanState->k * (input - kalmanState->x);
    kalmanState->p = (1.0f - kalmanState->k) * kalmanState->p;
    return kalmanState->x;
}

FAST_CODE float kalman_update(float input, int axis) {
    float preFiltered;
    preFiltered = input;
    if (gyroConfig()->imuf_w >= 3) {
        input = kalman_process(&kalmanFilterStateRate[axis], input);
    }
    if (axis == FD_ROLL) {
        DEBUG_SET(DEBUG_KALMAN, 0, preFiltered); // prefiltered
        DEBUG_SET(DEBUG_KALMAN, 1, input); // postfiltered
        DEBUG_SET(DEBUG_KALMAN, 2, kalmanFilterStateRate[axis].r * 1000.0f); // covariance
        DEBUG_SET(DEBUG_KALMAN, 3, kalmanFilterStateRate[axis].k * 1000.0f); // gain
    }

    return input;
}
#endif
