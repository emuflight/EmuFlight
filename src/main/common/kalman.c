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

#include <string.h>
#include "arm_math.h"

#include "kalman.h"
#include "filter.h"
#include "fc/fc_rc.h"
#include "build/debug.h"

kalman_t    kalmanFilterStateRate[XYZ_AXIS_COUNT];

void init_kalman(kalman_t *filter, float q, float updateRate)
{
    memset(filter, 0, sizeof(kalman_t));
    filter->q = q * 0.001f;             //add multiplier to make tuning easier
    filter->r = 88.0f;                  //seeding R at 88.0f
    filter->p = 30.0f;                  //seeding P at 30.0f
    filter->e = 1.0f;
    filter->s = gyroConfig()->imuf_sharpness / 250.0f;     //adding the new sharpness :) time to overfilter :O
    filter->w = gyroConfig()->imuf_w;
    filter->inverseN = 1.0f/(float)(filter->w);
    // set cutoff frequency
    const float k = pt1FilterGain(70, updateRate);
    pt1FilterInit(&filter->lp_filter, k);
    filter->lp_filter.state = 1.0f;		// e's default value
    filter->updateRate = updateRate;
}


void kalman_init(void)
{
    isSetpointNew = 0;

    init_kalman(&kalmanFilterStateRate[X],  gyroConfig()->imuf_roll_q, gyro.targetLooptime * 1e-6f);
    init_kalman(&kalmanFilterStateRate[Y],  gyroConfig()->imuf_pitch_q, gyro.targetLooptime * 1e-6f);
    init_kalman(&kalmanFilterStateRate[Z],  gyroConfig()->imuf_yaw_q, gyro.targetLooptime * 1e-6f);
}

void update_kalman_covariance(float gyroRateData, int axis)
{
     kalmanFilterStateRate[axis].axisWindow[kalmanFilterStateRate[axis].windex] = gyroRateData;
     kalmanFilterStateRate[axis].axisSumMean += kalmanFilterStateRate[axis].axisWindow[kalmanFilterStateRate[axis].windex];
     kalmanFilterStateRate[axis].axisSumVar = kalmanFilterStateRate[axis].axisSumVar + (kalmanFilterStateRate[axis].axisWindow[kalmanFilterStateRate[axis].windex] *  kalmanFilterStateRate[axis].axisWindow[ kalmanFilterStateRate[axis].windex]);
     kalmanFilterStateRate[axis].windex++;
    if ( kalmanFilterStateRate[axis].windex >= kalmanFilterStateRate[axis].w)
    {
         kalmanFilterStateRate[axis].windex = 0;
    }
     kalmanFilterStateRate[axis].axisSumMean -= kalmanFilterStateRate[axis].axisWindow[kalmanFilterStateRate[axis].windex];
     kalmanFilterStateRate[axis].axisSumVar = kalmanFilterStateRate[axis].axisSumVar - (kalmanFilterStateRate[axis].axisWindow[kalmanFilterStateRate[axis].windex] * kalmanFilterStateRate[axis].axisWindow[kalmanFilterStateRate[axis].windex]);
     kalmanFilterStateRate[axis].axisMean = kalmanFilterStateRate[axis].axisSumMean * kalmanFilterStateRate[axis].inverseN;
     kalmanFilterStateRate[axis].axisVar = fabsf(kalmanFilterStateRate[axis].axisSumVar * kalmanFilterStateRate[axis].inverseN - (kalmanFilterStateRate[axis].axisMean * kalmanFilterStateRate[axis].axisMean));

    kalmanFilterStateRate[axis].r = sqrtf(kalmanFilterStateRate[axis].axisVar) * VARIANCE_SCALE;
}

FAST_CODE float kalman_process(kalman_t* kalmanState, float input, float target)
{
  float targetAbs = fabsf(target);
  //project the state ahead using acceleration
  kalmanState->x += (kalmanState->x - kalmanState->lastX);

  //figure out how much to boost or reduce our error in the estimate based on setpoint target.
  //this should be close to 0 as we approach the sepoint and really high the futher away we are from the setpoint.
  //update last state
  kalmanState->lastX = kalmanState->x;

  if (kalmanState->lastX != 0.0f) {
  // calculate the error and add multiply sharpness boost
  	float errorMultiplier = fabsf(target - kalmanState->x) * kalmanState->s;

  // give a boost to the setpoint, used to caluclate the kalman q, based on the error and setpoint/gyrodata

  	errorMultiplier = constrainf(errorMultiplier * fabsf(1.0f - (target / kalmanState->lastX)) + 1.0f, 1.0f, 50.0f);

    kalmanState->e = fabsf(1.0f - (((targetAbs + 1.0f) * errorMultiplier) / fabsf(kalmanState->lastX)));
  }

  //prediction update
  kalmanState->p = kalmanState->p + (kalmanState->q * kalmanState->e);

  //measurement update
  kalmanState->k = kalmanState->p / (kalmanState->p + kalmanState->r);
  kalmanState->x += kalmanState->k * (input - kalmanState->x);
  kalmanState->p = (1.0f - kalmanState->k) * kalmanState->p;

  const float cutoff_frequency = constrain(70 * kalmanState->e, 10.0f, 500.0f);
  const float k = pt1FilterGain(cutoff_frequency, kalmanState->updateRate);
  pt1FilterUpdateCutoff(&kalmanState->lp_filter, k);

  kalmanState->x = pt1FilterApply(&kalmanState->lp_filter, kalmanState->x);

  return kalmanState->x;
}


FAST_CODE float kalman_update(float input, int axis)
{

    update_kalman_covariance(input, axis);
    input = kalman_process(&kalmanFilterStateRate[axis], input, getSetpointRate(axis) );

    int16_t Kgain = (kalmanFilterStateRate[axis].k * 1000.0f);
    DEBUG_SET(DEBUG_KALMAN, axis, Kgain);                               //Kalman gain
    return input;
}
