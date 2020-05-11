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

#include "dtermkalman.h"
#include "fc/fc_rc.h"
#include "build/debug.h"
#include "flight/pid.h"

dtermkalman_t dtermkalmanFilterStateRate[XYZ_AXIS_COUNT];

void dterminit_kalman(dtermkalman_t *filter, float q, const pidProfile_t* pidProfile)
{
    memset(filter, 0, sizeof(dtermkalman_t));
    filter->q = q * 0.001f;             //add multiplier to make tuning easier
    filter->r = 88.0f;                  //seeding R at 88.0f
    filter->p = 30.0f;                  //seeding P at 30.0f
    filter->e = 1.0f;
    filter->s = pidProfile->imuf_sharpness / 250.0f;     //adding the new sharpness :) time to overfilter :O
    filter->w = pidProfile->imuf_w;
    filter->inverseN = 1.0f/(float)(filter->w);
}


void dtermkalman_init(const pidProfile_t* pidProfile)
{
    isSetpointNew = 0;

    dterminit_kalman(&dtermkalmanFilterStateRate[X],  pidProfile->imuf_roll_q, pidProfile);
    dterminit_kalman(&dtermkalmanFilterStateRate[Y],  pidProfile->imuf_pitch_q, pidProfile);
    dterminit_kalman(&dtermkalmanFilterStateRate[Z],  pidProfile->imuf_yaw_q, pidProfile);
}

void dtermupdate_kalman_covariance(float gyroRateData, int axis)
{
     dtermkalmanFilterStateRate[axis].axisWindow[ dtermkalmanFilterStateRate[axis].windex] = gyroRateData;
     dtermkalmanFilterStateRate[axis].axisSumMean +=  dtermkalmanFilterStateRate[axis].axisWindow[ dtermkalmanFilterStateRate[axis].windex];
     dtermkalmanFilterStateRate[axis].axisSumVar =  dtermkalmanFilterStateRate[axis].axisSumVar + ( dtermkalmanFilterStateRate[axis].axisWindow[ dtermkalmanFilterStateRate[axis].windex] *  dtermkalmanFilterStateRate[axis].axisWindow[ dtermkalmanFilterStateRate[axis].windex]);
     dtermkalmanFilterStateRate[axis].windex++;
    if ( dtermkalmanFilterStateRate[axis].windex >= dtermkalmanFilterStateRate[axis].w)
    {
         dtermkalmanFilterStateRate[axis].windex = 0;
    }
     dtermkalmanFilterStateRate[axis].axisSumMean -=  dtermkalmanFilterStateRate[axis].axisWindow[ dtermkalmanFilterStateRate[axis].windex];
     dtermkalmanFilterStateRate[axis].axisSumVar =  dtermkalmanFilterStateRate[axis].axisSumVar - ( dtermkalmanFilterStateRate[axis].axisWindow[ dtermkalmanFilterStateRate[axis].windex] *  dtermkalmanFilterStateRate[axis].axisWindow[ dtermkalmanFilterStateRate[axis].windex]);
     dtermkalmanFilterStateRate[axis].axisMean =  dtermkalmanFilterStateRate[axis].axisSumMean *  dtermkalmanFilterStateRate[axis].inverseN;
     dtermkalmanFilterStateRate[axis].axisVar =  fabsf(dtermkalmanFilterStateRate[axis].axisSumVar *  dtermkalmanFilterStateRate[axis].inverseN - ( dtermkalmanFilterStateRate[axis].axisMean *  dtermkalmanFilterStateRate[axis].axisMean));

    dtermkalmanFilterStateRate[axis].r = sqrtf(dtermkalmanFilterStateRate[axis].axisVar) * VARIANCE_SCALE;

}

FAST_CODE float dtermkalman_process(dtermkalman_t* kalmanState, float input, float target)
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
  return kalmanState->x;
}


FAST_CODE float dtermkalman_update(float input, int axis)
{
    dtermupdate_kalman_covariance(input, axis);
    input = dtermkalman_process(&dtermkalmanFilterStateRate[axis], input, getSetpointRate(axis) );

    return input;
}
