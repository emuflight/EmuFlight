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

#include <string.h>
#include "arm_math.h"

#include "kalman.h"
#include "fc/rc.h"
#include "build/debug.h"

kalman_t    kalmanFilterStateRate[XYZ_AXIS_COUNT];

static void init_kalman(kalman_t *filter, float q)
{
    memset(filter, 0, sizeof(kalman_t));
    filter->q = q * 0.001f;             //add multiplier to make tuning easier
    filter->r = 88.0f;                  //seeding R at 88.0f
    filter->p = 30.0f;                  //seeding P at 30.0f
    filter->e = 1.0f;
    filter->w = gyroConfig()->imuf_w;
    filter->inverseN = 1.0f/(float)(filter->w);
}


void kalman_init(void)
{
    init_kalman(&kalmanFilterStateRate[X],  gyroConfig()->imuf_roll_q);
    init_kalman(&kalmanFilterStateRate[Y],  gyroConfig()->imuf_pitch_q);
    init_kalman(&kalmanFilterStateRate[Z],  gyroConfig()->imuf_yaw_q);
}

static FAST_CODE void update_kalman_covariance(float gyroRateData, int axis)
{
     kalmanFilterStateRate[axis].axisWindow[ kalmanFilterStateRate[axis].windex] = gyroRateData;
     kalmanFilterStateRate[axis].axisSumMean +=  kalmanFilterStateRate[axis].axisWindow[ kalmanFilterStateRate[axis].windex];
     kalmanFilterStateRate[axis].axisSumVar =  kalmanFilterStateRate[axis].axisSumVar + ( kalmanFilterStateRate[axis].axisWindow[ kalmanFilterStateRate[axis].windex] *  kalmanFilterStateRate[axis].axisWindow[ kalmanFilterStateRate[axis].windex]);
     kalmanFilterStateRate[axis].windex++;
    if ( kalmanFilterStateRate[axis].windex >= kalmanFilterStateRate[axis].w)
    {
         kalmanFilterStateRate[axis].windex = 0;
    }
     kalmanFilterStateRate[axis].axisSumMean -=  kalmanFilterStateRate[axis].axisWindow[ kalmanFilterStateRate[axis].windex];
     kalmanFilterStateRate[axis].axisSumVar =  kalmanFilterStateRate[axis].axisSumVar - ( kalmanFilterStateRate[axis].axisWindow[ kalmanFilterStateRate[axis].windex] *  kalmanFilterStateRate[axis].axisWindow[ kalmanFilterStateRate[axis].windex]);
     kalmanFilterStateRate[axis].axisMean =  kalmanFilterStateRate[axis].axisSumMean *  kalmanFilterStateRate[axis].inverseN;
     kalmanFilterStateRate[axis].axisVar =  fabsf(kalmanFilterStateRate[axis].axisSumVar *  kalmanFilterStateRate[axis].inverseN - ( kalmanFilterStateRate[axis].axisMean *  kalmanFilterStateRate[axis].axisMean));

    kalmanFilterStateRate[axis].r = sqrtf(kalmanFilterStateRate[axis].axisVar) * VARIANCE_SCALE;
}

FAST_CODE float kalman_process(kalman_t* kalmanState, float input, float target)
{
  //project the state ahead using acceleration
  kalmanState->x += (kalmanState->x - kalmanState->lastX);

  //figure out how much to boost or reduce our error in the estimate based on setpoint target.
  //this should be close to 0 as we approach the sepoint and really high the futher away we are from the setpoint.
  //update last state
  kalmanState->lastX = kalmanState->x;

  if (kalmanState->lastX != 0.0f)
  {
      kalmanState->e = fabsf(1.0f - (target / kalmanState->lastX));
  }
  //prediction update
  kalmanState->p = kalmanState->p + (kalmanState->q * kalmanState->e);
  //measurement update
  kalmanState->k = kalmanState->p / (kalmanState->p + kalmanState->r);
  kalmanState->x += kalmanState->k * (input - kalmanState->x);
  kalmanState->p = (1.0f - kalmanState->k) * kalmanState->p;
  return kalmanState->x;
}

FAST_CODE float kalman_update(float input, int axis)
{
 if (gyroConfig()->imuf_w >= 3) {
    update_kalman_covariance(input, axis);
    input = kalman_process(&kalmanFilterStateRate[axis], input, getSetpointRate(axis));
 }
    return input;
}
