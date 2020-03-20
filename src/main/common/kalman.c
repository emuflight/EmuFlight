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
#include "fc/fc_rc.h"
#include "build/debug.h"

kalman_t    kalmanFilterStateRate[XYZ_AXIS_COUNT];
variance_t  varStruct;
float       setPoint[XYZ_AXIS_COUNT];


void init_kalman(kalman_t *filter, float q)
{
    memset(filter, 0, sizeof(kalman_t));
    filter->q = q * 0.001f;             //add multiplier to make tuning easier
    filter->r = 88.0f;                  //seeding R at 88.0f
    filter->p = 30.0f;                  //seeding P at 30.0f
    filter->e = 1.0f;
    filter->s = gyroConfig()->imuf_sharpness / 250.0f;     //adding the new sharpness :) time to overfilter :O
}


void kalman_init(void)
{
    isSetpointNew = 0;

    memset(&varStruct, 0, sizeof(varStruct));
    init_kalman(&kalmanFilterStateRate[X],  gyroConfig()->imuf_roll_q);
    init_kalman(&kalmanFilterStateRate[Y],  gyroConfig()->imuf_pitch_q);
    init_kalman(&kalmanFilterStateRate[Z],  gyroConfig()->imuf_yaw_q);

    varStruct.w = gyroConfig()->imuf_w;
    varStruct.inverseN = 1.0f/(float)(varStruct.w);
}

void update_kalman_covariance(float *gyroRateData)
{
     varStruct.xWindow[ varStruct.windex] = gyroRateData[X];
     varStruct.yWindow[ varStruct.windex] = gyroRateData[Y];
     varStruct.zWindow[ varStruct.windex] = gyroRateData[Z];

     varStruct.xSumMean +=  varStruct.xWindow[ varStruct.windex];
     varStruct.ySumMean +=  varStruct.yWindow[ varStruct.windex];
     varStruct.zSumMean +=  varStruct.zWindow[ varStruct.windex];
     varStruct.xSumVar =  varStruct.xSumVar + ( varStruct.xWindow[ varStruct.windex] *  varStruct.xWindow[ varStruct.windex]);
     varStruct.ySumVar =  varStruct.ySumVar + ( varStruct.yWindow[ varStruct.windex] *  varStruct.yWindow[ varStruct.windex]);
     varStruct.zSumVar =  varStruct.zSumVar + ( varStruct.zWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);
     varStruct.xySumCoVar =  varStruct.xySumCoVar + ( varStruct.xWindow[ varStruct.windex] *  varStruct.yWindow[ varStruct.windex]);
     varStruct.xzSumCoVar =  varStruct.xzSumCoVar + ( varStruct.xWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);
     varStruct.yzSumCoVar =  varStruct.yzSumCoVar + ( varStruct.yWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);
     varStruct.windex++;
    if ( varStruct.windex >= varStruct.w)
    {
         varStruct.windex = 0;
    }
     varStruct.xSumMean -=  varStruct.xWindow[ varStruct.windex];
     varStruct.ySumMean -=  varStruct.yWindow[ varStruct.windex];
     varStruct.zSumMean -=  varStruct.zWindow[ varStruct.windex];
     varStruct.xSumVar =  varStruct.xSumVar - ( varStruct.xWindow[ varStruct.windex] *  varStruct.xWindow[ varStruct.windex]);
     varStruct.ySumVar =  varStruct.ySumVar - ( varStruct.yWindow[ varStruct.windex] *  varStruct.yWindow[ varStruct.windex]);
     varStruct.zSumVar =  varStruct.zSumVar - ( varStruct.zWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);
     varStruct.xySumCoVar =  varStruct.xySumCoVar - ( varStruct.xWindow[ varStruct.windex] *  varStruct.yWindow[ varStruct.windex]);
     varStruct.xzSumCoVar =  varStruct.xzSumCoVar - ( varStruct.xWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);
     varStruct.yzSumCoVar =  varStruct.yzSumCoVar - ( varStruct.yWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);

     varStruct.xMean =  varStruct.xSumMean *  varStruct.inverseN;
     varStruct.yMean =  varStruct.ySumMean *  varStruct.inverseN;
     varStruct.zMean =  varStruct.zSumMean *  varStruct.inverseN;

     varStruct.xVar =  fabsf(varStruct.xSumVar *  varStruct.inverseN - ( varStruct.xMean *  varStruct.xMean));
     varStruct.yVar =  fabsf(varStruct.ySumVar *  varStruct.inverseN - ( varStruct.yMean *  varStruct.yMean));
     varStruct.zVar =  fabsf(varStruct.zSumVar *  varStruct.inverseN - ( varStruct.zMean *  varStruct.zMean));
     varStruct.xyCoVar =  fabsf(varStruct.xySumCoVar *  varStruct.inverseN - ( varStruct.xMean *  varStruct.yMean));
     varStruct.xzCoVar =  fabsf(varStruct.xzSumCoVar *  varStruct.inverseN - ( varStruct.xMean *  varStruct.zMean));
     varStruct.yzCoVar =  fabsf(varStruct.yzSumCoVar *  varStruct.inverseN - ( varStruct.yMean *  varStruct.zMean));

    float squirt;
    arm_sqrt_f32(varStruct.xVar +  varStruct.xyCoVar +  varStruct.xzCoVar, &squirt);
    kalmanFilterStateRate[X].r = squirt * VARIANCE_SCALE;

    arm_sqrt_f32(varStruct.yVar +  varStruct.xyCoVar +  varStruct.yzCoVar, &squirt);
    kalmanFilterStateRate[Y].r = squirt * VARIANCE_SCALE;

    arm_sqrt_f32(varStruct.zVar +  varStruct.yzCoVar +  varStruct.xzCoVar, &squirt);
    kalmanFilterStateRate[Z].r = squirt * VARIANCE_SCALE;
}

FAST_CODE float kalman_process(kalman_t* kalmanState, float input, float target)
{
  //project the state ahead using acceleration
  kalmanState->x += (kalmanState->x - kalmanState->lastX);

  //figure out how much to boost or reduce our error in the estimate based on setpoint target.
  //this should be close to 0 as we approach the sepoint and really high the futher away we are from the setpoint.
  //update last state
  kalmanState->lastX = kalmanState->x;

  // calculate the error
  	float errorMultiplier = fabsf(target - kalmanState->x) * kalmanState->s;

  // give a boost to the setpoint, used to caluclate the filter cutoff, based on the error and setpoint/gyrodata

  	errorMultiplier = constrainf(errorMultiplier * fabsf(1.0f - (target / kalmanState->lastX)) + 1.0f, 1.0f, 50.0f);

  if (target != 0.0f) {
      kalmanState->e = fabsf(1.0f - ((target * errorMultiplier) / kalmanState->lastX));
  } else {
      kalmanState->e = 1.0f;
  }

  //kalmanState->e = ABS((target - input) * 3) + ABS(input/4);

  //prediction update
  kalmanState->p = kalmanState->p + (kalmanState->q * kalmanState->e);

  //measurement update
  kalmanState->k = kalmanState->p / (kalmanState->p + kalmanState->r);
  kalmanState->x += kalmanState->k * (input - kalmanState->x);
  kalmanState->p = (1.0f - kalmanState->k) * kalmanState->p;
  return kalmanState->x;
}


void FAST_CODE kalman_update(float* input, float* output)
{
    update_kalman_covariance(input);
    output[X] = kalman_process(&kalmanFilterStateRate[X], input[X], setPoint[X] );
    output[Y] = kalman_process(&kalmanFilterStateRate[Y], input[Y], setPoint[Y] );
    output[Z] = kalman_process(&kalmanFilterStateRate[Z], input[Z], setPoint[Z] );

    DEBUG_SET(DEBUG_KALMAN, 1, input[X]);                               //Gyro input

    int16_t Kgain = (kalmanFilterStateRate[X].k * 1000.0f);
    DEBUG_SET(DEBUG_KALMAN, 2, Kgain);                                  //Kalman gain
    DEBUG_SET(DEBUG_KALMAN, 3, output[X]);                              //Kalman output
}