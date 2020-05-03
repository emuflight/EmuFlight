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

void init_kalman(kalman_t *filter, float q, float sharpness)
{
    filter->q = q * 0.001f;             //add multiplier to make tuning easier
    filter->r = 88.0f;                  //seeding R at 88.0f
    filter->p = 30.0f;                  //seeding P at 30.0f
    filter->k = 1.0f;
    filter->x = 0.0f;
    filter->lastX = 0.0f;
    filter->e = 1.0f;
    filter->s = sharpness / 250.0f;     //adding the new sharpness :) time to overfilter :O
}

void init_variance(variance_t* variance, float kalmanW)
{
    variance->w = kalmanW;
    variance->inverseN = 1.0f / (float)(variance->w);
    variance->axisVar = 0.0f;
    variance->windex = 0;
    memset(variance->axisWindow, 0, sizeof(variance->axisWindow));
    variance->axisSumMean = 0.0f;
    variance->axisMean = 0.0f;
    variance->axisSumVar = 0.0f;
}

void kalman_init(float xAxis, float yAxis, float zAxis, float sharpness, float kalmanW, kalman_t* kalmanState, variance_t* varStruct)
{
    isSetpointNew = 0;

    init_variance(&varStruct[X], kalmanW);
    init_variance(&varStruct[Y], kalmanW);
    init_variance(&varStruct[Z], kalmanW);
    init_kalman(&kalmanState[X], xAxis, sharpness);
    init_kalman(&kalmanState[Y], yAxis, sharpness);
    init_kalman(&kalmanState[Z], zAxis, sharpness);
}

void update_kalman_covariance(float gyroRateData, kalman_t* kalmanState, variance_t* varStruct)
{
     varStruct->axisWindow[varStruct->windex] = gyroRateData;
     varStruct->axisSumMean += varStruct->axisWindow[varStruct->windex];
     varStruct->axisSumVar = varStruct->axisSumVar + (varStruct->axisWindow[varStruct->windex] * varStruct->axisWindow[varStruct->windex]);
     varStruct->windex++;
    if ( varStruct->windex >= varStruct->w)
    {
         varStruct->windex = 0;
    }
     varStruct->axisSumMean -= varStruct->axisWindow[varStruct->windex];
     varStruct->axisSumVar = varStruct->axisSumVar - (varStruct->axisWindow[varStruct->windex] * varStruct->axisWindow[varStruct->windex]);
     varStruct->axisMean = varStruct->axisSumMean * varStruct->inverseN;
     varStruct->axisVar = fabsf(varStruct->axisSumVar * varStruct->inverseN - (varStruct->axisMean * varStruct->axisMean));

    float squirt;
    arm_sqrt_f32(varStruct->axisVar, &squirt);
    kalmanState->r = squirt * VARIANCE_SCALE;
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
  return kalmanState->x;
}


float FAST_CODE kalman_update(float input, int axis, kalman_t* kalmanState, variance_t* varStruct)
{
    update_kalman_covariance(input, &kalmanState[axis], &varStruct[axis]);
    input = kalman_process(&kalmanState[axis], input, getSetpointRate(axis));

    int16_t Kgain = (kalmanState[axis].k * 1000.0f);
    DEBUG_SET(DEBUG_KALMAN, axis, Kgain);                               //Kalman gain

    return input;
}
