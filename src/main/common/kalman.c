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
#include "build/debug.h"
#include "sensors/gyro.h"

static void init_kalman(kalman_t *filter, float q)
{
    memset(filter, 0, sizeof(kalman_t));
    filter->q = q * 0.0001f;             //add multiplier to make tuning easier
    filter->r = 88.0f;                  //seeding R at 88.0f
    filter->p = 30.0f;                  //seeding P at 30.0f
    filter->e = 1.0f;
    filter->w = gyroConfig()->imuf_w;
    filter->inverseN = 1.0f/(float)(filter->w);

    pt1FilterInit(&filter->kFilter, pt1FilterGain(50, gyro.sampleLooptime * 1e-6f));
}


void kalman_init(void)
{
    init_kalman(&gyro.kalmanFilterStateRate[X],  gyroConfig()->imuf_q);
    init_kalman(&gyro.kalmanFilterStateRate[Y],  gyroConfig()->imuf_q);
    init_kalman(&gyro.kalmanFilterStateRate[Z],  gyroConfig()->imuf_q);
}

void update_kalman_covariance(kalman_t *kalmanState, float rate) {
  if (gyroConfig()->imuf_w >= 3) {
    kalmanState->axisWindow[kalmanState->windex] = rate;
    kalmanState->axisSumMean += kalmanState->axisWindow[kalmanState->windex];

    float varianceElement = kalmanState->axisWindow[kalmanState->windex] - kalmanState->axisMean;
    varianceElement = varianceElement * varianceElement;
    kalmanState->axisSumVar += varianceElement;
    kalmanState->varianceWindow[kalmanState->windex] = varianceElement;
    kalmanState->windex++;

    if (kalmanState->windex > kalmanState->w) {
        kalmanState->windex = 0;
    }

    kalmanState->axisSumMean -= kalmanState->axisWindow[kalmanState->windex];
    kalmanState->axisSumVar -= kalmanState->varianceWindow[kalmanState->windex];

    //New mean
    kalmanState->axisMean = kalmanState->axisSumMean * kalmanState->inverseN;
    kalmanState->axisVar = kalmanState->axisSumVar * kalmanState->inverseN;

    float squirt;
    arm_sqrt_f32(kalmanState->axisVar, &squirt);
    kalmanState->r = squirt * VARIANCE_SCALE;
  }
}

FAST_CODE float kalman_process(kalman_t* kalmanState, float input)
{
  //project the state ahead using acceleration
  kalmanState->x += (kalmanState->x - kalmanState->lastX) * kalmanState->k;

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

FAST_CODE float kalman_update(float input, int axis)
{
 if (gyroConfig()->imuf_w >= 3) {
    input = kalman_process(&gyro.kalmanFilterStateRate[axis], input);
 }
    return input;
}
