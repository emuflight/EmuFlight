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

#include "auto_notch.h"
#include "fc/rc.h"
#include "build/debug.h"
#include "sensors/gyro.h"

/*
 * The idea is simple, run a passband at the notch frequency to isolate noise
 * at the notch frequency. Then look at the averaged squared rate of change over
 * a period of time great enough to cover at least 1 full wave of noise at notch
 * frequency. This way we can measure noise and compare noise of pre/post notch.
 * this allows us to crossfade based on noise.
 */

/*
void init(auto_notch_t *auto_notch, float frequency, int gain) {
  for (int i = 0; i < SAMPLE_LENGTH - 1; i++) {
    float currentX = 1 * (i + 1.0);
    filter->sumX += currentX;
    filter->sumXSquared += currentX * currentX;
  }
  filter->sumXSquared = filter->sumXSquared * filter->w;
  filter->squaredSumX = filter->sumX * filter->sumX;
}

void update_kalman_variance(kalman_t *state, float input) {
  // put new data in the circular buffer
  state->axisWindow[state->windex] = input;
  state->windex++;

  if (state->windex > state->w) {
    state->windex = 0;
  }

  // least squares regression line
  float sumXY = 0.0f;
  float sumY = 0.0f;
  int tempPointer = state->windex;
  for (int i = 0; i < state->w; i++) {
    float currentX = 1 * (i + 1.0);
    sumXY += currentX * state->axisWindow[tempPointer];
    sumY += state->axisWindow[tempPointer];
    tempPointer++;
    if (tempPointer > state->w) {
      tempPointer = 0;
    }
  }

  tempPointer = state->windex;
  float m = (state->w * sumXY - state->sumX * sumY) * 2.9301453352086263478668541959681e-7f;
  float b = (sumY - m * state->sumX) * 0.0125f;

  // calculate variance against the curve
  float variance = 0.0f;
  for (int i = 0; i < state->w; i++) {
    float currentX = 1 * (i + 1.0);
    float pointOnLine = m * currentX + b;
    float error = state->axisWindow[tempPointer] - pointOnLine;
    variance += error * error;

    tempPointer++;
      if (tempPointer > state->w) {
        tempPointer = 0;
      }
  }
    float squirt;
    arm_sqrt_f32(variance, &squirt);
    state->variance = squirt * VARIANCE_SCALE;
    state->variance = pt1FilterApply(&state->kFilter, state->variance);
}
*/
