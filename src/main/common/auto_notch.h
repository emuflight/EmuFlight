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

#pragma once

#include "filter.h"

// 8k / 100 gives us ability to see down to 50hz noise
#define SAMPLE_LENGTH (8000 / 100) + 1
// SAMPLE_LENGTH = 81

// code below is precomputed to find the value of sum, sum squared, and squared sum
// will need to be recomputed for different SAMPLE_LENGTH values
/*int main()
{
    int sum = 0;
    int sum_squared = 0;
    int squared_sum = 0;
    for (int i = 0; i < SAMPLE_LENGTH - 1; i++) {
        int current_value = 1.0 * (i + 1.0);
        sum += current_value;
        sum_squared += current_value * current_value;
    }
    sum_squared = sum_squared * (SAMPLE_LENGTH - 1);
    squared_sum = sum * sum;

    printf("Hello, World!\n");
    printf("sum %d\n", sum);
    printf("sum squared %d\n", sum_squared);
    printf("squared sum %d\n", squared_sum);
}*/

#define SUM 3240
#define SUM_SQUARED 13910400
#define SQUARED_SUM 10497600


typedef struct auto_notch_s {

} auto_notch_t;
