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

/*
 * gyro_sync.c
 *
 *  Created on: 3 aug. 2015
 *      Author: borisb
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/gyro_sync.h"


bool gyroSyncCheckUpdate(gyroDev_t *gyro) {
    bool ret;
    if (gyro->dataReady) {
        ret = true;
        gyro->dataReady = false;
    } else {
        ret = false;
    }
    return ret;
}

uint32_t gyroSetSampleRate(gyroDev_t *gyro, uint8_t lpf, uint8_t gyroSyncDenominator, bool gyro_use_32khz) {
    bool lpfNoneOr256 = (lpf == GYRO_LPF_256HZ || lpf == GYRO_LPF_NONE);
    if (!lpfNoneOr256) {
        gyroSyncDenominator = 1; // Always full Sampling
    }
    gyro->mpuDividerDrops = gyroSyncDenominator - 1;
    gyro->gyroRateKHz = lpfNoneOr256 ? GYRO_RATE_8_kHz : GYRO_RATE_1_kHz;
    //20649 is a weird gyro
    if (gyro->mpuDetectionResult.sensor == ICM_20649_SPI) {
        gyro->gyroRateKHz = lpfNoneOr256 ? GYRO_RATE_9_kHz : GYRO_RATE_1100_Hz;
    } else if (gyro->mpuDetectionResult.sensor == BMI_160_SPI && lpfNoneOr256) {
        //brainFPV is also a weird gyro
        gyro->gyroRateKHz = GYRO_RATE_3200_Hz;
    } else if (gyro_use_32khz) {
        //use full 32k
        gyro->gyroRateKHz = GYRO_RATE_32_kHz;
    }
    // return the targetLooptime (expected cycleTime)
    return (uint32_t)(gyroSyncDenominator * gyro->gyroRateKHz);
}
