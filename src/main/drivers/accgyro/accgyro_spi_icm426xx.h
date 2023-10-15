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

#pragma once

#include "drivers/bus.h"

#define ICM426xx_BIT_RESET                  (0x80)

bool icm426xxAccDetect(accDev_t *acc);
bool icm426xxGyroDetect(gyroDev_t *gyro);

void icm426xxAccInit(accDev_t *acc);
void icm426xxGyroInit(gyroDev_t *gyro);

uint8_t icm426xxSpiDetect(const busDevice_t *dev);

bool icm426xxSpiAccDetect(accDev_t *acc);
bool icm426xxSpiGyroDetect(gyroDev_t *gyro);