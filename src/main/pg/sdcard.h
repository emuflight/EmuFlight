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

#include "pg/pg.h"
#include "drivers/io.h"

typedef struct sdcardConfig_s {
    uint8_t enabled;
    uint8_t device;
    ioTag_t cardDetectTag;
    ioTag_t chipSelectTag;
    uint8_t cardDetectInverted;
    // SDIO mode only (drivers/sdcard_sdio_baremetal.c) -- the SPI-mode driver resolves its own
    // DMA at the bus level via spiInitBusDMA(), no config-level identifier needed.
    uint8_t dmaIdentifier;
} sdcardConfig_t;

PG_DECLARE(sdcardConfig_t, sdcardConfig);
