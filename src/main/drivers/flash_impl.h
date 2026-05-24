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
 * Author: jflyper
 */

#pragma once

#include "drivers/bus.h"

#ifdef USE_QUADSPI
#include "drivers/bus_quadspi.h"
#endif

struct flashVTable_s;

// IO handle union: SPI uses extDevice_t*, QuadSPI uses extDevice_t* via bus_quadspi.
// The qspiDev pointer is a separate extDevice registered on the QSPI bus, distinct
// from the SPI extDevice used by m25p16/w25m.
typedef struct flashDeviceIO_s {
    union {
        extDevice_t *dev;   // SPI path: csnPin + spiInstance via spiSetBusInstance
#ifdef USE_QUADSPI
        extDevice_t *qspiDev; // QuadSPI path: registered via quadSpiSetBusInstance
#endif
    } handle;
} flashDeviceIO_t;

typedef struct flashDevice_s {
    // SPI path: legacy direct extDevice_t pointer (used by m25p16, w25m)
    extDevice_t *dev;

    const struct flashVTable_s *vTable;
    flashGeometry_t geometry;
    uint32_t currentWriteAddress;
    bool isLargeFlash;
    // Whether we've performed an action that could have made the device busy
    // for writes. This allows us to avoid polling for writable status
    // when it is definitely ready already.
    bool couldBeBusy;

    // QuadSPI path: io union populated by w25q128fv_detect(); NULL for SPI devices.
    flashDeviceIO_t io;

    // Timeout tracking for QuadSPI driver (milliseconds)
    uint32_t timeoutAt;
} flashDevice_t;

typedef struct flashVTable_s {
    bool (*isReady)(flashDevice_t *fdevice);
    bool (*waitForReady)(flashDevice_t *fdevice, uint32_t timeoutMillis);
    void (*eraseSector)(flashDevice_t *fdevice, uint32_t address);
    void (*eraseCompletely)(flashDevice_t *fdevice);
    void (*pageProgramBegin)(flashDevice_t *fdevice, uint32_t address);
    void (*pageProgramContinue)(flashDevice_t *fdevice, const uint8_t *data, int length);
    void (*pageProgramFinish)(flashDevice_t *fdevice);
    void (*pageProgram)(flashDevice_t *fdevice, uint32_t address, const uint8_t *data, int length);
    void (*flush)(flashDevice_t *fdevice);
    int (*readBytes)(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, int length);
    const flashGeometry_t *(*getGeometry)(flashDevice_t *fdevice);
} flashVTable_t;
