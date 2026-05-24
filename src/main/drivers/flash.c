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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "build/debug.h"

#ifdef USE_FLASH

#include "flash.h"
#include "flash_impl.h"
#include "flash_m25p16.h"
#include "flash_w25m.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"

#ifdef USE_FLASH_W25Q128FV
#include "flash_w25q128fv.h"
#include "drivers/bus_quadspi.h"
#endif

static extDevice_t busInstance;
static extDevice_t *dev;

static flashDevice_t flashDevice;

// Read chip identification and send it to device detect

#if defined(USE_FLASH_W25Q128FV) && defined(USE_QUADSPI)
static extDevice_t qspiFlashDevice;

static bool flashInitQuadSpi(void)
{
    extDevice_t *qdev = &qspiFlashDevice;

    // QUADSPI_FLASH_DEVICE is the QUADSPI device index configured for the target.
    // If undefined, default to device 1 (index 0).
#ifndef QUADSPI_FLASH_DEVICE
#define QUADSPI_FLASH_DEVICE QUADSPIDEV_1
#endif

    if (!quadSpiSetBusInstance(qdev, QUADSPI_FLASH_DEVICE)) {
        return false;
    }

    quadSpiSetDivisor(qdev, QUADSPI_CLOCK_INITIALISATION);

    uint8_t in[3] = { 0 };
    // RDID (0x9F) via 1LINE: instruction only, receive 3 data bytes
    if (!quadSpiReceive1LINE(qdev, SPIFLASH_INSTRUCTION_RDID, 0, in, sizeof(in))) {
        return false;
    }

    uint32_t chipID = ((uint32_t)in[0] << 16) | ((uint32_t)in[1] << 8) | (uint32_t)in[2];

    flashDevice.io.handle.qspiDev = qdev;

    if (w25q128fv_detect(&flashDevice, chipID)) {
        quadSpiSetDivisor(qdev, QUADSPI_CLOCK_FAST);
        return true;
    }

    return false;
}
#endif // USE_FLASH_W25Q128FV && USE_QUADSPI

bool flashInit(const flashConfig_t *flashConfig) {
#if defined(USE_FLASH_W25Q128FV) && defined(USE_QUADSPI)
    // Attempt QuadSPI-attached flash first when QUADSPI is available.
    // SPI flash config (csTag) may be absent on pure-QuadSPI boards.
    if (flashInitQuadSpi()) {
        return true;
    }
#endif

    dev = &busInstance;
    if (flashConfig->csTag) {
        dev->busType_u.spi.csnPin = IOGetByTag(flashConfig->csTag);
    } else {
        return false;
    }
    if (!IOIsFreeOrPreinit(dev->busType_u.spi.csnPin)) {
        return false;
    }
    if (!spiSetBusInstance(dev, flashConfig->spiDevice)) {
        return false;
    }
    IOInit(dev->busType_u.spi.csnPin, OWNER_FLASH_CS, 0);
    IOConfigGPIO(dev->busType_u.spi.csnPin, SPI_IO_CS_CFG);
    IOHi(dev->busType_u.spi.csnPin);
#ifndef FLASH_SPI_SHARED
    //Maximum speed for standard READ command is 20mHz, other commands tolerate 25mHz
    //spiSetDivisor(dev->bus->busType_u.spi.instance, SPI_CLOCK_FAST);
    spiSetDivisor(dev->bus->busType_u.spi.instance, SPI_CLOCK_STANDARD * 2);
#endif
    flashDevice.dev = dev;
    const uint8_t out[] = { SPIFLASH_INSTRUCTION_RDID, 0, 0, 0 };
    delay(50); // short delay required after initialisation of SPI device instance.
    /* Just in case transfer fails and writes nothing, so we don't try to verify the ID against random garbage
     * from the stack:
     */
    uint8_t in[4] = { 0 };
    // Clearing the CS bit terminates the command early so we don't have to read the chip UID:
    spiReadWriteBuf(dev, (uint8_t *)out, in, sizeof(out));
    // Manufacturer, memory type, and capacity
    uint32_t chipID = (in[1] << 16) | (in[2] << 8) | (in[3]);
#ifdef USE_FLASH_M25P16
    if (m25p16_detect(&flashDevice, chipID)) {
        return true;
    }
#endif
#ifdef USE_FLASH_W25M
    if (w25m_detect(&flashDevice, chipID)) {
        return true;
    }
#endif
    spiPreinitCsByTag(flashConfig->csTag);
    return false;
}

bool flashIsReady(void) {
    return flashDevice.vTable->isReady(&flashDevice);
}

bool flashWaitForReady(uint32_t timeoutMillis) {
    return flashDevice.vTable->waitForReady(&flashDevice, timeoutMillis);
}

void flashEraseSector(uint32_t address) {
    flashDevice.vTable->eraseSector(&flashDevice, address);
}

void flashEraseCompletely(void) {
    flashDevice.vTable->eraseCompletely(&flashDevice);
}

void flashPageProgramBegin(uint32_t address) {
    flashDevice.vTable->pageProgramBegin(&flashDevice, address);
}

void flashPageProgramContinue(const uint8_t *data, int length) {
    flashDevice.vTable->pageProgramContinue(&flashDevice, data, length);
}

void flashPageProgramFinish(void) {
    flashDevice.vTable->pageProgramFinish(&flashDevice);
}

void flashPageProgram(uint32_t address, const uint8_t *data, int length) {
    flashDevice.vTable->pageProgram(&flashDevice, address, data, length);
}

int flashReadBytes(uint32_t address, uint8_t *buffer, int length) {
    return flashDevice.vTable->readBytes(&flashDevice, address, buffer, length);
}

void flashFlush(void) {
    flashDevice.vTable->flush(&flashDevice);
}

static const flashGeometry_t noFlashGeometry = {
    .totalSize = 0,
};

const flashGeometry_t *flashGetGeometry(void) {
    if (flashDevice.vTable && flashDevice.vTable->getGeometry) {
        return flashDevice.vTable->getGeometry(&flashDevice);
    }
    return &noFlashGeometry;
}
#endif // USE_FLASH
