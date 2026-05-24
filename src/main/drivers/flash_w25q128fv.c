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
 *
 * Author: Dominic Clifton - Initial BF implementation.
 * EmuFlight port: adapted vtable signatures and synchronous QuadSPI API.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_FLASH_W25Q128FV) && defined(USE_QUADSPI)

#define USE_FLASH_WRITES_USING_4LINES
#define USE_FLASH_READS_USING_4LINES

#include "build/debug.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/flash.h"
#include "drivers/flash_impl.h"
#include "drivers/flash_w25q128fv.h"
#include "drivers/bus_quadspi.h"

// JEDEC IDs
#define JEDEC_ID_WINBOND_W25Q128FV_SPI          0xEF4018
#define JEDEC_ID_WINBOND_W25Q128FV_QUADSPI      0xEF6018
#define JEDEC_ID_WINBOND_W25Q128JV_QUADSPI      0xEF7018
#define JEDEC_ID_WINBOND_W25Q16JV_SPI           0xEF4015
#define JEDEC_ID_WINBOND_W25Q16JV_DTR_SPI       0xEF7015
#define JEDEC_ID_BOYAMICRO_BY25Q128ES_SPI       0x684018

// Device size parameters
#define W25Q128FV_PAGE_SIZE         2048
#define W25Q128FV_PAGES_PER_BLOCK   64
#define W25Q128FV_BLOCKS_PER_DIE    1024
#define W25Q128FV_BLOCK_SIZE        (W25Q128FV_PAGES_PER_BLOCK * W25Q128FV_PAGE_SIZE)

// Sizes
#define W25Q128FV_STATUS_REGISTER_BITS         8
#define W25Q128FV_ADDRESS_BITS                 24

// Instructions
#define W25Q128FV_INSTRUCTION_RDID                   0x9F
#define W25Q128FV_INSTRUCTION_ENABLE_RESET           0x66
#define W25Q128FV_INSTRUCTION_RESET_DEVICE           0x99
#define W25Q128FV_INSTRUCTION_READ_STATUS1_REG       0x05
#define W25Q128FV_INSTRUCTION_READ_STATUS2_REG       0x35
#define W25Q128FV_INSTRUCTION_READ_STATUS3_REG       0x15
#define W25Q128FV_INSTRUCTION_WRITE_STATUS1_REG      0x01
#define W25Q128FV_INSTRUCTION_WRITE_STATUS2_REG      0x31
#define W25Q128FV_INSTRUCTION_WRITE_STATUS3_REG      0x11
#define W25Q128FV_INSTRUCTION_WRITE_ENABLE           0x06
#define W25Q128FV_INSTRUCTION_VOLATILE_WRITE_ENABLE  0x50
#define W25Q128FV_INSTRUCTION_BLOCK_ERASE_64KB       0xD8
#define W25Q128FV_INSTRUCTION_CHIP_ERASE             0xC7
#define W25Q128FV_INSTRUCTION_ENTER_QPI_MODE         0x38
#define W25Q128FV_INSTRUCTION_READ_BYTES             0x03
#define W25Q128FV_INSTRUCTION_FAST_READ              0x0B
#define W25Q128FV_INSTRUCTION_FAST_READ_QUAD_OUTPUT  0x6B
#define W25Q128FV_INSTRUCTION_PAGE_PROGRAM           0x02
#define W25Q128FV_INSTRUCTION_QUAD_PAGE_PROGRAM      0x32

#define W25Q128FV_SR1_BIT_WRITE_IN_PROGRESS     (1 << 0)
#define W25Q128FV_SR1_BIT_WRITE_ENABLED         (1 << 1)
#define W25Q128FV_SR2_BIT_QUAD_ENABLE           (1 << 1)

// Values from W25Q128FV Datasheet Rev L.
#define W25Q128FV_TIMEOUT_PAGE_READ_MS          1
#define W25Q128FV_TIMEOUT_RESET_MS              1           // tRST = 30us
#define W25Q128FV_TIMEOUT_BLOCK_ERASE_64KB_MS   2000        // tBE2max = 2000ms
#define W25Q128FV_TIMEOUT_CHIP_ERASE_MS         (200 * 1000) // tCEmax 200s
#define W25Q128FV_TIMEOUT_PAGE_PROGRAM_MS       3           // tPPmax = 3ms
#define W25Q128FV_TIMEOUT_WRITE_ENABLE_MS       1

static bool w25q128fv_waitForReady(flashDevice_t *fdevice, uint32_t timeoutMillis);

MMFLASH_CODE static void w25q128fv_setTimeout(flashDevice_t *fdevice, uint32_t timeoutMillis)
{
    fdevice->timeoutAt = millis() + timeoutMillis;
}

// Shorthand: the QuadSPI extDevice_t is stored in io.handle.qspiDev
#define QSPI_DEV(fdevice) ((fdevice)->io.handle.qspiDev)

MMFLASH_CODE static void w25q128fv_performOneByteCommand(flashDevice_t *fdevice, uint8_t command)
{
    quadSpiTransmit1LINE(QSPI_DEV(fdevice), command, 0, NULL, 0);
}

MMFLASH_CODE static void w25q128fv_performCommandWithAddress(flashDevice_t *fdevice, uint8_t command, uint32_t address)
{
    quadSpiInstructionWithAddress1LINE(QSPI_DEV(fdevice), command, 0, address & 0xFFFFFF, W25Q128FV_ADDRESS_BITS);
}

MMFLASH_CODE static void w25q128fv_writeEnable(flashDevice_t *fdevice)
{
    w25q128fv_performOneByteCommand(fdevice, W25Q128FV_INSTRUCTION_WRITE_ENABLE);
}

MMFLASH_CODE static uint8_t w25q128fv_readRegister(flashDevice_t *fdevice, uint8_t command)
{
    uint8_t in[W25Q128FV_STATUS_REGISTER_BITS / 8] = { 0 };
    quadSpiReceive1LINE(QSPI_DEV(fdevice), command, 0, in, W25Q128FV_STATUS_REGISTER_BITS / 8);
    return in[0];
}

static void w25q128fv_writeRegister(flashDevice_t *fdevice, uint8_t command, uint8_t data)
{
    quadSpiTransmit1LINE(QSPI_DEV(fdevice), command, 0, &data, W25Q128FV_STATUS_REGISTER_BITS / 8);
}

static void w25q128fv_deviceReset(flashDevice_t *fdevice)
{
    w25q128fv_waitForReady(fdevice, W25Q128FV_TIMEOUT_RESET_MS);
    w25q128fv_performOneByteCommand(fdevice, W25Q128FV_INSTRUCTION_ENABLE_RESET);
    w25q128fv_performOneByteCommand(fdevice, W25Q128FV_INSTRUCTION_RESET_DEVICE);

    w25q128fv_setTimeout(fdevice, W25Q128FV_TIMEOUT_RESET_MS);
    while (millis() < fdevice->timeoutAt) { }
    fdevice->timeoutAt = 0;

    w25q128fv_waitForReady(fdevice, W25Q128FV_TIMEOUT_RESET_MS);

#if defined(USE_FLASH_WRITES_USING_4LINES) || defined(USE_FLASH_READS_USING_4LINES)
    uint8_t registerValue = w25q128fv_readRegister(fdevice, W25Q128FV_INSTRUCTION_READ_STATUS2_REG);

    // WARNING: DO NOT ENABLE QE bit if IO2/IO3 are connected to GND or VCC.
    // See datasheet W25Q128FV Rev M section 7.1.10 Quad Enable.
    if ((registerValue & W25Q128FV_SR2_BIT_QUAD_ENABLE) == 0) {
        registerValue = w25q128fv_readRegister(fdevice, W25Q128FV_INSTRUCTION_READ_STATUS2_REG);

        uint8_t newValue = registerValue | W25Q128FV_SR2_BIT_QUAD_ENABLE;

        // Volatile write enable avoids wearing the non-volatile status register.
        w25q128fv_performOneByteCommand(fdevice, W25Q128FV_INSTRUCTION_VOLATILE_WRITE_ENABLE);
        w25q128fv_writeRegister(fdevice, W25Q128FV_INSTRUCTION_WRITE_STATUS2_REG, newValue);
    }
#endif
}

MMFLASH_CODE static bool w25q128fv_isReady(flashDevice_t *fdevice)
{
    uint8_t status = w25q128fv_readRegister(fdevice, W25Q128FV_INSTRUCTION_READ_STATUS1_REG);
    return (status & W25Q128FV_SR1_BIT_WRITE_IN_PROGRESS) == 0;
}

MMFLASH_CODE static bool w25q128fv_waitForReady(flashDevice_t *fdevice, uint32_t timeoutMillis)
{
    uint32_t deadline = millis() + timeoutMillis;
    while (!w25q128fv_isReady(fdevice)) {
        if (millis() > deadline) {
            return false;
        }
    }
    fdevice->timeoutAt = 0;
    return true;
}

const flashVTable_t w25q128fv_vTable;

static void w25q128fv_deviceInit(flashDevice_t *flashdev)
{
    UNUSED(flashdev);
}

MMFLASH_CODE_NOINLINE bool w25q128fv_detect(flashDevice_t *fdevice, uint32_t jedecID)
{
    switch (jedecID) {
    case JEDEC_ID_WINBOND_W25Q128FV_SPI:
    case JEDEC_ID_WINBOND_W25Q128FV_QUADSPI:
    case JEDEC_ID_WINBOND_W25Q128JV_QUADSPI:
    case JEDEC_ID_BOYAMICRO_BY25Q128ES_SPI:
        fdevice->geometry.sectors           = 256;
        fdevice->geometry.pagesPerSector    = 256;
        fdevice->geometry.pageSize          = 256;
        // 16777216 bytes = 128Mbit = 16MB
        break;

    case JEDEC_ID_WINBOND_W25Q16JV_DTR_SPI:
    case JEDEC_ID_WINBOND_W25Q16JV_SPI:
        fdevice->geometry.sectors           = 32;
        fdevice->geometry.pagesPerSector    = 256;
        fdevice->geometry.pageSize          = 256;
        // 2097152 bytes = 16Mbit = 2MB
        break;

    default:
        fdevice->geometry.sectors       = 0;
        fdevice->geometry.pagesPerSector = 0;
        fdevice->geometry.sectorSize    = 0;
        fdevice->geometry.totalSize     = 0;
        return false;
    }

    fdevice->geometry.flashType = FLASH_TYPE_NOR;
    fdevice->geometry.sectorSize = fdevice->geometry.pagesPerSector * fdevice->geometry.pageSize;
    fdevice->geometry.totalSize  = fdevice->geometry.sectorSize * fdevice->geometry.sectors;

    fdevice->couldBeBusy = true;
    fdevice->vTable = &w25q128fv_vTable;

    w25q128fv_deviceReset(fdevice);
    w25q128fv_deviceInit(fdevice);

    return true;
}

MMFLASH_CODE static void w25q128fv_eraseSector(flashDevice_t *fdevice, uint32_t address)
{
    w25q128fv_waitForReady(fdevice, W25Q128FV_TIMEOUT_BLOCK_ERASE_64KB_MS);
    w25q128fv_writeEnable(fdevice);
    w25q128fv_performCommandWithAddress(fdevice, W25Q128FV_INSTRUCTION_BLOCK_ERASE_64KB, address);
    w25q128fv_setTimeout(fdevice, W25Q128FV_TIMEOUT_BLOCK_ERASE_64KB_MS);
}

static void w25q128fv_eraseCompletely(flashDevice_t *fdevice)
{
    w25q128fv_waitForReady(fdevice, W25Q128FV_TIMEOUT_CHIP_ERASE_MS);
    w25q128fv_writeEnable(fdevice);
    w25q128fv_performOneByteCommand(fdevice, W25Q128FV_INSTRUCTION_CHIP_ERASE);
    w25q128fv_setTimeout(fdevice, W25Q128FV_TIMEOUT_CHIP_ERASE_MS);
}

MMFLASH_CODE static void w25q128fv_pageProgramBegin(flashDevice_t *fdevice, uint32_t address)
{
    fdevice->currentWriteAddress = address;
}

MMFLASH_CODE static void w25q128fv_pageProgramContinue(flashDevice_t *fdevice, const uint8_t *data, int length)
{
    w25q128fv_waitForReady(fdevice, W25Q128FV_TIMEOUT_PAGE_PROGRAM_MS);
    w25q128fv_writeEnable(fdevice);

#ifdef USE_FLASH_WRITES_USING_4LINES
    quadSpiTransmitWithAddress4LINES(QSPI_DEV(fdevice), W25Q128FV_INSTRUCTION_QUAD_PAGE_PROGRAM, 0,
                                     fdevice->currentWriteAddress, W25Q128FV_ADDRESS_BITS, data, length);
#else
    quadSpiTransmitWithAddress1LINE(QSPI_DEV(fdevice), W25Q128FV_INSTRUCTION_PAGE_PROGRAM, 0,
                                    fdevice->currentWriteAddress, W25Q128FV_ADDRESS_BITS, data, length);
#endif

    w25q128fv_setTimeout(fdevice, W25Q128FV_TIMEOUT_PAGE_PROGRAM_MS);
    fdevice->currentWriteAddress += length;
}

MMFLASH_CODE static void w25q128fv_pageProgramFinish(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
}

MMFLASH_CODE static void w25q128fv_pageProgram(flashDevice_t *fdevice, uint32_t address, const uint8_t *data, int length)
{
    w25q128fv_pageProgramBegin(fdevice, address);
    w25q128fv_pageProgramContinue(fdevice, data, length);
    w25q128fv_pageProgramFinish(fdevice);
}

MMFLASH_CODE static void w25q128fv_flush(flashDevice_t *fdevice)
{
    UNUSED(fdevice);
}

MMFLASH_CODE static int w25q128fv_readBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, int length)
{
    if (!w25q128fv_waitForReady(fdevice, W25Q128FV_TIMEOUT_PAGE_READ_MS)) {
        return 0;
    }

#ifdef USE_FLASH_READS_USING_4LINES
    bool status = quadSpiReceiveWithAddress4LINES(QSPI_DEV(fdevice),
                      W25Q128FV_INSTRUCTION_FAST_READ_QUAD_OUTPUT, 8,
                      address, W25Q128FV_ADDRESS_BITS, buffer, length);
#else
    bool status = quadSpiReceiveWithAddress1LINE(QSPI_DEV(fdevice),
                      W25Q128FV_INSTRUCTION_READ_BYTES, 0,
                      address, W25Q128FV_ADDRESS_BITS, buffer, length);
#endif

    w25q128fv_setTimeout(fdevice, W25Q128FV_TIMEOUT_PAGE_READ_MS);

    if (!status) {
        return 0;
    }

    return length;
}

static const flashGeometry_t *w25q128fv_getGeometry(flashDevice_t *fdevice)
{
    return &fdevice->geometry;
}

const flashVTable_t w25q128fv_vTable = {
    .isReady          = w25q128fv_isReady,
    .waitForReady     = w25q128fv_waitForReady,
    .eraseSector      = w25q128fv_eraseSector,
    .eraseCompletely  = w25q128fv_eraseCompletely,
    .pageProgramBegin = w25q128fv_pageProgramBegin,
    .pageProgramContinue = w25q128fv_pageProgramContinue,
    .pageProgramFinish = w25q128fv_pageProgramFinish,
    .pageProgram      = w25q128fv_pageProgram,
    .flush            = w25q128fv_flush,
    .readBytes        = w25q128fv_readBytes,
    .getGeometry      = w25q128fv_getGeometry,
};

#endif // USE_FLASH_W25Q128FV && USE_QUADSPI
