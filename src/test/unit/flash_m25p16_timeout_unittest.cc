/*
 * This file is part of EmuFlight.
 *
 * EmuFlight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * EmuFlight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with EmuFlight.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <cstdint>
#include <cstring>

extern "C" {

#include "platform.h"

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/flash.h"
#include "drivers/flash_impl.h"
#include "drivers/time.h"

bool m25p16_identify(flashDevice_t *fdevice, uint32_t jedecID);

}

#include "gtest/gtest.h"

// The real m25p16 driver runs against a fake NOR chip. The fake chip stays busy for a set time
// after an erase or page program, as a real chip does. The fake clock advances by 1 ms per
// status poll. Busy times come from the Winbond datasheets (W25Q128FV tBE2 max 2000 ms,
// tCE max 200 s, tPP max 3 ms), not from the driver.

#define JEDEC_ID_W25Q128 0xEF4018u
#define STATUS_WIP 0x01
#define CMD_READ_STATUS 0x05
#define CMD_WRITE_ENABLE 0x06
#define CMD_PAGE_PROGRAM 0x02
#define CMD_BLOCK_ERASE 0xD8
#define CMD_CHIP_ERASE 0xC7
#define FOREVER 0xFFFFFFFFu

static uint32_t fakeNowMs;
static uint32_t busyUntilMs;
static uint32_t blockEraseBusyMs;
static uint32_t chipEraseBusyMs;
static uint32_t pageProgramBusyMs;
static uint32_t busBusyPolls;
static int blockEraseCount;
static int chipEraseCount;
static int pageProgramCount;

static busDevice_t fakeBus;
static extDevice_t fakeDev;

static void chipBusyFor(uint32_t ms)
{
    busyUntilMs = (ms == FOREVER) ? FOREVER : fakeNowMs + ms;
}

static bool chipIsBusy(void)
{
    // Wrap-safe compare, as the driver uses.
    return busyUntilMs == FOREVER || (int32_t)(fakeNowMs - busyUntilMs) < 0;
}

extern "C" {

timeMs_t millis(void) { return fakeNowMs; }

void spiWait(const extDevice_t *dev) { (void)dev; }
// Another device holds the shared bus for busBusyPolls polls; each poll takes 1 ms.
bool spiIsBusy(const extDevice_t *dev)
{
    (void)dev;
    if (busBusyPolls > 0) {
        busBusyPolls--;
        fakeNowMs++;
        return true;
    }
    return false;
}
void spiSetClkDivisor(const extDevice_t *dev, uint16_t divider) { (void)dev; (void)divider; }
uint16_t spiCalculateDivider(uint32_t freq) { (void)freq; return 2; }
uint16_t spiGetErrorCounter(SPI_TypeDef *instance) { (void)instance; return 0; }

void spiReadWriteBuf(const extDevice_t *dev, uint8_t *txData, uint8_t *rxData, int len)
{
    (void)dev;
    switch (txData[0]) {
    case CMD_READ_STATUS:
        fakeNowMs++;
        if (rxData && len > 1) {
            rxData[1] = chipIsBusy() ? STATUS_WIP : 0;
        }
        break;
    case CMD_BLOCK_ERASE:
        blockEraseCount++;
        chipBusyFor(blockEraseBusyMs);
        break;
    case CMD_CHIP_ERASE:
        chipEraseCount++;
        chipBusyFor(chipEraseBusyMs);
        break;
    default:
        break;
    }
}

void spiSequence(const extDevice_t *dev, busSegment_t *segments)
{
    (void)dev;
    if (segments[0].u.buffers.txData[0] == CMD_PAGE_PROGRAM) {
        pageProgramCount++;
        chipBusyFor(pageProgramBusyMs);
    }
}

}

class FlashM25p16TimeoutTest : public ::testing::Test {
protected:
    flashDevice_t fdevice;
    uint8_t data[4];

    void SetUp() override
    {
        fakeNowMs = 100000;
        busyUntilMs = 0;
        blockEraseBusyMs = 0;
        chipEraseBusyMs = 0;
        pageProgramBusyMs = 0;
        busBusyPolls = 0;
        blockEraseCount = chipEraseCount = pageProgramCount = 0;
        memset(&fakeBus, 0, sizeof(fakeBus));
        memset(&fakeDev, 0, sizeof(fakeDev));
        fakeDev.bus = &fakeBus;
        memset(&fdevice, 0, sizeof(fdevice));
        fdevice.io.mode = FLASHIO_SPI;
        fdevice.io.handle.dev = &fakeDev;
        memset(data, 0xA5, sizeof(data));
        ASSERT_TRUE(m25p16_identify(&fdevice, JEDEC_ID_W25Q128));
    }

    uint32_t program()
    {
        const uint8_t *buffers[1] = { data };
        uint32_t sizes[1] = { sizeof(data) };
        fdevice.vTable->pageProgramBegin(&fdevice, 0, NULL);
        return fdevice.vTable->pageProgramContinue(&fdevice, buffers, sizes, 1);
    }
};

TEST_F(FlashM25p16TimeoutTest, ProgramAfterBlockEraseOfTypicalDatasheetTime)
{
    blockEraseBusyMs = 150; // W25Q128FV tBE2 typ
    fdevice.vTable->eraseSector(&fdevice, 0);
    EXPECT_EQ(sizeof(data), program());
}

TEST_F(FlashM25p16TimeoutTest, ProgramAfterBlockEraseLongerThan500ms)
{
    blockEraseBusyMs = 1500; // inside tBE2 max 2000 ms
    fdevice.vTable->eraseSector(&fdevice, 0);
    EXPECT_EQ(sizeof(data), program());
    EXPECT_EQ(1, pageProgramCount);
}

TEST_F(FlashM25p16TimeoutTest, ProgramAfterBlockEraseAtDatasheetMaximum)
{
    blockEraseBusyMs = 2000; // W25Q128FV tBE2 max
    fdevice.vTable->eraseSector(&fdevice, 0);
    EXPECT_EQ(sizeof(data), program());
}

TEST_F(FlashM25p16TimeoutTest, ProgramAfterBlockEraseOfHybridParameterRegionAtMaximum)
{
    blockEraseBusyMs = 10400; // S25FL128S tSE max, 64 KB group of sixteen 4 KB sectors
    fdevice.vTable->eraseSector(&fdevice, 0);
    EXPECT_EQ(sizeof(data), program());
}

TEST_F(FlashM25p16TimeoutTest, ProgramAfterChipEraseOfW25q512AtMaximum)
{
    chipEraseBusyMs = 1000u * 1000u; // W25Q512JV tCE max
    fdevice.vTable->eraseCompletely(&fdevice);
    EXPECT_EQ(sizeof(data), program());
}

TEST_F(FlashM25p16TimeoutTest, BackToBackBlockErasesBothReachTheChip)
{
    blockEraseBusyMs = 1500;
    fdevice.vTable->eraseSector(&fdevice, 0);
    fdevice.vTable->eraseSector(&fdevice, 0x10000);
    EXPECT_EQ(2, blockEraseCount);
}

TEST_F(FlashM25p16TimeoutTest, ProgramAfterChipEraseAtDatasheetMaximum)
{
    chipEraseBusyMs = 200u * 1000u; // W25Q128FV tCE max
    fdevice.vTable->eraseCompletely(&fdevice);
    EXPECT_EQ(sizeof(data), program());
}

TEST_F(FlashM25p16TimeoutTest, BackToBackProgramsAtDatasheetMaximum)
{
    pageProgramBusyMs = 3; // W25Q128FV tPP max
    EXPECT_EQ(sizeof(data), program());
    EXPECT_EQ(sizeof(data), program());
    EXPECT_EQ(2, pageProgramCount);
}

TEST_F(FlashM25p16TimeoutTest, StuckChipAfterBlockEraseStillTimesOut)
{
    blockEraseBusyMs = FOREVER;
    fdevice.vTable->eraseSector(&fdevice, 0);
    const uint32_t start = fakeNowMs;
    EXPECT_EQ(0u, program());
    EXPECT_EQ(0, pageProgramCount);
    EXPECT_LT(fakeNowMs - start, 12000u); // bounded by the block erase maximum, not unbounded
}

TEST_F(FlashM25p16TimeoutTest, StuckChipBlocksTheNextEraseCommand)
{
    blockEraseBusyMs = FOREVER;
    fdevice.vTable->eraseSector(&fdevice, 0);
    fdevice.vTable->eraseSector(&fdevice, 0x10000);
    EXPECT_EQ(1, blockEraseCount);
}

TEST_F(FlashM25p16TimeoutTest, StuckChipAfterChipEraseStillTimesOut)
{
    chipEraseBusyMs = FOREVER;
    fdevice.vTable->eraseCompletely(&fdevice);
    const uint32_t start = fakeNowMs;
    EXPECT_EQ(0u, program());
    EXPECT_LT(fakeNowMs - start, 1100u * 1000u);
}

TEST_F(FlashM25p16TimeoutTest, StuckChipAfterProgramTimesOutQuickly)
{
    pageProgramBusyMs = FOREVER;
    EXPECT_EQ(sizeof(data), program());
    const uint32_t start = fakeNowMs;
    EXPECT_EQ(0u, program());
    EXPECT_LT(fakeNowMs - start, 100u);
}

TEST_F(FlashM25p16TimeoutTest, WaitSurvivesMillisWraparound)
{
    fakeNowMs = 0xFFFFFFFFu - 500u; // the erase stays busy across the 2^32 wrap
    blockEraseBusyMs = 1500;
    fdevice.vTable->eraseSector(&fdevice, 0);
    EXPECT_EQ(sizeof(data), program());
    EXPECT_LT(fakeNowMs, 2000u); // clock wrapped while the chip was busy
}

TEST_F(FlashM25p16TimeoutTest, ProgramAtMaximumOfFiveMilliseconds)
{
    pageProgramBusyMs = 5; // MX25L6406E / N25Q064A tPP max
    EXPECT_EQ(sizeof(data), program());
    EXPECT_EQ(sizeof(data), program());
    EXPECT_EQ(2, pageProgramCount);
}

TEST_F(FlashM25p16TimeoutTest, IdleChipOnBusySharedBusIgnoresStaleChipDeadline)
{
    pageProgramBusyMs = 3;
    EXPECT_EQ(sizeof(data), program());
    fakeNowMs += 60000; // chip long idle: its old deadline is far in the past
    fdevice.couldBeBusy = false;
    fakeBus.useDMA = true;
    busBusyPolls = 100; // another device holds the shared bus for 100 ms
    EXPECT_EQ(sizeof(data), program());
    EXPECT_EQ(2, pageProgramCount);
}

TEST_F(FlashM25p16TimeoutTest, StuckSharedBusStillTimesOut)
{
    fdevice.couldBeBusy = false; // idle chip, only the bus is stuck
    fakeBus.useDMA = true;
    busBusyPolls = FOREVER;
    const uint32_t start = fakeNowMs;
    EXPECT_EQ(0u, program());
    EXPECT_EQ(0, pageProgramCount);
    EXPECT_LT(fakeNowMs - start, 1000u);
}
