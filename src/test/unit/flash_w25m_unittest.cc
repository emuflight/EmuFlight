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

#include <cstring>

extern "C" {

#include "platform.h"

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/flash.h"
#include "drivers/flash_impl.h"
#include "drivers/flash_w25m.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// The real w25m_readBytes() runs against a fake NAND die driver that clamps every read to its
// page, as w25n_readBytes() does. Only the die driver and the SPI die-select calls are faked.

#define FAKE_PAGE_SIZE 2048u
#define FAKE_DIE_PAGES 4u
#define FAKE_DIE_SIZE (FAKE_PAGE_SIZE * FAKE_DIE_PAGES)
#define FAKE_DIE_COUNT 2u
#define JEDEC_ID_W25M02G 0xEFAB21

static const uint32_t pageSize = FAKE_PAGE_SIZE;
static const uint32_t dieSize = FAKE_DIE_SIZE;

static int dieReadCalls;
static uint32_t dieReadLength[16];
static uint32_t dieReadAddress[16];
static int failOnCall; // 1-based call number that returns 0, 0 = never
static uint32_t overReturnBy; // makes the fake die driver report more bytes than it was asked for

extern "C" {

void spiWait(const extDevice_t *dev) { (void)dev; }
void spiSequence(const extDevice_t *dev, busSegment_t *segments) { (void)dev; (void)segments; }

static uint8_t patternAt(uint32_t globalAddress)
{
    return (uint8_t)((globalAddress * 7u + (globalAddress >> 8) * 13u) ^ 0x5a);
}

static int fakeDieIndex(flashDevice_t *fdevice);

static int fakeReadBytes(flashDevice_t *fdevice, uint32_t address, uint8_t *buffer, uint32_t length)
{
    dieReadCalls++;

    if (failOnCall == dieReadCalls) {
        return 0;
    }

    const uint32_t column = address % pageSize;
    const uint32_t transfer = (length > pageSize - column) ? pageSize - column : length;
    const uint32_t base = fakeDieIndex(fdevice) * dieSize;

    if (dieReadCalls <= 16) {
        dieReadLength[dieReadCalls - 1] = length;
        dieReadAddress[dieReadCalls - 1] = address;
    }

    for (uint32_t i = 0; i < transfer; i++) {
        buffer[i] = patternAt(base + address + i);
    }

    return transfer + overReturnBy;
}

static flashVTable_t fakeVTable;

static flashDevice_t *registeredDie[FAKE_DIE_COUNT];
static unsigned registeredCount;

static int fakeDieIndex(flashDevice_t *fdevice)
{
    for (unsigned i = 0; i < registeredCount; i++) {
        if (registeredDie[i] == fdevice) {
            return i;
        }
    }
    return -1;
}

bool w25n_identify(flashDevice_t *fdevice, uint32_t jedecID)
{
    (void)jedecID;

    fakeVTable = flashVTable_t{};
    fakeVTable.readBytes = fakeReadBytes;
    fdevice->vTable = &fakeVTable;
    fdevice->geometry.pageSize = FAKE_PAGE_SIZE;
    fdevice->geometry.pagesPerSector = 64;
    fdevice->geometry.sectorSize = FAKE_PAGE_SIZE * 64;
    fdevice->geometry.sectors = 1;
    fdevice->geometry.totalSize = FAKE_DIE_SIZE;

    if (registeredCount < FAKE_DIE_COUNT) {
        registeredDie[registeredCount++] = fdevice;
    }
    return true;
}

}

class W25mReadTest : public ::testing::Test {
protected:
    flashDevice_t flash;
    uint8_t buffer[3 * FAKE_PAGE_SIZE + 64];
    uint8_t expected[3 * FAKE_PAGE_SIZE + 64];

    void SetUp() override
    {
        memset(&flash, 0, sizeof(flash));
        registeredCount = 0;
        dieReadCalls = 0;
        failOnCall = 0;
        overReturnBy = 0;
        memset(dieReadLength, 0, sizeof(dieReadLength));
        memset(dieReadAddress, 0, sizeof(dieReadAddress));
        memset(buffer, 0xEE, sizeof(buffer));

        ASSERT_TRUE(w25m_identify(&flash, JEDEC_ID_W25M02G));
        ASSERT_EQ(FAKE_DIE_SIZE * FAKE_DIE_COUNT, flash.geometry.totalSize);
    }

    // Expected bytes come from the pattern definition, not from the driver's output.
    void expectData(uint32_t address, uint32_t length)
    {
        for (uint32_t i = 0; i < length; i++) {
            ASSERT_EQ(patternAt(address + i), buffer[i]) << "offset " << i << " of read at " << address;
        }
    }
};

TEST_F(W25mReadTest, ReadWithinOnePageIsOneDieCall)
{
    const int n = flash.vTable->readBytes(&flash, 100, buffer, 512);

    EXPECT_EQ(512, n);
    EXPECT_EQ(1, dieReadCalls);
    EXPECT_EQ(512u, dieReadLength[0]);
    expectData(100, 512);
}

TEST_F(W25mReadTest, ReadEndingExactlyOnPageBoundaryIsOneDieCall)
{
    const int n = flash.vTable->readBytes(&flash, 1024, buffer, 1024);

    EXPECT_EQ(1024, n);
    EXPECT_EQ(1, dieReadCalls);
    expectData(1024, 1024);
}

TEST_F(W25mReadTest, ReadCrossingPageBoundaryReturnsAllBytesInOrder)
{
    const uint32_t address = pageSize - 100;
    const int n = flash.vTable->readBytes(&flash, address, buffer, 300);

    EXPECT_EQ(300, n);
    EXPECT_EQ(2, dieReadCalls);
    EXPECT_EQ(address, dieReadAddress[0]);
    EXPECT_EQ(pageSize, dieReadAddress[1]);
    expectData(address, 300);
    // Nothing past the requested length may be written.
    EXPECT_EQ(0xEE, buffer[300]);
}

TEST_F(W25mReadTest, MultiPageReadAdvancesByReturnedLength)
{
    const uint32_t address = 10;
    const uint32_t length = 2 * pageSize + 500;
    const int n = flash.vTable->readBytes(&flash, address, buffer, length);

    EXPECT_EQ((int)length, n);
    EXPECT_EQ(3, dieReadCalls);
    expectData(address, length);
}

TEST_F(W25mReadTest, ReadCrossingDieBoundaryReadsBothDies)
{
    const uint32_t address = dieSize - 50;
    const int n = flash.vTable->readBytes(&flash, address, buffer, 120);

    EXPECT_EQ(120, n);
    EXPECT_EQ(2, dieReadCalls);
    EXPECT_EQ(dieSize - 50, dieReadAddress[0]);
    EXPECT_EQ(0u, dieReadAddress[1]); // second die starts at its own address 0
    expectData(address, 120);
}

TEST_F(W25mReadTest, ReadCrossingPageAndDieBoundariesIsCorrect)
{
    const uint32_t address = dieSize - pageSize - 40;
    const uint32_t length = pageSize + 200;
    const int n = flash.vTable->readBytes(&flash, address, buffer, length);

    EXPECT_EQ((int)length, n);
    expectData(address, length);
}

TEST_F(W25mReadTest, FailureOnFirstRoundReturnsZero)
{
    failOnCall = 1;

    EXPECT_EQ(0, flash.vTable->readBytes(&flash, 0, buffer, 100));
}

TEST_F(W25mReadTest, FailureOnLaterRoundReturnsBytesAlreadyRead)
{
    failOnCall = 2;

    const uint32_t address = pageSize - 100;
    const int n = flash.vTable->readBytes(&flash, address, buffer, 300);

    EXPECT_EQ(100, n);
    EXPECT_EQ(2, dieReadCalls);
    expectData(address, 100);
}

TEST_F(W25mReadTest, DieDriverReturningMoreThanAskedStopsTheRead)
{
    overReturnBy = 1;

    const int n = flash.vTable->readBytes(&flash, 0, buffer, 100);

    EXPECT_EQ(0, n);
    EXPECT_EQ(1, dieReadCalls);
}

TEST_F(W25mReadTest, ReadPastEndOfDeviceStopsAtTheEnd)
{
    const uint32_t total = flash.geometry.totalSize;

    EXPECT_EQ(0, flash.vTable->readBytes(&flash, total, buffer, 64));

    const int n = flash.vTable->readBytes(&flash, total - 30, buffer, 64);
    EXPECT_EQ(30, n);
    expectData(total - 30, 30);
}

TEST_F(W25mReadTest, ZeroLengthReadReturnsZeroWithoutDieCall)
{
    EXPECT_EQ(0, flash.vTable->readBytes(&flash, 0, buffer, 0));
    EXPECT_EQ(0, dieReadCalls);
}
