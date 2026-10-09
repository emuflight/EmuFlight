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

#include <atomic>
#include <chrono>
#include <cstring>
#include <thread>

extern "C" {

#include "platform.h"

#include "drivers/flash.h"

#include "io/flashfs.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// The real io/flashfs.c runs against a fake flash driver that models an asynchronously completing
// device: it reports ready at all times and completes each accepted write only when the test
// invokes the stored completion callback, i.e. a write still in flight while the device is ready.

#define FAKE_SECTOR_SIZE 4096u
#define FAKE_SECTORS 4u

static flashGeometry_t fakeGeometry;
static flashPartition_t fakePartition;

static void (*pendingCallback)(uint32_t arg);
static uint32_t pendingBytes;
static int programCalls;
static uint32_t presentedTotal;
static bool acceptNothing;
static bool callbackSync; // models a driver that completes inside flashPageProgramContinue(), as the in-tree drivers do
static bool callbackOnZero; // models a driver that reports zero bytes through its callback

extern "C" {

bool flashIsReady(void) { return true; }
const flashGeometry_t *flashGetGeometry(void) { return &fakeGeometry; }
flashPartition_t *flashPartitionFindByType(flashPartitionType_e type) { (void)type; return &fakePartition; }
int flashPartitionCount(void) { return 1; }
void flashEraseSector(uint32_t address) { (void)address; }
void flashEraseCompletely(void) {}
void flashFlush(void) {}

int flashReadBytes(uint32_t address, uint8_t *buffer, uint32_t length)
{
    (void)address;
    memset(buffer, 0xFF, length);
    return length;
}

void flashPageProgramBegin(uint32_t address, void (*callback)(uint32_t arg))
{
    (void)address;
    pendingCallback = callback;
}

uint32_t flashPageProgramContinue(const uint8_t **buffers, uint32_t *bufferSizes, uint32_t bufferCount)
{
    (void)buffers;

    programCalls++;

    if (acceptNothing) {
        if (callbackOnZero && pendingCallback) {
            pendingCallback(0);
        }
        return 0;
    }

    uint32_t total = 0;
    for (uint32_t i = 0; i < bufferCount; i++) {
        total += bufferSizes[i];
    }
    pendingBytes = total;
    presentedTotal += total;

    if (callbackSync && pendingCallback) {
        pendingBytes = 0;
        pendingCallback(total);
    }

    return total;
}

void flashPageProgramFinish(void) {}

}

class FlashfsInterlock : public ::testing::Test {
protected:
    void SetUp() override
    {
        fakeGeometry = {};
        fakeGeometry.sectors = FAKE_SECTORS;
        fakeGeometry.pageSize = 256;
        fakeGeometry.sectorSize = FAKE_SECTOR_SIZE;
        fakeGeometry.totalSize = FAKE_SECTOR_SIZE * FAKE_SECTORS;
        fakeGeometry.pagesPerSector = FAKE_SECTOR_SIZE / 256;
        fakeGeometry.flashType = FLASH_TYPE_NOR;
        fakePartition = {};
        fakePartition.startSector = 0;
        fakePartition.endSector = FAKE_SECTORS - 1;

        completePending();
        acceptNothing = false;
        callbackOnZero = false;
        callbackSync = false;
        flashfsInit();
        flashfsSeekAbs(0);
        programCalls = 0;
        presentedTotal = 0;
        pendingBytes = 0;
        pendingCallback = nullptr;
    }

    void TearDown() override
    {
        completePending();
    }

    static void completePending()
    {
        if (pendingCallback && pendingBytes) {
            const uint32_t bytes = pendingBytes;
            pendingBytes = 0;
            pendingCallback(bytes);
        }
    }
};

// A threshold-crossing async write starts one device write mid-loop; the capture at the end of
// flashfsWrite() must not present the same bytes again while that write is outstanding.
TEST_F(FlashfsInterlock, AsyncWriteDoesNotRepresentInFlightBytes)
{
    uint8_t data[80];
    memset(data, 0xA5, sizeof(data));

    flashfsWrite(data, sizeof(data), false);

    EXPECT_EQ(1, programCalls);

    completePending();
    flashfsFlushAsync(true);
    completePending();

    EXPECT_EQ(sizeof(data), presentedTotal);
    EXPECT_EQ(sizeof(data), flashfsGetOffset());
}

// With a write outstanding, a synchronous flush must wait for its completion before it captures
// the buffers, otherwise both completions advance the tail past the head.
TEST_F(FlashfsInterlock, FlushSyncWaitsForInFlightWrite)
{
    uint8_t data[80];
    memset(data, 0x5A, sizeof(data));

    flashfsWrite(data, sizeof(data), false);
    ASSERT_EQ(1, programCalls);

    std::atomic<int> callsAtCompletion(-1);
    std::thread completer([&callsAtCompletion]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        callsAtCompletion = programCalls;
        completePending();
    });

    flashfsFlushSync();
    completer.join();

    EXPECT_EQ(1, callsAtCompletion);
    EXPECT_EQ(2, programCalls);
    completePending();

    EXPECT_EQ(sizeof(data), presentedTotal);
    EXPECT_EQ(sizeof(data), flashfsGetOffset());
}

// A synchronous write with a write outstanding waits for it before presenting the remainder.
TEST_F(FlashfsInterlock, SyncWriteWaitsForInFlightWrite)
{
    uint8_t data[80];
    memset(data, 0x3C, sizeof(data));

    flashfsWrite(data, 64, false);
    ASSERT_EQ(1, programCalls);

    std::atomic<int> callsAtCompletion(-1);
    std::thread completer([&callsAtCompletion]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        callsAtCompletion = programCalls;
        completePending();
    });

    flashfsWrite(data, 64, true);
    completer.join();

    EXPECT_EQ(1, callsAtCompletion);
    completePending();

    EXPECT_EQ(128u, presentedTotal);
}

// A driver that accepts nothing never calls back, so the interlock must be released at once and
// the dirty bytes presented again on the next flush instead of stalling every later write.
TEST_F(FlashfsInterlock, ZeroByteAcceptReleasesInterlock)
{
    uint8_t data[64];
    memset(data, 0xC3, sizeof(data));

    acceptNothing = true;
    flashfsWrite(data, sizeof(data), false);
    ASSERT_GE(programCalls, 1);
    const int callsBefore = programCalls;

    acceptNothing = false;
    flashfsFlushAsync(true);

    EXPECT_EQ(callsBefore + 1, programCalls);
    EXPECT_EQ(sizeof(data), presentedTotal);

    completePending();
    EXPECT_EQ(sizeof(data), flashfsGetOffset());
}

// In-tree drivers complete inside flashPageProgramContinue(); every write path must behave as before
// the interlock waits existed: no hang, every byte presented exactly once.
TEST_F(FlashfsInterlock, SynchronousCompletionPresentsEveryByteOnce)
{
    uint8_t data[200];
    for (unsigned i = 0; i < sizeof(data); i++) {
        data[i] = (uint8_t)i;
    }

    callbackSync = true;
    flashfsWrite(data, 100, false);
    flashfsWrite(data + 100, 100, true);
    flashfsFlushSync();

    EXPECT_EQ(sizeof(data), presentedTotal);
    EXPECT_EQ(sizeof(data), flashfsGetOffset());
}

// A driver that reports zero bytes through its callback as well as its return value must not
// leave the interlock set or advance the tail.
TEST_F(FlashfsInterlock, ZeroByteCallbackDoesNotStallOrAdvance)
{
    uint8_t data[64];
    memset(data, 0x7E, sizeof(data));

    acceptNothing = true;
    callbackOnZero = true;
    flashfsWrite(data, sizeof(data), false);
    ASSERT_GE(programCalls, 1);
    const int callsBefore = programCalls;
    EXPECT_EQ(0u, presentedTotal);

    acceptNothing = false;
    flashfsFlushAsync(true);

    EXPECT_EQ(callsBefore + 1, programCalls);
    EXPECT_EQ(sizeof(data), presentedTotal);

    completePending();
    EXPECT_EQ(sizeof(data), flashfsGetOffset());
}

// Adversarial: no write outstanding and nothing buffered must stay a no-op for both entry points.
TEST_F(FlashfsInterlock, EmptyBufferPresentsNothing)
{
    uint8_t unused = 0;

    flashfsFlushSync();
    flashfsWrite(&unused, 0, true);
    flashfsWrite(&unused, 0, false);

    EXPECT_EQ(0, programCalls);
    EXPECT_EQ(0u, flashfsGetOffset());
}

