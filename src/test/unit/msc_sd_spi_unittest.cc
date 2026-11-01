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

#include <stdint.h>

extern "C" {

#include "platform.h"
#include "drivers/sdcard.h"
#include "pg/sdcard.h"
#include "msc/usbd_storage.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// Compiles the real msc/usbd_storage_sd_spi.c. Each STORAGE_* wait must end on a card that
// never answers. SD spec minimums (physical layer spec): init 1 s, read 100 ms, write 250 ms;
// a bound shorter than those would cut off a healthy card.
namespace {

constexpr uint32_t SPEC_INIT_MS = 1000;
constexpr uint32_t SPEC_READ_MS = 100;
constexpr uint32_t SPEC_WRITE_MS = 250;
constexpr uint32_t MAX_CALLS = 100000;

uint32_t fakeNowMs;
uint32_t pollCalls;
uint32_t initCalls;
uint32_t pollReadyAfter;     // poll returns true once this many calls were made; UINT32_MAX = never
bool readStartReady;         // sdcard_readBlock result
sdcardOperationStatus_e writeStartResult;

void setUpFakeCard(uint32_t startMs) {
    fakeNowMs = startMs;
    pollCalls = 0;
    initCalls = 0;
    pollReadyAfter = UINT32_MAX;
    readStartReady = true;
    writeStartResult = SDCARD_OPERATION_IN_PROGRESS;
}

uint32_t elapsed(uint32_t startMs) {
    return fakeNowMs - startMs;
}

}  // namespace

extern "C" {

sdcardConfig_t sdcardConfig_System;

uint32_t millis(void) {
    return fakeNowMs++;
}

void sdcard_init(const sdcardConfig_t *) { initCalls++; }

bool sdcard_poll(void) {
    pollCalls++;
    return pollCalls > pollReadyAfter;
}

const sdcardMetadata_t *sdcard_getMetadata(void) {
    static sdcardMetadata_t metadata;
    return &metadata;
}

bool sdcard_readBlock(uint32_t, uint8_t *, sdcard_operationCompleteCallback_c, uint32_t) {
    return readStartReady;
}

sdcardOperationStatus_e sdcard_writeBlock(uint32_t, uint8_t *, sdcard_operationCompleteCallback_c, uint32_t) {
    return writeStartResult;
}

}

TEST(MscSdSpiTest, InitReturnsErrorWhenCardNeverReady) {
    const uint32_t start = 5;
    setUpFakeCard(start);
    EXPECT_EQ(-1, USBD_MSC_MICRO_SDIO_fops.Init(0));
    EXPECT_GE(elapsed(start), SPEC_INIT_MS);
    EXPECT_LT(pollCalls, MAX_CALLS);
}

TEST(MscSdSpiTest, InitSucceedsWhenCardBecomesReady) {
    setUpFakeCard(0);
    pollReadyAfter = 300;
    EXPECT_EQ(0, USBD_MSC_MICRO_SDIO_fops.Init(0));
}

TEST(MscSdSpiTest, InitTimeoutSurvivesMillisWraparound) {
    const uint32_t start = UINT32_MAX - 10;
    setUpFakeCard(start);
    EXPECT_EQ(-1, USBD_MSC_MICRO_SDIO_fops.Init(0));
    EXPECT_GE(elapsed(start), SPEC_INIT_MS);
}

TEST(MscSdSpiTest, ReadReturnsErrorWhenCardNeverStartsRead) {
    const uint32_t start = 0;
    setUpFakeCard(start);
    readStartReady = false;
    uint8_t buf[512];
    EXPECT_EQ(-1, USBD_MSC_MICRO_SDIO_fops.Read(0, buf, 0, 1));
    EXPECT_GE(elapsed(start), SPEC_READ_MS);
}

TEST(MscSdSpiTest, ReadReturnsErrorWhenCardNeverFinishesRead) {
    const uint32_t start = 0;
    setUpFakeCard(start);
    uint8_t buf[512];
    EXPECT_EQ(-1, USBD_MSC_MICRO_SDIO_fops.Read(0, buf, 0, 1));
    EXPECT_GE(elapsed(start), SPEC_READ_MS);
    EXPECT_LT(pollCalls, MAX_CALLS);
    EXPECT_EQ(1u, initCalls);  // the pending read is dropped before the USB buffer is handed back
}

TEST(MscSdSpiTest, ReadSucceedsWhenCardAnswers) {
    setUpFakeCard(0);
    pollReadyAfter = 0;
    uint8_t buf[512];
    EXPECT_EQ(0, USBD_MSC_MICRO_SDIO_fops.Read(0, buf, 0, 1));
    EXPECT_EQ(0u, initCalls);  // a healthy transfer never resets the driver
}

TEST(MscSdSpiTest, WriteReturnsErrorWhenCardStaysBusy) {
    const uint32_t start = 0;
    setUpFakeCard(start);
    writeStartResult = SDCARD_OPERATION_BUSY;
    uint8_t buf[512] = {};
    EXPECT_EQ(-1, USBD_MSC_MICRO_SDIO_fops.Write(0, buf, 0, 1));
    EXPECT_GE(elapsed(start), SPEC_WRITE_MS);
    EXPECT_EQ(1u, initCalls);
}

TEST(MscSdSpiTest, WriteReturnsErrorWhenCardNeverFinishesWrite) {
    const uint32_t start = 0;
    setUpFakeCard(start);
    uint8_t buf[512] = {};
    EXPECT_EQ(-1, USBD_MSC_MICRO_SDIO_fops.Write(0, buf, 0, 1));
    EXPECT_GE(elapsed(start), SPEC_WRITE_MS);
}

TEST(MscSdSpiTest, WriteSucceedsWhenCardAnswers) {
    setUpFakeCard(0);
    pollReadyAfter = 0;
    uint8_t buf[512] = {};
    EXPECT_EQ(0, USBD_MSC_MICRO_SDIO_fops.Write(0, buf, 0, 1));
}
