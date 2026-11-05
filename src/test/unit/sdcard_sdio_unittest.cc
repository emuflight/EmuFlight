/*
 * This file is part of EmuFlight.
 *
 * EmuFlight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * EmuFlight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <cstring>
#include <limits>

extern "C" {
#include "platform.h"
#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/time.h"
#include "pg/sdio.h"
#include "drivers/sdcard.h"
#include "drivers/sdcard_standard.h"
#include "drivers/sdmmc_sdio.h"
}

#include "gtest/gtest.h"

extern "C" {

sdioConfig_t sdioConfig_System;
sdioConfig_t sdioConfig_Copy;
SD_CardInfo_t SD_CardInfo;
SD_CardType_t SD_CardType;

static timeMs_t mockTimeMs;
static SD_Error_t mockInitStatus;
static SD_Error_t mockReadStatus;
static SD_Error_t mockWriteStatus;
static bool mockCardReady;
static unsigned initCalls;
static unsigned readCheckCalls;
static unsigned writeCheckCalls;
static unsigned stateCalls;
static unsigned cardInfoCalls;
static bool failCsdRead;
static unsigned readCompletesAfterCalls;
static unsigned cardReadyAfterCalls;
static unsigned callbackCalls;
static uint8_t *callbackBuffer;

timeMs_t millis(void) { return mockTimeMs; }
timeUs_t micros(void) { return mockTimeMs * 1000; }

IO_t IOGetByTag(ioTag_t tag) { return tag ? (IO_t)1 : (IO_t)0; }
static unsigned ioInitCalls;
static unsigned ioConfigCalls;
static resourceOwner_e lastIoOwner;
static ioConfig_t lastIoConfig;
void IOInit(IO_t, resourceOwner_e owner, uint8_t) { ioInitCalls++; lastIoOwner = owner; }
void IOConfigGPIO(IO_t, ioConfig_t config) { ioConfigCalls++; lastIoConfig = config; }
static bool mockCardDetectRead;
bool IORead(IO_t) { return mockCardDetectRead; }
static bool mockDmaSpecAvailable;
static dmaChannelSpec_t mockDmaSpec;
const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e, uint8_t, int8_t) { return mockDmaSpecAvailable ? &mockDmaSpec : nullptr; }

bool SD_Initialize_LL(DMA_Stream_TypeDef *) { return true; }
SD_Error_t SD_Init(void) { initCalls++; return mockInitStatus; }
bool SD_GetState(void) { stateCalls++; return mockCardReady || stateCalls > cardReadyAfterCalls; }
SD_Error_t SD_GetCardInfo(void) { cardInfoCalls++; return failCsdRead && cardInfoCalls % 2 == 0 ? SD_ERROR : SD_OK; }
SD_Error_t SD_ReadBlocks_DMA(uint64_t, uint32_t *, uint32_t, uint32_t) { return SD_OK; }
SD_Error_t SD_CheckRead(void)
{
    readCheckCalls++;
    return readCheckCalls > readCompletesAfterCalls ? SD_OK : mockReadStatus;
}
SD_Error_t SD_WriteBlocks_DMA(uint64_t, uint32_t *, uint32_t, uint32_t) { return SD_OK; }
SD_Error_t SD_CheckWrite(void)
{
    writeCheckCalls++;
    return mockWriteStatus;
}

static void operationComplete(sdcardBlockOperation_e, uint32_t, uint8_t *buffer, uint32_t)
{
    callbackCalls++;
    callbackBuffer = buffer;
}

}

// Include the driver itself: its `sdcard` state is file-static.
extern "C" {
#include "drivers/sdcard_sdio_baremetal.c"
}

class SdcardSdioTest : public ::testing::Test {
protected:
    uint8_t buffer[512];

    void SetUp() override
    {
        std::memset(&sdcard, 0, sizeof(sdcard));
        cacheCount = 0;
        mockCardDetectRead = false;
        mockDmaSpecAvailable = false;
        ioInitCalls = 0;
        ioConfigCalls = 0;
        std::memset(buffer, 0, sizeof(buffer));
        sdcard.enabled = true;
        sdcard.pendingOperation.buffer = buffer;
        sdcard.pendingOperation.blockIndex = 7;
        sdcard.pendingOperation.callback = operationComplete;
        mockTimeMs = 1000;
        mockInitStatus = SD_OK;
        mockReadStatus = SD_OK;
        mockWriteStatus = SD_OK;
        mockCardReady = true;
        initCalls = 0;
        readCheckCalls = 0;
        writeCheckCalls = 0;
        stateCalls = 0;
        cardInfoCalls = 0;
        failCsdRead = false;
        readCompletesAfterCalls = std::numeric_limits<unsigned>::max();
        cardReadyAfterCalls = std::numeric_limits<unsigned>::max();
        callbackCalls = 0;
        callbackBuffer = nullptr;
    }
};

TEST_F(SdcardSdioTest, RejectsCompletedReadWhenCardIsNotReady)
{
    sdcard.state = SDCARD_STATE_READING;
    sdcard.operationStartTime = mockTimeMs;
    mockCardReady = false;
    cardReadyAfterCalls = 32;

    EXPECT_FALSE(sdcard_poll());

    EXPECT_EQ(1u, readCheckCalls);
    EXPECT_EQ(1u, stateCalls);
    EXPECT_EQ(1u, initCalls);
    EXPECT_EQ(SDCARD_STATE_RESET, sdcard.state);
    EXPECT_EQ(1u, callbackCalls);
    EXPECT_EQ(nullptr, callbackBuffer);
}

TEST_F(SdcardSdioTest, TimesOutBusyReadWithoutRetryingInSamePoll)
{
    sdcard.state = SDCARD_STATE_READING;
    sdcard.operationStartTime = mockTimeMs - SDCARD_TIMEOUT_READ_MSEC - 1;
    mockReadStatus = SD_BUSY;
    readCompletesAfterCalls = 32;

    EXPECT_FALSE(sdcard_poll());

    EXPECT_EQ(1u, readCheckCalls);
    EXPECT_EQ(1u, initCalls);
    EXPECT_EQ(SDCARD_STATE_RESET, sdcard.state);
    EXPECT_EQ(1u, callbackCalls);
    EXPECT_EQ(nullptr, callbackBuffer);
}

TEST_F(SdcardSdioTest, LeavesBusyReadPendingBeforeTimeout)
{
    sdcard.state = SDCARD_STATE_READING;
    sdcard.operationStartTime = mockTimeMs;
    mockReadStatus = SD_BUSY;

    EXPECT_FALSE(sdcard_poll());

    EXPECT_EQ(1u, readCheckCalls);
    EXPECT_EQ(0u, initCalls);
    EXPECT_EQ(SDCARD_STATE_READING, sdcard.state);
    EXPECT_EQ(0u, callbackCalls);
}

TEST_F(SdcardSdioTest, SuccessfulReadCompletesNormally)
{
    sdcard.state = SDCARD_STATE_READING;
    sdcard.operationStartTime = mockTimeMs;

    EXPECT_TRUE(sdcard_poll());

    EXPECT_EQ(1u, readCheckCalls);
    EXPECT_EQ(SDCARD_STATE_READY, sdcard.state);
    EXPECT_EQ(1u, callbackCalls);
    EXPECT_EQ(buffer, callbackBuffer);
}

TEST_F(SdcardSdioTest, SuccessfulReinitChangesWriteRecoveryState)
{
    sdcard.state = SDCARD_STATE_WAITING_FOR_WRITE;
    sdcard.operationStartTime = mockTimeMs - SDCARD_TIMEOUT_WRITE_MSEC - 1;
    mockCardReady = false;
    cardReadyAfterCalls = 32;

    EXPECT_FALSE(sdcard_poll());

    EXPECT_EQ(1u, initCalls);
    EXPECT_EQ(SDCARD_STATE_RESET, sdcard.state);
    EXPECT_EQ(1u, stateCalls);
}

TEST_F(SdcardSdioTest, TimesOutBusyWriteAndReportsFailure)
{
    sdcard.state = SDCARD_STATE_SENDING_WRITE;
    sdcard.operationStartTime = mockTimeMs - SDCARD_TIMEOUT_WRITE_MSEC - 1;
    mockWriteStatus = SD_BUSY;

    EXPECT_FALSE(sdcard_poll());

    EXPECT_EQ(1u, writeCheckCalls);
    EXPECT_EQ(1u, initCalls);
    EXPECT_EQ(SDCARD_STATE_RESET, sdcard.state);
    EXPECT_EQ(1u, callbackCalls);
    EXPECT_EQ(nullptr, callbackBuffer);
}

TEST_F(SdcardSdioTest, ResetAfterFailedCachedWriteDropsCachedBlocks)
{
    // Three blocks cached for a multi-block write that then fails with a write timeout.
    cacheCount = 3 * 512;
    sdcard.useCache = true;
    sdcard.state = SDCARD_STATE_SENDING_WRITE;
    sdcard.operationStartTime = mockTimeMs - SDCARD_TIMEOUT_WRITE_MSEC - 1;
    mockWriteStatus = SD_BUSY;

    EXPECT_FALSE(sdcard_poll());

    EXPECT_EQ(SDCARD_STATE_RESET, sdcard.state);
    EXPECT_EQ(0u, cache_getCount());
}

TEST_F(SdcardSdioTest, ResetAfterWriteErrorDropsCachedBlocks)
{
    cacheCount = 5 * 512;
    sdcard.useCache = true;
    sdcard.state = SDCARD_STATE_SENDING_WRITE;
    sdcard.operationStartTime = mockTimeMs;
    mockWriteStatus = SD_ERROR;

    EXPECT_FALSE(sdcard_poll());

    EXPECT_EQ(SDCARD_STATE_RESET, sdcard.state);
    EXPECT_EQ(0u, cache_getCount());
}

TEST_F(SdcardSdioTest, ResetWithCardRemovedDropsCachedBlocks)
{
    // Inverted detect pin: it reads high when the card is out, so sdcard_isInserted() is false.
    sdcard.cardDetectPin = (IO_t)1;
    sdcard.detectionInverted = true;
    mockCardDetectRead = true;
    cacheCount = 5 * 512;
    sdcard.useCache = true;
    sdcard.state = SDCARD_STATE_SENDING_WRITE;
    sdcard.operationStartTime = mockTimeMs - SDCARD_TIMEOUT_WRITE_MSEC - 1;
    mockWriteStatus = SD_BUSY;

    EXPECT_FALSE(sdcard_poll());

    EXPECT_EQ(SDCARD_STATE_NOT_PRESENT, sdcard.state);
    EXPECT_EQ(0u, cache_getCount());
}

TEST_F(SdcardSdioTest, DetectPinHighMeansInsertedWhenNotInverted)
{
    sdcard.cardDetectPin = (IO_t)1;
    sdcard.detectionInverted = false;

    mockCardDetectRead = true;
    EXPECT_TRUE(sdcard_isInserted());
    mockCardDetectRead = false;
    EXPECT_FALSE(sdcard_isInserted());
}

TEST_F(SdcardSdioTest, DetectPinLowMeansInsertedWhenInverted)
{
    sdcard.cardDetectPin = (IO_t)1;
    sdcard.detectionInverted = true;

    mockCardDetectRead = false;
    EXPECT_TRUE(sdcard_isInserted());
    mockCardDetectRead = true;
    EXPECT_FALSE(sdcard_isInserted());
}

TEST_F(SdcardSdioTest, WithoutDetectPinTheCardCountsAsInsertedWhateverTheFlag)
{
    sdcard.cardDetectPin = (IO_t)0;

    sdcard.detectionInverted = false;
    mockCardDetectRead = false;
    EXPECT_TRUE(sdcard_isInserted());
    mockCardDetectRead = true;
    EXPECT_TRUE(sdcard_isInserted());

    sdcard.detectionInverted = true;
    EXPECT_TRUE(sdcard_isInserted());
}

TEST_F(SdcardSdioTest, DetectConfigureCopiesPinAndInvertedFlagFromTheCardConfig)
{
    sdcardConfig_t config;
    std::memset(&config, 0, sizeof(config));
    config.cardDetectTag = 1;
    config.cardDetectInverted = 1;

    sdcardDetectConfigure(&config);

    EXPECT_NE((IO_t)0, sdcard.cardDetectPin);
    EXPECT_TRUE(sdcard.detectionInverted);
    mockCardDetectRead = false;
    EXPECT_TRUE(sdcard_isInserted());
    mockCardDetectRead = true;
    EXPECT_FALSE(sdcard_isInserted());
}

TEST_F(SdcardSdioTest, DetectConfigureWithoutTagClearsPinAndFlag)
{
    sdcard.cardDetectPin = (IO_t)1;
    sdcard.detectionInverted = true;
    sdcardConfig_t config;
    std::memset(&config, 0, sizeof(config));

    sdcardDetectConfigure(&config);

    EXPECT_EQ((IO_t)0, sdcard.cardDetectPin);
    EXPECT_FALSE(sdcard.detectionInverted);
    mockCardDetectRead = true;
    EXPECT_TRUE(sdcard_isInserted());
    mockCardDetectRead = false;
    EXPECT_TRUE(sdcard_isInserted());
}

TEST_F(SdcardSdioTest, InitConfiguresTheDetectPinAsPulledUpInput)
{
    mockDmaSpecAvailable = true;
    sdcardConfig_t config;
    std::memset(&config, 0, sizeof(config));
    config.mode = SDCARD_MODE_SDIO;
    config.cardDetectTag = 1;

    sdcard_init(&config);

    EXPECT_NE((IO_t)0, sdcard.cardDetectPin);
    EXPECT_EQ(1u, ioInitCalls);
    EXPECT_EQ(OWNER_SDCARD_DETECT, lastIoOwner);
    EXPECT_EQ(1u, ioConfigCalls);
    EXPECT_EQ(IOCFG_IPU, lastIoConfig);
}

TEST_F(SdcardSdioTest, InitWithoutDetectTagLeavesTheGpioUntouched)
{
    mockDmaSpecAvailable = true;
    sdcardConfig_t config;
    std::memset(&config, 0, sizeof(config));
    config.mode = SDCARD_MODE_SDIO;

    sdcard_init(&config);

    EXPECT_EQ((IO_t)0, sdcard.cardDetectPin);
    EXPECT_EQ(0u, ioInitCalls);
    EXPECT_EQ(0u, ioConfigCalls);
}

TEST_F(SdcardSdioTest, UnconfiguredDetectorReportsInsertedWithoutReadingThePin)
{
    // The zero state before any configuration: no pin, so the card counts as inserted whatever the pin reads.
    // This is why mass storage must call sdcardDetectConfigure() before it uses sdcard_isInserted().
    mockCardDetectRead = false;
    EXPECT_TRUE(sdcard_isInserted());
    mockCardDetectRead = true;
    EXPECT_TRUE(sdcard_isInserted());
}

TEST_F(SdcardSdioTest, LeavesBusyWritePendingBeforeTimeout)
{
    sdcard.state = SDCARD_STATE_SENDING_WRITE;
    sdcard.operationStartTime = mockTimeMs;
    mockWriteStatus = SD_BUSY;

    EXPECT_FALSE(sdcard_poll());

    EXPECT_EQ(1u, writeCheckCalls);
    EXPECT_EQ(0u, initCalls);
    EXPECT_EQ(SDCARD_STATE_SENDING_WRITE, sdcard.state);
    EXPECT_EQ(0u, callbackCalls);
}

TEST_F(SdcardSdioTest, ReportsFailureForNonBusyWriteError)
{
    sdcard.state = SDCARD_STATE_READY;
    ASSERT_EQ(SDCARD_OPERATION_IN_PROGRESS,
        sdcard_writeBlock(7, buffer, operationComplete, 0));
    mockWriteStatus = SD_ERROR;

    EXPECT_FALSE(sdcard_poll());

    EXPECT_EQ(1u, writeCheckCalls);
    EXPECT_EQ(1u, initCalls);
    EXPECT_EQ(SDCARD_STATE_RESET, sdcard.state);
    EXPECT_EQ(1u, callbackCalls);
    EXPECT_EQ(nullptr, callbackBuffer);
    sdcard_poll();
    EXPECT_EQ(1u, callbackCalls);
}

TEST_F(SdcardSdioTest, SuccessfulReinitializationsStillExhaustRetryBudget)
{
    sdcard.state = SDCARD_STATE_CARD_INIT_IN_PROGRESS;
    mockCardReady = false;

    for (unsigned i = 0; i < SDCARD_MAX_CONSECUTIVE_FAILURES; i++) {
        sdcard.operationStartTime = mockTimeMs;
        mockTimeMs += SDCARD_TIMEOUT_INIT_MILLIS + 1;
        EXPECT_FALSE(sdcard_poll());
        EXPECT_EQ(i + 1, initCalls);
        EXPECT_EQ(i + 1, sdcard.failureCount);
    }
    EXPECT_EQ(SDCARD_STATE_NOT_PRESENT, sdcard.state);
}

TEST_F(SdcardSdioTest, RepeatedTransferFailuresDisableCardWhenInitSucceeds)
{
    for (unsigned i = 0; i < SDCARD_MAX_CONSECUTIVE_FAILURES; i++) {
        sdcard.state = SDCARD_STATE_READY;
        ASSERT_EQ(SDCARD_OPERATION_IN_PROGRESS,
            sdcard_writeBlock(7, buffer, operationComplete, 0));
        mockWriteStatus = SD_ERROR;

        EXPECT_FALSE(sdcard_poll());
        EXPECT_EQ(i + 1, initCalls);
        EXPECT_EQ(i + 1, sdcard.failureCount);
        mockWriteStatus = SD_OK;
    }
    EXPECT_EQ(SDCARD_STATE_NOT_PRESENT, sdcard.state);
    EXPECT_FALSE(sdcard_isFunctional());
}

TEST_F(SdcardSdioTest, CompletedReadClearsFailureCount)
{
    sdcard.state = SDCARD_STATE_READING;
    sdcard.operationStartTime = mockTimeMs;
    sdcard.failureCount = SDCARD_MAX_CONSECUTIVE_FAILURES - 1;

    EXPECT_TRUE(sdcard_poll());

    EXPECT_EQ(SDCARD_STATE_READY, sdcard.state);
    EXPECT_EQ(0, sdcard.failureCount);
}

TEST_F(SdcardSdioTest, ConsecutiveFailedReinitializationsDisableCard)
{
    sdcard.state = SDCARD_STATE_CARD_INIT_IN_PROGRESS;
    mockCardReady = false;
    mockInitStatus = SD_ERROR;

    for (unsigned i = 0; i < SDCARD_MAX_CONSECUTIVE_FAILURES; i++) {
        sdcard.operationStartTime = mockTimeMs;
        mockTimeMs += SDCARD_TIMEOUT_INIT_MILLIS + 1;
        EXPECT_FALSE(sdcard_poll());
        EXPECT_EQ(i + 1, initCalls);
        EXPECT_EQ(i + 1, sdcard.failureCount);
    }
    EXPECT_EQ(SDCARD_STATE_NOT_PRESENT, sdcard.state);
}

TEST_F(SdcardSdioTest, FailedCsdReadRetriesOnNextPoll)
{
    sdcard.state = SDCARD_STATE_CARD_INIT_IN_PROGRESS;
    sdcard.operationStartTime = mockTimeMs;
    failCsdRead = true;

    EXPECT_FALSE(sdcard_poll());
    EXPECT_EQ(2u, cardInfoCalls);
    EXPECT_EQ(1u, initCalls);
    EXPECT_EQ(SDCARD_STATE_RESET, sdcard.state);
    EXPECT_EQ(1, sdcard.failureCount);

    failCsdRead = false;
    EXPECT_TRUE(sdcard_poll());
    EXPECT_EQ(SDCARD_STATE_READY, sdcard.state);
}
