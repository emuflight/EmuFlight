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
#include "drivers/dma_reqmap.h"
#include "drivers/sdmmc_sdio.h"
#include "pg/sdio.h"
#include "msc/usbd_storage.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// Compiles the real msc/usbd_storage_sdio.c. A DMA transfer that never completes must end the
// USB read or write with an error. Bounds scale with the block count; SD spec minimums (physical
// layer spec): read 100 ms, write 250 ms per block.
namespace {

constexpr uint32_t SPEC_READ_MS_PER_BLOCK = 100;
constexpr uint32_t SPEC_WRITE_MS_PER_BLOCK = 250;
constexpr uint32_t MAX_CALLS = 10000000;

uint32_t fakeNowMs;
uint32_t checkCalls;
uint32_t checkBusyCalls;    // SD_Check* reports busy this many times, then done
bool stateReady;            // SD_GetState result
bool cardInserted = true;

void setUpFakeCard(uint32_t startMs) {
    cardInserted = true;
    fakeNowMs = startMs;
    checkCalls = 0;
    checkBusyCalls = UINT32_MAX;
    stateReady = false;
}

uint32_t elapsed(uint32_t startMs) {
    return fakeNowMs - startMs;
}

SD_Error_t fakeCheck() {
    return (checkCalls++ < checkBusyCalls) ? SD_BUSY : SD_OK;
}

}  // namespace

extern "C" {

sdioConfig_t sdioConfig_System;

uint32_t millis(void) {
    return fakeNowMs++;
}

static dmaChannelSpec_t fakeDmaSpec;
const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e, uint8_t, int8_t) { return &fakeDmaSpec; }
bool SD_Initialize_LL(DMA_Stream_TypeDef *) { return true; }
SD_Error_t SD_Init(void) { return SD_OK; }
SD_CardInfo_t SD_CardInfo;
SD_Error_t SD_GetCardInfo(void) { return SD_OK; }
bool sdcard_isInserted(void) { return cardInserted; }
bool SD_GetState(void) { return stateReady; }
SD_Error_t SD_CheckRead(void) { return fakeCheck(); }
SD_Error_t SD_CheckWrite(void) { return fakeCheck(); }
SD_Error_t SD_ReadBlocks_DMA(uint64_t, uint32_t *, uint32_t, uint32_t) { return SD_OK; }
SD_Error_t SD_WriteBlocks_DMA(uint64_t, uint32_t *, uint32_t, uint32_t) { return SD_OK; }

}

TEST(MscSdioTest, BackendServesSdioMode) {
    // mscStart() serves SD storage only when `sdcard_mode` equals this backend's mode.
    EXPECT_EQ(SDCARD_MODE_SDIO, USBD_MSC_MICRO_SD_MODE);
    EXPECT_NE(SDCARD_MODE_NONE, USBD_MSC_MICRO_SD_MODE);
    EXPECT_NE(SDCARD_MODE_SPI, USBD_MSC_MICRO_SD_MODE);
}

TEST(MscSdioTest, ReadReturnsErrorWhenTransferNeverCompletes) {
    const uint32_t start = 0;
    const uint16_t blocks = 8;
    setUpFakeCard(start);
    uint8_t buf[512 * 8];
    EXPECT_EQ(-1, USBD_MSC_MICRO_SDIO_fops.Read(0, buf, 0, blocks));
    EXPECT_GE(elapsed(start), SPEC_READ_MS_PER_BLOCK * blocks);
    EXPECT_LT(checkCalls, MAX_CALLS);
}

TEST(MscSdioTest, ReadReturnsErrorWhenCardStateNeverReady) {
    const uint32_t start = 0;
    setUpFakeCard(start);
    checkBusyCalls = 0;
    uint8_t buf[512];
    EXPECT_EQ(-1, USBD_MSC_MICRO_SDIO_fops.Read(0, buf, 0, 1));
    EXPECT_GE(elapsed(start), SPEC_READ_MS_PER_BLOCK);
}

TEST(MscSdioTest, ReadSucceedsWhenTransferCompletes) {
    setUpFakeCard(0);
    checkBusyCalls = 50;
    stateReady = true;
    uint8_t buf[512];
    EXPECT_EQ(0, USBD_MSC_MICRO_SDIO_fops.Read(0, buf, 0, 1));
}

TEST(MscSdioTest, WriteReturnsErrorWhenTransferNeverCompletes) {
    const uint32_t start = UINT32_MAX - 10;
    const uint16_t blocks = 4;
    setUpFakeCard(start);
    uint8_t buf[512 * 4] = {};
    EXPECT_EQ(-1, USBD_MSC_MICRO_SDIO_fops.Write(0, buf, 0, blocks));
    EXPECT_GE(elapsed(start), SPEC_WRITE_MS_PER_BLOCK * blocks);
}

TEST(MscSdioTest, WriteReturnsErrorWhenCardStateNeverReady) {
    const uint32_t start = 0;
    setUpFakeCard(start);
    checkBusyCalls = 0;
    uint8_t buf[512] = {};
    EXPECT_EQ(-1, USBD_MSC_MICRO_SDIO_fops.Write(0, buf, 0, 1));
    EXPECT_GE(elapsed(start), SPEC_WRITE_MS_PER_BLOCK);
}

TEST(MscSdioTest, WriteSucceedsWhenTransferCompletes) {
    setUpFakeCard(0);
    checkBusyCalls = 50;
    stateReady = true;
    uint8_t buf[512] = {};
    EXPECT_EQ(0, USBD_MSC_MICRO_SDIO_fops.Write(0, buf, 0, 1));
}

TEST(MscSdioTest, EveryOperationFailsWhenNoCardIsInserted) {
    // Card removed: capacity, ready, read and write all report an error, none touches the card.
    setUpFakeCard(0);
    cardInserted = false;
    stateReady = true;
    uint32_t blocks = 123;
#ifdef USE_HAL_DRIVER
    uint16_t size = 77;
#else
    uint32_t size = 77;
#endif
    uint8_t buf[512];
    EXPECT_EQ(-1, USBD_MSC_MICRO_SDIO_fops.GetCapacity(0, &blocks, &size));
    EXPECT_EQ(123u, blocks);
    EXPECT_EQ(77u, size);
    EXPECT_EQ(-1, USBD_MSC_MICRO_SDIO_fops.IsReady(0));
    EXPECT_EQ(-1, USBD_MSC_MICRO_SDIO_fops.Read(0, buf, 0, 1));
    EXPECT_EQ(-1, USBD_MSC_MICRO_SDIO_fops.Write(0, buf, 0, 1));
    EXPECT_EQ(0u, checkCalls);
}

TEST(MscSdioTest, CapacityAndReadyAreServedWhenCardIsInserted) {
    setUpFakeCard(0);
    cardInserted = true;
    stateReady = true;
    SD_CardInfo.CardCapacity = 4096;
    uint32_t blocks = 0;
#ifdef USE_HAL_DRIVER
    uint16_t size = 0;
#else
    uint32_t size = 0;
#endif
    EXPECT_EQ(0, USBD_MSC_MICRO_SDIO_fops.GetCapacity(0, &blocks, &size));
    EXPECT_EQ(4096u, blocks);
    EXPECT_EQ(512u, size);
    EXPECT_EQ(0, USBD_MSC_MICRO_SDIO_fops.IsReady(0));
}

TEST(MscSdioTest, InitFailsWithoutACardAndSucceedsWithOne) {
    setUpFakeCard(0);
    cardInserted = false;
    EXPECT_EQ(1, USBD_MSC_MICRO_SDIO_fops.Init(0));
    cardInserted = true;
    EXPECT_EQ(0, USBD_MSC_MICRO_SDIO_fops.Init(0));
}
