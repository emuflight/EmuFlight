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

// Compiles the real dma_stm32f4xx.c under UNIT_TEST, mocking only its register surface.
// resetDmaTestState() clears dmaDescriptors[] via a UNIT_TEST-only hook added for this file.

extern "C" {

#include "platform.h"
#include "drivers/dma.h"

static int nvicInitCallCount;
static int rccClockCmdCallCount;
static uint32_t lastRccPeriph;

void NVIC_Init(NVIC_InitTypeDef *initStruct) {
    (void)initStruct;
    nvicInitCallCount++;
}

void RCC_AHB1PeriphClockCmd(uint32_t periph, FunctionalState state) {
    (void)state;
    rccClockCmdCallCount++;
    lastRccPeriph = periph;
}

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

namespace {
void resetDmaTestState() {
    dmaResetAllocationsForTest();
    nvicInitCallCount = 0;
    rccClockCmdCallCount = 0;
    lastRccPeriph = 0;
}
} // namespace

// --- dmaAllocate(): real ownership-check logic from the production file ---

TEST(DmaStm32F4xxUnittest, AllocateOnFreeStreamSucceedsAndRecordsOwner)
{
    resetDmaTestState();
    const dmaIdentifier_e id = DMA1_ST2_HANDLER;
    ASSERT_EQ(dmaGetOwner(id), OWNER_FREE);

    EXPECT_TRUE(dmaAllocate(id, OWNER_SPI_SDO, 3));
    EXPECT_EQ(dmaGetOwner(id), OWNER_SPI_SDO);
    EXPECT_EQ(dmaGetResourceIndex(id), 3);
}

TEST(DmaStm32F4xxUnittest, DoubleAllocateSameStreamByDifferentOwnerFails)
{
    resetDmaTestState();
    const dmaIdentifier_e id = DMA1_ST3_HANDLER;
    ASSERT_TRUE(dmaAllocate(id, OWNER_SPI_SDI, 1));

    // adversarial: a second, different owner must not steal an already-claimed stream.
    EXPECT_FALSE(dmaAllocate(id, OWNER_SERIAL_RX, 2));
    EXPECT_EQ(dmaGetOwner(id), OWNER_SPI_SDI); // original claim untouched
    EXPECT_EQ(dmaGetResourceIndex(id), 1);
}

TEST(DmaStm32F4xxUnittest, AllocateRejectsInvalidIdentifier)
{
    resetDmaTestState();
    // adversarial: DMA_NONE and past-the-end identifiers must not index dmaDescriptors[].
    EXPECT_FALSE(dmaAllocate(DMA_NONE, OWNER_SPI_SDO, 0));
    EXPECT_FALSE(dmaAllocate(static_cast<dmaIdentifier_e>(DMA_LAST_HANDLER + 1), OWNER_SPI_SDO, 0));
}

// --- dmaGetIdentifier() / dmaGetDescriptorByIdentifier(): real stream-table lookup ---

TEST(DmaStm32F4xxUnittest, GetIdentifierResolvesRealStreamPointer)
{
    resetDmaTestState();
    EXPECT_EQ(dmaGetIdentifier(DMA1_Stream4), DMA1_ST4_HANDLER);
    EXPECT_EQ(dmaGetIdentifier(DMA2_Stream6), DMA2_ST6_HANDLER);
}

TEST(DmaStm32F4xxUnittest, GetIdentifierRejectsUnknownStreamPointer)
{
    resetDmaTestState();
    DMA_Stream_TypeDef unknownStream = {};
    EXPECT_EQ(dmaGetIdentifier(&unknownStream), DMA_NONE);
}

TEST(DmaStm32F4xxUnittest, GetDescriptorByIdentifierRejectsInvalidIdentifier)
{
    resetDmaTestState();
    EXPECT_EQ(dmaGetDescriptorByIdentifier(DMA_NONE), nullptr);
    EXPECT_EQ(dmaGetDescriptorByIdentifier(static_cast<dmaIdentifier_e>(DMA_LAST_HANDLER + 1)), nullptr);
}

TEST(DmaStm32F4xxUnittest, GetDescriptorByIdentifierReturnsMatchingStream)
{
    resetDmaTestState();
    dmaChannelDescriptor_t *descriptor = dmaGetDescriptorByIdentifier(DMA1_ST5_HANDLER);
    ASSERT_NE(descriptor, nullptr);
    EXPECT_EQ(descriptor->ref, DMA1_Stream5);
    EXPECT_EQ(descriptor->dma, DMA1);
}

// --- dmaEnable() / dmaSetHandler(): real functions must reach the mocked registers ---

TEST(DmaStm32F4xxUnittest, EnableOnValidIdentifierClocksTheControllerOnce)
{
    resetDmaTestState();
    dmaEnable(DMA1_ST6_HANDLER);
    EXPECT_EQ(rccClockCmdCallCount, 1);
    EXPECT_EQ(lastRccPeriph, RCC_AHB1Periph_DMA1);
}

TEST(DmaStm32F4xxUnittest, EnableOnInvalidIdentifierNeverTouchesRegisters)
{
    // adversarial: guard must reject before the register write, not after.
    resetDmaTestState();
    dmaEnable(DMA_NONE);
    dmaEnable(static_cast<dmaIdentifier_e>(DMA_LAST_HANDLER + 1));
    EXPECT_EQ(rccClockCmdCallCount, 0);
}

TEST(DmaStm32F4xxUnittest, SetHandlerRegistersCallbackAndEnablesInterrupt)
{
    resetDmaTestState();
    const dmaIdentifier_e id = DMA2_ST0_HANDLER;
    dmaChannelDescriptor_t *descriptor = dmaGetDescriptorByIdentifier(id);
    ASSERT_NE(descriptor, nullptr);

    dmaSetHandler(id, nullptr, 0, 0x1234);

    EXPECT_EQ(nvicInitCallCount, 1);
    EXPECT_EQ(rccClockCmdCallCount, 1);
    EXPECT_EQ(lastRccPeriph, RCC_AHB1Periph_DMA2);
    EXPECT_EQ(descriptor->userParam, (uintptr_t)0x1234);
}

TEST(DmaStm32F4xxUnittest, SetHandlerOnInvalidIdentifierNeverTouchesRegisters)
{
    // adversarial: same guard as dmaEnable(), exercised via the other register-touching entry point.
    resetDmaTestState();
    dmaSetHandler(DMA_NONE, nullptr, 0, 0);
    EXPECT_EQ(nvicInitCallCount, 0);
    EXPECT_EQ(rccClockCmdCallCount, 0);
}

// --- dmaInit(): documents its real, unchecked overwrite -- unlike dmaAllocate() above ---

TEST(DmaStm32F4xxUnittest, InitOverwritesExistingOwnerWithNoOwnershipCheck)
{
    resetDmaTestState();
    const dmaIdentifier_e id = DMA1_ST7_HANDLER;
    ASSERT_TRUE(dmaAllocate(id, OWNER_SPI_SDO, 1));

    dmaInit(id, OWNER_SERIAL_TX, 4); // no ownership check, unlike dmaAllocate() above

    EXPECT_EQ(dmaGetOwner(id), OWNER_SERIAL_TX);
    EXPECT_EQ(dmaGetResourceIndex(id), 4);
}
