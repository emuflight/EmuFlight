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

// Compiles the real src/main/drivers/dma_stm32f4xx.c under UNIT_TEST, mocking only the
// register-level surface it touches (RCC_AHB1PeriphClockCmd, NVIC_Init) -- unlike
// dma_bounds_unittest.cc, which mirrors dmaIdentifierIsValid() locally rather than linking
// the production file. dmaDescriptors[] is file-static inside the real .c file (no reset
// hook exists), so each TEST() below claims its own dedicated, never-reused DMA identifier
// instead of resetting shared state between cases.

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
void resetRegisterMockCounters() {
    nvicInitCallCount = 0;
    rccClockCmdCallCount = 0;
    lastRccPeriph = 0;
}
} // namespace

// --- dmaAllocate(): real ownership-check logic from the production file ---

TEST(DmaStm32F4xxUnittest, AllocateOnFreeStreamSucceedsAndRecordsOwner)
{
    const dmaIdentifier_e id = DMA1_ST2_HANDLER;
    ASSERT_EQ(dmaGetOwner(id), OWNER_FREE);

    EXPECT_TRUE(dmaAllocate(id, OWNER_SPI_SDO, 3));
    EXPECT_EQ(dmaGetOwner(id), OWNER_SPI_SDO);
    EXPECT_EQ(dmaGetResourceIndex(id), 3);
}

TEST(DmaStm32F4xxUnittest, DoubleAllocateSameStreamByDifferentOwnerFails)
{
    const dmaIdentifier_e id = DMA1_ST3_HANDLER;
    ASSERT_TRUE(dmaAllocate(id, OWNER_SPI_SDI, 1));

    // adversarial: a second, different owner must not steal an already-claimed stream.
    EXPECT_FALSE(dmaAllocate(id, OWNER_SERIAL_RX, 2));
    EXPECT_EQ(dmaGetOwner(id), OWNER_SPI_SDI); // original claim untouched
    EXPECT_EQ(dmaGetResourceIndex(id), 1);
}

TEST(DmaStm32F4xxUnittest, AllocateRejectsInvalidIdentifier)
{
    // adversarial: DMA_NONE and past-the-end identifiers must not index dmaDescriptors[].
    EXPECT_FALSE(dmaAllocate(DMA_NONE, OWNER_SPI_SDO, 0));
    EXPECT_FALSE(dmaAllocate(static_cast<dmaIdentifier_e>(DMA_LAST_HANDLER + 1), OWNER_SPI_SDO, 0));
}

// --- dmaGetIdentifier() / dmaGetDescriptorByIdentifier(): real stream-table lookup ---

TEST(DmaStm32F4xxUnittest, GetIdentifierResolvesRealStreamPointer)
{
    EXPECT_EQ(dmaGetIdentifier(DMA1_Stream4), DMA1_ST4_HANDLER);
    EXPECT_EQ(dmaGetIdentifier(DMA2_Stream6), DMA2_ST6_HANDLER);
}

TEST(DmaStm32F4xxUnittest, GetIdentifierRejectsUnknownStreamPointer)
{
    DMA_Stream_TypeDef unknownStream;
    EXPECT_EQ(dmaGetIdentifier(&unknownStream), DMA_NONE);
}

TEST(DmaStm32F4xxUnittest, GetDescriptorByIdentifierRejectsInvalidIdentifier)
{
    EXPECT_EQ(dmaGetDescriptorByIdentifier(DMA_NONE), nullptr);
    EXPECT_EQ(dmaGetDescriptorByIdentifier(static_cast<dmaIdentifier_e>(DMA_LAST_HANDLER + 1)), nullptr);
}

TEST(DmaStm32F4xxUnittest, GetDescriptorByIdentifierReturnsMatchingStream)
{
    dmaChannelDescriptor_t *descriptor = dmaGetDescriptorByIdentifier(DMA1_ST5_HANDLER);
    ASSERT_NE(descriptor, nullptr);
    EXPECT_EQ(descriptor->ref, DMA1_Stream5);
    EXPECT_EQ(descriptor->dma, DMA1);
}

// --- dmaEnable() / dmaSetHandler(): real functions must reach the mocked registers ---

TEST(DmaStm32F4xxUnittest, EnableOnValidIdentifierClocksTheControllerOnce)
{
    resetRegisterMockCounters();
    dmaEnable(DMA1_ST6_HANDLER);
    EXPECT_EQ(rccClockCmdCallCount, 1);
    EXPECT_EQ(lastRccPeriph, RCC_AHB1Periph_DMA1);
}

TEST(DmaStm32F4xxUnittest, EnableOnInvalidIdentifierNeverTouchesRegisters)
{
    // adversarial: guard must reject before the register write, not after.
    resetRegisterMockCounters();
    dmaEnable(DMA_NONE);
    dmaEnable(static_cast<dmaIdentifier_e>(DMA_LAST_HANDLER + 1));
    EXPECT_EQ(rccClockCmdCallCount, 0);
}

TEST(DmaStm32F4xxUnittest, SetHandlerRegistersCallbackAndEnablesInterrupt)
{
    resetRegisterMockCounters();
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
    resetRegisterMockCounters();
    dmaSetHandler(DMA_NONE, nullptr, 0, 0);
    EXPECT_EQ(nvicInitCallCount, 0);
    EXPECT_EQ(rccClockCmdCallCount, 0);
}

// --- dmaInit(): documents its real, unchecked overwrite -- unlike dmaAllocate() above ---

TEST(DmaStm32F4xxUnittest, InitOverwritesExistingOwnerWithNoOwnershipCheck)
{
    const dmaIdentifier_e id = DMA1_ST7_HANDLER;
    ASSERT_TRUE(dmaAllocate(id, OWNER_SPI_SDO, 1));

    dmaInit(id, OWNER_SERIAL_TX, 4); // no ownership check, unlike dmaAllocate() above

    EXPECT_EQ(dmaGetOwner(id), OWNER_SERIAL_TX);
    EXPECT_EQ(dmaGetResourceIndex(id), 4);
}
