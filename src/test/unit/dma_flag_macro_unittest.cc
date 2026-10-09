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
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// Checks DMA_CLEAR_FLAG / DMA_GET_FLAG_STATUS with single and OR-ed flag masks.
// Expected values come from the F4/F7/H7 DMA register layout: stream n owns a 6-bit field
// {FEIF, -, DMEIF, TEIF, HTIF, TCIF} at bit offset 0, 6, 16, 22 (streams 0-3, low register)
// and the same offsets again in the high register (streams 4-7).

extern "C" {

#include "platform.h"
#include "drivers/dma.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

namespace {

const uint32_t kStreamShift[4] = { 0, 6, 16, 22 };

// A mask that mirrors the hardware: every flag of the stream, at the stream's own bit offset.
uint32_t expectedMask(uint32_t flags, unsigned shift) {
    uint32_t result = 0;
    for (unsigned bit = 0; bit < 6; bit++) {
        if (flags & (1u << bit)) {
            result |= 1u << (bit + shift);
        }
    }
    return result;
}

dmaChannelDescriptor_t makeDescriptor(DMA_TypeDef *regs, uint8_t flagsShift) {
    dmaChannelDescriptor_t d = {};
    d.dma = regs;
    d.flagsShift = flagsShift;
    return d;
}

} // namespace

TEST(DmaFlagMacroUnittest, ClearOredMaskShiftsEveryFlagOntoItsOwnStream)
{
    const uint32_t mask = DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF; // the SPI driver call
    for (unsigned i = 0; i < 4; i++) {
        DMA_TypeDef regs = {};
        dmaChannelDescriptor_t d = makeDescriptor(&regs, kStreamShift[i]);
        dmaChannelDescriptor_t *dp = &d;
        DMA_CLEAR_FLAG(dp, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
        EXPECT_EQ(expectedMask(mask, kStreamShift[i]), regs.LIFCR) << "low stream " << i;
        EXPECT_EQ(0u, regs.HIFCR);
    }
}

TEST(DmaFlagMacroUnittest, ClearOredMaskUsesHighRegisterForStreams4To7)
{
    const uint32_t mask = DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF;
    for (unsigned i = 0; i < 4; i++) {
        DMA_TypeDef regs = {};
        dmaChannelDescriptor_t d = makeDescriptor(&regs, 32 + kStreamShift[i]);
        dmaChannelDescriptor_t *dp = &d;
        DMA_CLEAR_FLAG(dp, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
        EXPECT_EQ(expectedMask(mask, kStreamShift[i]), regs.HIFCR) << "high stream " << i;
        EXPECT_EQ(0u, regs.LIFCR);
    }
}

TEST(DmaFlagMacroUnittest, ClearOredMaskLeavesNeighbourStreamBitsUntouched)
{
    // Stream 3 (shift 22): bits 0-5 belong to stream 0 and must stay zero.
    DMA_TypeDef regs = {};
    dmaChannelDescriptor_t d = makeDescriptor(&regs, 22);
    dmaChannelDescriptor_t *dp = &d;
    DMA_CLEAR_FLAG(dp, DMA_IT_TCIF | DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF);
    EXPECT_EQ(0u, regs.LIFCR & 0x3Fu);
    EXPECT_EQ(0x0F400000u, regs.LIFCR); // TCIF3|HTIF3|TEIF3|DMEIF3|FEIF3 per the register map
}

TEST(DmaFlagMacroUnittest, ClearSingleFlagUnchanged)
{
    DMA_TypeDef regs = {};
    dmaChannelDescriptor_t d = makeDescriptor(&regs, 16);
    dmaChannelDescriptor_t *dp = &d;
    DMA_CLEAR_FLAG(dp, DMA_IT_TCIF);
    EXPECT_EQ(0x00200000u, regs.LIFCR);
}

TEST(DmaFlagMacroUnittest, ClearIsOneStatementInDanglingElseContext)
{
    DMA_TypeDef regs = {};
    dmaChannelDescriptor_t d = makeDescriptor(&regs, 6);
    dmaChannelDescriptor_t *dp = &d;
    const bool cond = false;
    int elseTaken = 0;
    if (cond)
        DMA_CLEAR_FLAG(dp, DMA_IT_TCIF);
    else
        elseTaken = 1;
    EXPECT_EQ(1, elseTaken);
    EXPECT_EQ(0u, regs.LIFCR);
}

TEST(DmaFlagMacroUnittest, GetStatusOredMaskTestsEveryFlagOnItsOwnStream)
{
    DMA_TypeDef regs = {};
    dmaChannelDescriptor_t d = makeDescriptor(&regs, 22);
    dmaChannelDescriptor_t *dp = &d;
    regs.LISR = 0x00400000u; // only FEIF of stream 3
    EXPECT_NE(0u, DMA_GET_FLAG_STATUS(dp, DMA_IT_FEIF | DMA_IT_TEIF));
    EXPECT_EQ(0u, DMA_GET_FLAG_STATUS(dp, DMA_IT_TCIF | DMA_IT_HTIF));
    regs.LISR = 0x0000003Fu; // all flags of stream 0 only
    EXPECT_EQ(0u, DMA_GET_FLAG_STATUS(dp, DMA_IT_TCIF | DMA_IT_HTIF | DMA_IT_TEIF));
}

TEST(DmaFlagMacroUnittest, GetStatusHighRegisterOredMask)
{
    DMA_TypeDef regs = {};
    dmaChannelDescriptor_t d = makeDescriptor(&regs, 54);
    dmaChannelDescriptor_t *dp = &d;
    regs.HISR = 0x02000000u; // TEIF of stream 7 (bit 25)
    EXPECT_NE(0u, DMA_GET_FLAG_STATUS(dp, DMA_IT_HTIF | DMA_IT_TEIF));
    regs.HISR = 0x0000003Fu; // stream 4 only
    EXPECT_EQ(0u, DMA_GET_FLAG_STATUS(dp, DMA_IT_HTIF | DMA_IT_TEIF));
}
