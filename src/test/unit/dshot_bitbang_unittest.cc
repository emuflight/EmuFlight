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
#include <string.h>

extern "C" {

#include "platform.h"
#include "drivers/dshot_bitbang.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

namespace {

// BSRR layout: low half sets pins, high half resets pins.
constexpr int WORDS = MOTOR_DSHOT_BUF_LENGTH;
constexpr int HOLD = MOTOR_DSHOT_FRAME_BITS * MOTOR_DSHOT_STATE_PER_SYMBOL;

uint32_t setWord(uint16_t pins) { return pins; }
uint32_t resetWord(uint16_t pins) { return (uint32_t)pins << 16; }

// DShot 600: 16 bits at 3 states each, plus 3 hold states.
TEST(DshotBitbangTest, BufferLengthIsFrameAndHold)
{
    EXPECT_EQ(51, WORDS);
}

TEST(DshotBitbangTest, OnlyDshot150300600AreActiveWhenOn)
{
    const uint8_t supported[] = {PWM_TYPE_DSHOT150, PWM_TYPE_DSHOT300, PWM_TYPE_DSHOT600};
    for (uint8_t protocol : supported) {
        EXPECT_TRUE(dshotBitbangIsActiveFor(DSHOT_BITBANG_ON, protocol)) << (int)protocol;
    }
    const uint8_t classicOnly[] = {PWM_TYPE_DSHOT1200, PWM_TYPE_DSHOT2400, PWM_TYPE_DSHOT4800, PWM_TYPE_PROSHOT1000,
                                   PWM_TYPE_STANDARD, PWM_TYPE_ONESHOT125, PWM_TYPE_ONESHOT42, PWM_TYPE_MULTISHOT, PWM_TYPE_BRUSHED};
    for (uint8_t protocol : classicOnly) {
        EXPECT_FALSE(dshotBitbangIsActiveFor(DSHOT_BITBANG_ON, protocol)) << (int)protocol;
    }
}

TEST(DshotBitbangTest, OffAndAutoNeverActivate)
{
    const uint8_t protocols[] = {PWM_TYPE_DSHOT150, PWM_TYPE_DSHOT300, PWM_TYPE_DSHOT600, PWM_TYPE_DSHOT1200, PWM_TYPE_PROSHOT1000};
    for (uint8_t protocol : protocols) {
        EXPECT_FALSE(dshotBitbangIsActiveFor(DSHOT_BITBANG_OFF, protocol)) << (int)protocol;
        EXPECT_FALSE(dshotBitbangIsActiveFor(DSHOT_BITBANG_AUTO, protocol)) << (int)protocol;
    }
}

TEST(DshotBitbangTest, OutOfRangeValuesDoNotActivate)
{
    EXPECT_FALSE(dshotBitbangIsActiveFor(255, PWM_TYPE_DSHOT600));
    EXPECT_FALSE(dshotBitbangIsActiveFor(DSHOT_BITBANG_ON, 255));
    EXPECT_FALSE(dshotBitbangIsActiveFor(DSHOT_BITBANG_ON, PWM_TYPE_MAX));
}

TEST(DshotBitbangTest, InitSetsAndResetsEverySymbolAndHoldsIdle)
{
    uint32_t buffer[WORDS];
    memset(buffer, 0, sizeof(buffer));
    dshotBitbangOutputDataInit(buffer, 0x0005); // pins 0 and 2

    for (int bit = 0; bit < MOTOR_DSHOT_FRAME_BITS; bit++) {
        EXPECT_EQ(setWord(0x0005), buffer[bit * 3 + 0]) << bit;
        EXPECT_EQ(0u, buffer[bit * 3 + 1]) << bit;
        EXPECT_EQ(resetWord(0x0005), buffer[bit * 3 + 2]) << bit;
    }
    EXPECT_EQ(resetWord(0x0005), buffer[HOLD + 0]);
    EXPECT_EQ(0u, buffer[HOLD + 1]);
    EXPECT_EQ(0u, buffer[HOLD + 2]);
}

// A 1 bit keeps the line high through the middle state, a 0 bit drops it there.
void expectMiddleWords(const uint32_t *buffer, uint16_t packet, uint16_t pin)
{
    for (int pos = 0; pos < MOTOR_DSHOT_FRAME_BITS; pos++) {
        const bool bitIsOne = (packet >> (15 - pos)) & 1;
        const uint32_t expected = bitIsOne ? 0u : resetWord(1u << pin);
        EXPECT_EQ(expected, buffer[pos * 3 + 1]) << "pos " << pos << " packet " << packet;
    }
}

TEST(DshotBitbangTest, SetEncodesPacketMsbFirst)
{
    const uint16_t packets[] = {0x0000, 0xFFFF, 0x8000, 0x0001, 0xAAAA, 0x5555, 0xA5C3};
    for (uint16_t packet : packets) {
        uint32_t buffer[WORDS];
        memset(buffer, 0, sizeof(buffer));
        dshotBitbangOutputDataInit(buffer, 1u << 3);
        dshotBitbangOutputDataSet(buffer, 3, packet);
        expectMiddleWords(buffer, packet, 3);
    }
}

TEST(DshotBitbangTest, AllZeroPacketResetsEveryMiddleWord)
{
    uint32_t buffer[WORDS];
    memset(buffer, 0, sizeof(buffer));
    dshotBitbangOutputDataInit(buffer, 1);
    dshotBitbangOutputDataSet(buffer, 0, 0x0000);
    for (int pos = 0; pos < MOTOR_DSHOT_FRAME_BITS; pos++) {
        EXPECT_EQ(resetWord(1), buffer[pos * 3 + 1]);
    }
}

TEST(DshotBitbangTest, HighestPinUsesBit31AndDoesNotWrap)
{
    uint32_t buffer[WORDS];
    memset(buffer, 0, sizeof(buffer));
    dshotBitbangOutputDataInit(buffer, 0x8000);
    dshotBitbangOutputDataSet(buffer, 15, 0x0000);
    for (int pos = 0; pos < MOTOR_DSHOT_FRAME_BITS; pos++) {
        EXPECT_EQ(0x80000000u, buffer[pos * 3 + 1]) << pos;
        EXPECT_EQ(0x00008000u, buffer[pos * 3 + 0]) << pos;
    }
}

TEST(DshotBitbangTest, PinsOfOnePortMergeIndependently)
{
    uint32_t buffer[WORDS];
    memset(buffer, 0, sizeof(buffer));
    dshotBitbangOutputDataInit(buffer, 0x0001);
    dshotBitbangOutputDataInit(buffer, 0x0002);
    dshotBitbangOutputDataSet(buffer, 0, 0xFFFF); // pin 0 sends only 1 bits
    dshotBitbangOutputDataSet(buffer, 1, 0x0000); // pin 1 sends only 0 bits

    for (int pos = 0; pos < MOTOR_DSHOT_FRAME_BITS; pos++) {
        EXPECT_EQ(setWord(0x0003), buffer[pos * 3 + 0]);
        EXPECT_EQ(resetWord(0x0002), buffer[pos * 3 + 1]);
        EXPECT_EQ(resetWord(0x0003), buffer[pos * 3 + 2]);
    }
}

TEST(DshotBitbangTest, AllSixteenPinsOfAPortFit)
{
    uint32_t buffer[WORDS];
    memset(buffer, 0, sizeof(buffer));
    dshotBitbangOutputDataInit(buffer, 0xFFFF);
    for (int pin = 0; pin < 16; pin++) {
        dshotBitbangOutputDataSet(buffer, pin, 0x0000);
    }
    for (int pos = 0; pos < MOTOR_DSHOT_FRAME_BITS; pos++) {
        EXPECT_EQ(resetWord(0xFFFF), buffer[pos * 3 + 1]);
    }
}

TEST(DshotBitbangTest, ClearRestoresNoChangeButKeepsSetAndReset)
{
    uint32_t buffer[WORDS];
    memset(buffer, 0, sizeof(buffer));
    dshotBitbangOutputDataInit(buffer, 0x0010);
    dshotBitbangOutputDataSet(buffer, 4, 0x0000);
    dshotBitbangOutputDataClear(buffer);

    for (int pos = 0; pos < MOTOR_DSHOT_FRAME_BITS; pos++) {
        EXPECT_EQ(setWord(0x0010), buffer[pos * 3 + 0]);
        EXPECT_EQ(0u, buffer[pos * 3 + 1]);
        EXPECT_EQ(resetWord(0x0010), buffer[pos * 3 + 2]);
    }
    EXPECT_EQ(resetWord(0x0010), buffer[HOLD]);
}

TEST(DshotBitbangTest, SetNeverTouchesHoldStates)
{
    uint32_t buffer[WORDS];
    memset(buffer, 0, sizeof(buffer));
    dshotBitbangOutputDataInit(buffer, 0x0001);
    dshotBitbangOutputDataSet(buffer, 0, 0x0000);
    EXPECT_EQ(resetWord(0x0001), buffer[HOLD + 0]);
    EXPECT_EQ(0u, buffer[HOLD + 1]);
    EXPECT_EQ(0u, buffer[HOLD + 2]);
}

} // namespace
