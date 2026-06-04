/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdint.h>
#include <stdlib.h>

#include <limits.h>

extern "C" {
    #include "build/build_config.h"

    #include "common/color.h"

    #include "drivers/light_ws2811strip.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
STATIC_UNIT_TESTED extern uint16_t dmaBufferOffset;

STATIC_UNIT_TESTED void fastUpdateLEDDMABuffer(ledStripFormatRGB_e ledFormat, rgbColor24bpp_t *color);
STATIC_UNIT_TESTED void updateLEDDMABuffer(uint8_t componentValue);
}

// Helper to verify byte bits match expected value
static void verifyByteBits(uint8_t byteIndex, uint8_t expectedValue) {
    for (int bit = 7; bit >= 0; --bit) {
        uint8_t expectedBit = (expectedValue >> bit) & 1;
        const auto expectedCompare = expectedBit ? BIT_COMPARE_1 : BIT_COMPARE_0;
        EXPECT_EQ(expectedCompare, ledStripDMABuffer[(byteIndex * 8) + (7 - bit)]);
    }
}

TEST(WS2812, updateDMABufferGRB) {
    // given
    rgbColor24bpp_t color1 = { .raw = {0xFF,0xAA,0x55} };  // R=0xFF, G=0xAA, B=0x55

    // and
    dmaBufferOffset = 0;

    // when
    fastUpdateLEDDMABuffer(LED_GRB, &color1);  // GRB format: G, R, B byte order

    // then: verify byte offset is 24 bits (3 bytes)
    EXPECT_EQ(24, dmaBufferOffset);

    // and: verify GRB byte ordering (G first = 0xFF all 1's)
    // Byte 0: G = 0xFF = 11111111
    verifyByteBits(0, 0xFF);

    // Byte 1: R = 0xAA = 10101010
    verifyByteBits(1, 0xAA);

    // Byte 2: B = 0x55 = 01010101
    verifyByteBits(2, 0x55);
}

TEST(WS2812, updateDMABufferRGB) {
    // given
    rgbColor24bpp_t color1 = { .raw = {0xFF,0xAA,0x55} };  // R=0xFF, G=0xAA, B=0x55

    // and
    dmaBufferOffset = 0;

    // when
    fastUpdateLEDDMABuffer(LED_RGB, &color1);  // RGB format: R, G, B byte order

    // then: verify byte offset is 24 bits (3 bytes)
    EXPECT_EQ(24, dmaBufferOffset);

    // and: verify RGB byte ordering (R first = 0xAA)
    // Byte 0: R = 0xAA = 10101010
    verifyByteBits(0, 0xAA);

    // Byte 1: G = 0xFF = 11111111 (all 1's)
    verifyByteBits(1, 0xFF);

    // Byte 2: B = 0x55 = 01010101
    verifyByteBits(2, 0x55);
}

extern "C" {
rgbColor24bpp_t* hsvToRgb24(const hsvColor_t *c) {
    UNUSED(c);
    return NULL;
}

void ws2811LedStripHardwareInit(ioTag_t ioTag) {
    UNUSED(ioTag);
}

void ws2811LedStripDMAEnable(void) {}
}
