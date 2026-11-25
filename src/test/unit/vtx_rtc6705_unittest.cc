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
 * but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <string.h>

#include <vector>

extern "C" {
    #include "platform.h"

    #include "drivers/bus_spi.h"
    #include "drivers/io.h"
    #include "drivers/time.h"
    #include "drivers/vtx_rtc6705.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// External power pins carry a 2-bit code: pin 1 = bit 0, pin 2 = bit 1.
// 00 = lowest power, 01 = mid, 10 = max, 11 = undefined and unsafe.
// Register 0x07 values from the RTC6705 datasheet default 0x4FBD: the low setting sets PD_Q5G
// (0x0040) and clears PA5G_PW (0x0180) and PA5G_BS (0x0E00), which gives 0x407D.
namespace {

constexpr uint8_t TAG_P1 = 1;
constexpr uint8_t TAG_P2 = 2;
constexpr uint32_t REG_LOW = 0x407D;
constexpr uint32_t REG_DEFAULT = 0x4FBD;

bool pinState[8];
struct PinWrite {
    bool pin1After;
    bool pin2After;
};
std::vector<PinWrite> writes;
uint8_t spiBytes[4];
int spiByteCount;
int regWrites;
uint32_t lastReg;

void recordWrite(IO_t io, bool value)
{
    pinState[(uintptr_t)io] = value;
    writes.push_back({ pinState[TAG_P1], pinState[TAG_P2] });
}

uint32_t reverse32(uint32_t in)
{
    uint32_t out = 0;
    for (int i = 0; i < 32; i++) {
        out |= ((in >> i) & 1) << (31 - i);
    }
    return out;
}

} // namespace

extern "C" {

IO_t IOGetByTag(ioTag_t tag) { return tag ? (IO_t)(uintptr_t)tag : NULL; }
void IOInit(IO_t, resourceOwner_e, uint8_t) {}
void IOConfigGPIO(IO_t, ioConfig_t) {}
void IOWrite(IO_t io, bool value) { recordWrite(io, value); }
void IOHi(IO_t io) { recordWrite(io, true); }
void IOLo(IO_t io) { recordWrite(io, false); }

void spiSetDivisor(SPI_TypeDef *, uint16_t) {}
uint8_t spiTransferByte(SPI_TypeDef *, uint8_t data)
{
    spiBytes[spiByteCount++ & 3] = data;
    if ((spiByteCount & 3) == 0) {
        const uint32_t command = ((uint32_t)spiBytes[0] << 24) | ((uint32_t)spiBytes[1] << 16) | ((uint32_t)spiBytes[2] << 8) | spiBytes[3];
        lastReg = (reverse32(command) >> 5) & 0xFFFF;
        regWrites++;
    }
    return 0;
}
void delayMicroseconds(timeUs_t) {}

}

class Rtc6705DynamicPowerTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        memset(pinState, 0, sizeof(pinState));
        pinState[TAG_P1] = pinState[TAG_P2] = true; // init must clear both pins
        writes.clear();
        spiByteCount = 0;
        regWrites = 0;
        lastReg = 0;
        rtc6705IOInit();
    }

    // Set a power index and check the resulting pin code and register.
    void expectLevel(uint8_t index, bool pin1, bool pin2, uint32_t reg)
    {
        rtc6705SetRFPower(index);
        EXPECT_EQ(pin1, pinState[TAG_P1]) << "index " << (int)index;
        EXPECT_EQ(pin2, pinState[TAG_P2]) << "index " << (int)index;
        EXPECT_EQ(reg, lastReg) << "index " << (int)index;
    }
};

TEST_F(Rtc6705DynamicPowerTest, PinsStartLowAtInit)
{
    EXPECT_FALSE(pinState[TAG_P1]);
    EXPECT_FALSE(pinState[TAG_P2]);
}

TEST_F(Rtc6705DynamicPowerTest, ThreeLevelsFromZero)
{
    EXPECT_EQ(3, VTX_RTC6705_POWER_COUNT);
    EXPECT_EQ(0, VTX_RTC6705_MIN_POWER);
}

TEST_F(Rtc6705DynamicPowerTest, EachIndexSetsItsCodeAndRegister)
{
    expectLevel(0, false, false, REG_LOW);
    expectLevel(1, true, false, REG_LOW);
    expectLevel(2, false, true, REG_DEFAULT);
}

TEST_F(Rtc6705DynamicPowerTest, IndexOutsideRangeNeverGivesCode11)
{
    for (uint8_t index : { 3, 4, 100, 255 }) {
        expectLevel(index, false, true, REG_DEFAULT);
    }
}

TEST_F(Rtc6705DynamicPowerTest, LevelChangesNeverPassThroughCode11)
{
    const uint8_t indexes[] = { 0, 1, 2, 3, 255 };
    for (uint8_t from : indexes) {
        for (uint8_t to : indexes) {
            writes.clear();
            rtc6705SetRFPower(from);
            rtc6705SetRFPower(to);
            for (const PinWrite &w : writes) {
                EXPECT_FALSE(w.pin1After && w.pin2After)
                    << "code 11 after a write, from " << (int)from << " to " << (int)to;
            }
        }
    }
}

TEST_F(Rtc6705DynamicPowerTest, EveryCallWritesTheRegister)
{
    rtc6705SetRFPower(1);
    rtc6705SetRFPower(2);
    rtc6705SetRFPower(2);
    EXPECT_EQ(3, regWrites);
}
