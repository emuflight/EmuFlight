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

#include <csetjmp>
#include <stdint.h>
#include <stdbool.h>

extern "C" {
#include "platform.h"

#include "drivers/system.h"
#include "fc/runtime_config.h"
#include "interface/msp_reboot.h"
#include "interface/msp_protocol.h"
#include "interface/msp.h"
#include "common/time.h"
#include "common/streambuf.h"
#include "drivers/serial.h"

void testMspReboot(uint8_t mode);
uint8_t testMspRebootFirmwareMode(void);

mspResult_e testMspSetRtc(uint8_t *payload, int length);
static uint32_t testMillis;
uint32_t millis(void) { return testMillis; }
bool isSerialTransmitBufferEmpty(const serialPort_t *) { return true; }
void serialWrite(serialPort_t *, uint8_t) {}

static std::jmp_buf resetJump;
static unsigned motorStopCount;
static unsigned resetCount;
void stopPwmAllMotors(void) { motorStopCount++; }
void systemReset(void) { resetCount++; std::longjmp(resetJump, 1); }
void systemResetToBootloader(bootloaderRequestType_e) { resetCount++; std::longjmp(resetJump, 1); }
}

#include "gtest/gtest.h"

static void runReboot(void)
{
    motorStopCount = resetCount = 0;
    if (setjmp(resetJump) == 0) {
        testMspReboot(testMspRebootFirmwareMode());
    }
}

TEST(MspRebootTest, AllowedWhileDisarmed)
{
    armingFlags = 0;
    EXPECT_TRUE(mspRebootIsAllowed());
}

TEST(MspRebootTest, RejectedWhileArmed)
{
    armingFlags = ARMED;
    EXPECT_FALSE(mspRebootIsAllowed());
}

TEST(MspRebootTest, ArmedCallbackKeepsMotorsAndDoesNotReset)
{
    armingFlags = ARMED;
    runReboot();
    EXPECT_EQ(0u, motorStopCount);
    EXPECT_EQ(0u, resetCount);
}

TEST(MspRebootTest, DisarmedCallbackStopsMotorsAndResets)
{
    armingFlags = 0;
    runReboot();
    EXPECT_EQ(1u, motorStopCount);
    EXPECT_EQ(1u, resetCount);
}

// MSP_SET_RTC payload: u16 year (little endian), month, day, hours, minutes, seconds.
static mspResult_e setRtc(uint16_t y, uint8_t mo, uint8_t d, uint8_t h, uint8_t mi, uint8_t s)
{
    uint8_t p[7] = { (uint8_t)(y & 0xff), (uint8_t)(y >> 8), mo, d, h, mi, s };
    return testMspSetRtc(p, sizeof(p));
}

// Stored RTC time in Unix milliseconds; -1 when no time is stored.
static int64_t storedMillis(void)
{
    rtcTime_t t = -1;
    if (!rtcGet(&t)) {
        return -1;
    }
    return t;
}

TEST(MspSetRtcTest, AcceptedDateAcksAndStores)
{
    testMillis = 0;
    // 2020-01-01T00:00:00Z = 1577836800 (calendar fact)
    EXPECT_EQ(MSP_RESULT_ACK, setRtc(2020, 1, 1, 0, 0, 0));
    EXPECT_EQ(1577836800000, storedMillis());
}

TEST(MspSetRtcTest, RejectedDatesReturnErrorAndKeepClock)
{
    testMillis = 0;
    ASSERT_EQ(MSP_RESULT_ACK, setRtc(2020, 1, 1, 0, 0, 0));
    const int64_t before = storedMillis();
    ASSERT_EQ(1577836800000, before);
    EXPECT_EQ(MSP_RESULT_ERROR, setRtc(1970, 1, 1, 0, 0, 0));   // before 2000
    EXPECT_EQ(before, storedMillis());
    EXPECT_EQ(MSP_RESULT_ERROR, setRtc(2100, 1, 1, 0, 0, 0));   // after 2099
    EXPECT_EQ(before, storedMillis());
    EXPECT_EQ(MSP_RESULT_ERROR, setRtc(2023, 2, 30, 0, 0, 0));  // impossible day
    EXPECT_EQ(before, storedMillis());
    EXPECT_EQ(MSP_RESULT_ERROR, setRtc(2023, 13, 1, 0, 0, 0));  // impossible month
    EXPECT_EQ(before, storedMillis());
}
