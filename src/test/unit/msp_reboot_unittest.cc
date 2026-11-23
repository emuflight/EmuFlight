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

void testMspReboot(uint8_t mode);
uint8_t testMspRebootFirmwareMode(void);

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
