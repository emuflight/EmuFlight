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
