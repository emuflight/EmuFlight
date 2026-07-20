/*
 * This file is part of EmuFlight.
 *
 * EmuFlight is free software. You can redistribute this software and/or
 * modify this software under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * EmuFlight is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this software. If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Tests that the DShot motor beacon fires via beeper(BEEPER_RX_SET) even when
 * a higher-priority battery alarm (BEEPER_BAT_CRIT_LOW) currently owns the
 * physical buzzer and would otherwise block the beacon via priority arbitration.
 *
 * Regression guard for: dshotBeaconPendingMode must be set before the
 * selectedCandidate==NULL early-return in beeper(), not after.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

extern "C" {
    #include "platform.h"
    #include "io/beeper.h"
    #include "pg/beeper.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "sensors/battery.h"
    #include "drivers/pwm_output.h"
    #include "fc/rc_modes.h"
    #include "io/gps.h"

    // beeperConfig PG is defined in io/beeper.c itself (linked via _SRC)

    // --- controllable stub state ---
    static bool stubMotorsRunning = false;
    static timeUs_t stubLastDisarmTime = 0;
    static bool stubTryingToArm = false;
    static batteryState_e stubBatteryState = BATTERY_OK;
    static bool stubRcModeBeeper = false;
    static int dshotBeaconCallCount = 0;
    static uint8_t dshotBeaconToneRecorded = 0;

    // --- stubs ---
    bool areMotorsRunning(void) { return stubMotorsRunning; }
    timeUs_t getLastDisarmTimeUs(void) { return stubLastDisarmTime; }
    bool isTryingToArm(void) { return stubTryingToArm; }
    batteryState_e getBatteryState(void) { return stubBatteryState; }

    bool IS_RC_MODE_ACTIVE(boxId_e modeId) {
        if (modeId == BOXBEEPERON) return stubRcModeBeeper;
        return false;
    }

    void pwmWriteDshotCommand(uint8_t index, uint8_t motorCount, uint8_t command, bool blocking) {
        UNUSED(index); UNUSED(motorCount); UNUSED(blocking);
        dshotBeaconCallCount++;
        dshotBeaconToneRecorded = command;
    }

    // minimal stubs for beeper.c dependencies
    void schedulerIgnoreTaskExecTime(void) {}
    void BEEP_ON_stub(void) {}
    void BEEP_OFF_stub(void) {}
    void warningLedEnable(void) {}
    void warningLedDisable(void) {}
    void warningLedRefresh(void) {}
    void warningLedFlash(void) {}
    void warningLedUpdate(void) {}

    uint32_t micros(void) { return 0; }
    uint32_t millis(void) { return 0; }
}

#include "gtest/gtest.h"

// Helper: run beeperUpdate() several times to advance through a full beep sequence.
// Multiple iterations guard against future changes to beeperSilence() that might
// leave beeperIsOn in an unexpected state on the first tick.
static void runBeeperUpdate(timeUs_t timeUs, int iterations = 3) {
    for (int i = 0; i < iterations; ++i) beeperUpdate(timeUs);
}

class DshotBeaconPriorityTest : public ::testing::Test {
protected:
    void SetUp() override {
        pgResetAll();
        // Enable both beacon modes (default has them masked off)
        beeperConfigMutable()->dshotBeaconOffFlags = 0;
        beeperSilence();
        stubMotorsRunning = false;
        stubLastDisarmTime = 0;
        stubTryingToArm = false;
        stubBatteryState = BATTERY_OK;
        stubRcModeBeeper = false;
        dshotBeaconCallCount = 0;
        dshotBeaconToneRecorded = 0;
    }
};

// Normal case: beeper switch on, no competing alarm → beacon fires.
TEST_F(DshotBeaconPriorityTest, BeaconFiresWhenRxSetActiveNoBatteryAlarm)
{
    // given: beeper switch active, well past disarm guard, motors idle
    stubRcModeBeeper = true;
    const timeUs_t t = 1200000U + 1000;

    // when
    runBeeperUpdate(t);

    // then: DShot beacon was sent
    EXPECT_GE(dshotBeaconCallCount, 1);
}

// Key regression: battery critical alarm active (priority 6 > RX_SET priority 9).
// Before the fix, beeper(BEEPER_RX_SET) returned early and never set
// dshotBeaconPendingMode, so the beacon was silenced.
TEST_F(DshotBeaconPriorityTest, BeaconFiresWhenBatteryCritLowOwnsBeeper)
{
    // given: battery critical alarm is active (it calls beeper(BEEPER_BAT_CRIT_LOW) externally)
    beeper(BEEPER_BAT_CRIT_LOW);

    // and: beeper switch is also active → beeperUpdate() calls beeper(BEEPER_RX_SET)
    stubRcModeBeeper = true;
    const timeUs_t t = 1200000U + 1000;

    // when
    runBeeperUpdate(t);

    // then: DShot motor beacon MUST still fire despite battery alarm owning the buzzer
    EXPECT_GE(dshotBeaconCallCount, 1)
        << "DShot beacon was silenced by battery alarm priority — regression in dshotBeaconPendingMode placement";
}

// Mirror: battery low (priority 7) should also not block the beacon.
TEST_F(DshotBeaconPriorityTest, BeaconFiresWhenBatteryLowOwnsBeeper)
{
    beeper(BEEPER_BAT_LOW);
    stubRcModeBeeper = true;
    const timeUs_t t = 1200000U + 1000;

    runBeeperUpdate(t);

    EXPECT_GE(dshotBeaconCallCount, 1)
        << "DShot beacon was silenced by BAT_LOW alarm priority";
}

// Beacon must NOT fire when motors are running (safety guard).
TEST_F(DshotBeaconPriorityTest, BeaconDoesNotFireWhenMotorsRunning)
{
    stubRcModeBeeper = true;
    stubMotorsRunning = true;
    const timeUs_t t = 1200000U + 1000;

    runBeeperUpdate(t);

    EXPECT_EQ(0, dshotBeaconCallCount);
}

// Beacon must NOT fire within the disarm guard window.
TEST_F(DshotBeaconPriorityTest, BeaconDoesNotFireInsideDisarmGuard)
{
    stubRcModeBeeper = true;
    stubLastDisarmTime = 0;
    const timeUs_t t = 1200000U / 2; // inside guard window

    runBeeperUpdate(t);

    EXPECT_EQ(0, dshotBeaconCallCount);
}

// Beacon must NOT fire when dshotBeaconOffFlags masks BEEPER_RX_SET.
TEST_F(DshotBeaconPriorityTest, BeaconDoesNotFireWhenMaskedByOffFlags)
{
    beeperConfigMutable()->dshotBeaconOffFlags = BEEPER_GET_FLAG(BEEPER_RX_SET);
    stubRcModeBeeper = true;
    const timeUs_t t = 1200000U + 1000;

    runBeeperUpdate(t);

    EXPECT_EQ(0, dshotBeaconCallCount);
}

// RX_LOST (priority 1) was already immune pre-fix; confirm it still fires.
TEST_F(DshotBeaconPriorityTest, BeaconFiresOnRxLost)
{
    beeper(BEEPER_BAT_CRIT_LOW); // lower-priority alarm present
    beeper(BEEPER_RX_LOST);      // RX_LOST priority 1 wins buzzer too
    const timeUs_t t = 1200000U + 1000;

    runBeeperUpdate(t);

    EXPECT_GE(dshotBeaconCallCount, 1);
}

// beeperSilence() must clear dshotBeaconPendingMode so a silenced beeper
// cannot fire a motor beacon on the next tick.
TEST_F(DshotBeaconPriorityTest, BeeperSilenceClearsPendingMode)
{
    // given: pending mode is primed directly (RC mode is OFF so beeperUpdate()
    // cannot re-arm it after the silence)
    stubRcModeBeeper = false;
    beeper(BEEPER_RX_SET);
    const timeUs_t t = 1200000U + 1000;

    // when: beeperSilence() is called before beeperUpdate() consumes the pending mode
    beeperSilence();

    // then: beeperUpdate() must NOT fire the motor beacon
    runBeeperUpdate(t);
    EXPECT_EQ(0, dshotBeaconCallCount)
        << "beeperSilence() failed to clear dshotBeaconPendingMode";
}

// STUBS — satisfy link dependencies not covered above

extern "C" {
    uint8_t armingFlags = 0;
    int16_t debug[4] = {};
    uint32_t stateFlags = 0;
    gpsSolutionData_t gpsSol = {};
    void delay(uint32_t ms) { UNUSED(ms); }
    bool isMotorProtocolDshot(void) { return true; }
    uint8_t getMotorCount(void) { return 4; }
    void systemBeep(bool onoff) { UNUSED(onoff); }
    bool feature(uint32_t mask) { UNUSED(mask); return false; }
}
