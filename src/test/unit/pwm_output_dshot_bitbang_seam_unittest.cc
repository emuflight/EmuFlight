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
#include "drivers/io.h"
#include "drivers/dshot_bitbang.h"
#include "drivers/pwm_output.h"
#include "drivers/timer.h"

    void motorDevInit(const motorDevConfig_t *motorDevConfig, uint16_t idlePulse, uint8_t motorCount);

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// Real pwm_output.c seam; the bitbang driver and the classic per-motor config are fakes.
// Expected values come from the rules in the PR: bitbang replaces classic only for ON with
// DSHOT150/300/600 and no inversion, motor pins are set up without taking their timers,
// a failed claim stops all DShot output, and motors are set up again after a passthrough.
namespace {

constexpr int MOTORS = 4;
constexpr ioTag_t TAG_BASE = 0x10;

int classicConfigCalls;
int allocCalls;
int bbMotorInitCalls;
uint8_t bbMotorInitIndex[MOTORS];
uint8_t bbMotorInitReordered[MOTORS];
ioTag_t bbMotorInitTag[MOTORS];
bool bbPostInitResult;
int bbPostInitCalls;
int bbRestoreCalls;
int bbWriteCalls;
uint8_t bbWriteIndex[16];
uint16_t bbWriteValue[16];
int bbCompleteCalls;
int bbMotorInitFailAt;
uint8_t bbSupportedInversionArg;
timerHardware_t fakeTimerHardware[MOTORS + 1];
uint32_t fakeMicros;

void resetFakes() {
    classicConfigCalls = allocCalls = bbMotorInitCalls = bbPostInitCalls = bbRestoreCalls = bbWriteCalls = bbCompleteCalls = 0;
    bbPostInitResult = true;
    bbMotorInitFailAt = -1;
    bbSupportedInversionArg = 0xFF;
    memset(bbMotorInitIndex, 0xFF, sizeof(bbMotorInitIndex));
    memset(bbMotorInitReordered, 0xFF, sizeof(bbMotorInitReordered));
    memset(bbMotorInitTag, 0, sizeof(bbMotorInitTag));
    memset(fakeTimerHardware, 0, sizeof(fakeTimerHardware));
}

void initMotors(uint8_t mode, uint8_t protocol, uint8_t inversion = 0) {
    motorDevConfig_t config = {};
    config.motorPwmProtocol = protocol;
    config.useDshotBitbang = mode;
    config.motorPwmInversion = inversion;
    for (int i = 0; i < MOTORS; i++) {
        config.ioTags[i] = TAG_BASE + i;
        config.motorOutputReordering[i] = MOTORS - 1 - i;
    }
    motorDevInit(&config, 0, MOTORS);
    pwmEnableMotors();
}

} // namespace

TEST(PwmOutputBitbangSeam, OnWithDshot600SetsUpPinsWithoutTakingTimersOrClassicDma) {
    resetFakes();
    initMotors(DSHOT_BITBANG_ON, PWM_TYPE_DSHOT600);
    ASSERT_EQ(MOTORS, bbMotorInitCalls);
    EXPECT_EQ(0, allocCalls);          // pin timers stay free so they can pace a port group
    EXPECT_EQ(0, classicConfigCalls);  // no per-channel DMA claim
    for (int i = 0; i < MOTORS; i++) {
        const int slot = MOTORS - 1 - i; // reversed mapping: logical motor i uses pad slot MOTORS-1-i
        EXPECT_EQ(i, bbMotorInitIndex[i]);
        EXPECT_EQ(slot, bbMotorInitReordered[i]);
        EXPECT_EQ(TAG_BASE + slot, bbMotorInitTag[i]);
    }
    EXPECT_TRUE(pwmAreMotorsEnabled());
}

TEST(PwmOutputBitbangSeam, AllThreeSupportedProtocolsUseBitbang) {
    const uint8_t protocols[] = {PWM_TYPE_DSHOT150, PWM_TYPE_DSHOT300, PWM_TYPE_DSHOT600};
    for (uint8_t protocol : protocols) {
        resetFakes();
        initMotors(DSHOT_BITBANG_ON, protocol);
        EXPECT_EQ(MOTORS, bbMotorInitCalls) << (int)protocol;
        EXPECT_EQ(0, classicConfigCalls) << (int)protocol;
    }
}

TEST(PwmOutputBitbangSeam, OffAndAutoKeepClassicDshot) {
    const uint8_t modes[] = {DSHOT_BITBANG_OFF, DSHOT_BITBANG_AUTO};
    for (uint8_t mode : modes) {
        resetFakes();
        initMotors(mode, PWM_TYPE_DSHOT600);
        EXPECT_EQ(0, bbMotorInitCalls) << (int)mode;
        EXPECT_EQ(MOTORS, classicConfigCalls) << (int)mode;
        EXPECT_EQ(MOTORS, allocCalls) << (int)mode;
    }
}

TEST(PwmOutputBitbangSeam, FastProtocolWithOnKeepsClassicDshot) {
    const uint8_t protocols[] = {PWM_TYPE_DSHOT1200, PWM_TYPE_DSHOT2400, PWM_TYPE_DSHOT4800, PWM_TYPE_PROSHOT1000};
    for (uint8_t protocol : protocols) {
        resetFakes();
        initMotors(DSHOT_BITBANG_ON, protocol);
        EXPECT_EQ(0, bbMotorInitCalls) << (int)protocol;
        EXPECT_EQ(MOTORS, classicConfigCalls) << (int)protocol;
    }
}

TEST(PwmOutputBitbangSeam, InversionIsPassedToTheSupportCheckAndKeepsClassicDshot) {
    resetFakes();
    initMotors(DSHOT_BITBANG_ON, PWM_TYPE_DSHOT600, 1);
    EXPECT_EQ(1, bbSupportedInversionArg);
    EXPECT_EQ(0, bbMotorInitCalls);
    EXPECT_EQ(MOTORS, classicConfigCalls);

    resetFakes();
    initMotors(DSHOT_BITBANG_ON, PWM_TYPE_DSHOT600, 0);
    EXPECT_EQ(0, bbSupportedInversionArg);
    EXPECT_EQ(MOTORS, bbMotorInitCalls);
}

TEST(PwmOutputBitbangSeam, PostInitSuccessRoutesWritesToBitbang) {
    resetFakes();
    initMotors(DSHOT_BITBANG_ON, PWM_TYPE_DSHOT600);
    motorDevPostInit();
    EXPECT_EQ(1, bbPostInitCalls);
    EXPECT_TRUE(pwmAreMotorsEnabled());

    pwmWriteMotor(2, 1234.6f);        // the float is rounded to the nearest DShot value
    pwmCompleteMotorUpdate(MOTORS);
    ASSERT_EQ(1, bbWriteCalls);
    EXPECT_EQ(2, bbWriteIndex[0]);
    EXPECT_EQ(1235, bbWriteValue[0]);
    EXPECT_EQ(1, bbCompleteCalls);
}

TEST(PwmOutputBitbangSeam, PostInitFailureStopsAllOutput) {
    resetFakes();
    bbPostInitResult = false;
    initMotors(DSHOT_BITBANG_ON, PWM_TYPE_DSHOT600);
    motorDevPostInit();
    EXPECT_EQ(1, bbPostInitCalls);
    EXPECT_FALSE(pwmAreMotorsEnabled());

    pwmWriteMotor(0, 1100.0f);
    pwmCompleteMotorUpdate(MOTORS);
    EXPECT_EQ(0, bbWriteCalls);
    EXPECT_EQ(0, bbCompleteCalls);
}

TEST(PwmOutputBitbangSeam, PostInitIsSkippedWhenBitbangIsNotActive) {
    resetFakes();
    initMotors(DSHOT_BITBANG_OFF, PWM_TYPE_DSHOT600);
    motorDevPostInit();
    EXPECT_EQ(0, bbPostInitCalls);
    EXPECT_TRUE(pwmAreMotorsEnabled());
}

TEST(PwmOutputBitbangSeam, PinSetupFailureStopsAllOutputAndEndsInit) {
    resetFakes();
    bbMotorInitFailAt = 1;
    initMotors(DSHOT_BITBANG_ON, PWM_TYPE_DSHOT600);
    EXPECT_EQ(2, bbMotorInitCalls);    // motor 0 ok, motor 1 refused, motors 2 and 3 never tried
    EXPECT_EQ(0, classicConfigCalls);
    EXPECT_FALSE(pwmAreMotorsEnabled());

    pwmWriteMotor(0, 1100.0f);
    EXPECT_EQ(0, bbWriteCalls);
}

TEST(PwmOutputBitbangSeam, MotorsAreSetUpAgainOnlyAfterTheyWereDisabled) {
    resetFakes();
    initMotors(DSHOT_BITBANG_ON, PWM_TYPE_DSHOT600);
    motorDevPostInit();
    EXPECT_EQ(0, bbRestoreCalls);      // boot: motors are already enabled

    pwmEnableMotors();                 // still enabled: nothing to restore
    EXPECT_EQ(0, bbRestoreCalls);

    pwmDisableMotors();                // passthrough start
    EXPECT_FALSE(pwmAreMotorsEnabled());
    pwmEnableMotors();                 // passthrough end
    EXPECT_EQ(1, bbRestoreCalls);
    EXPECT_TRUE(pwmAreMotorsEnabled());

    pwmEnableMotors();
    EXPECT_EQ(1, bbRestoreCalls);
}

TEST(PwmOutputBitbangSeam, ClassicDshotNeverCallsRestore) {
    resetFakes();
    initMotors(DSHOT_BITBANG_OFF, PWM_TYPE_DSHOT600);
    pwmDisableMotors();
    pwmEnableMotors();
    EXPECT_EQ(0, bbRestoreCalls);
}

// A blocking command writes the selected motor with the command and every other motor with
// nothing, then completes the update once per repeat. BEACON1 has one repeat.
TEST(PwmOutputBitbangSeam, BlockingCommandGoesThroughBitbangForSelectedMotorOnly) {
    resetFakes();
    initMotors(DSHOT_BITBANG_ON, PWM_TYPE_DSHOT600);
    motorDevPostInit();
    pwmWriteDshotCommand(1, MOTORS, DSHOT_CMD_BEACON1, true);
    ASSERT_EQ(1, bbWriteCalls);
    EXPECT_EQ(1, bbWriteIndex[0]);
    EXPECT_EQ(DSHOT_CMD_BEACON1, bbWriteValue[0]);
    EXPECT_EQ(1, bbCompleteCalls);
}

TEST(PwmOutputBitbangSeam, BlockingCommandToAllMotorsWritesEveryMotor) {
    resetFakes();
    initMotors(DSHOT_BITBANG_ON, PWM_TYPE_DSHOT600);
    motorDevPostInit();
    pwmWriteDshotCommand(ALL_MOTORS, MOTORS, DSHOT_CMD_BEACON1, true);
    ASSERT_EQ(MOTORS, bbWriteCalls);
    for (int i = 0; i < MOTORS; i++) {
        EXPECT_EQ(i, bbWriteIndex[i]);
        EXPECT_EQ(DSHOT_CMD_BEACON1, bbWriteValue[i]);
    }
    EXPECT_EQ(1, bbCompleteCalls);
}

extern "C" {

// bitbang driver fakes
bool dshotBitbangSupported(uint8_t motorPwmInversion) {
    bbSupportedInversionArg = motorPwmInversion;
    return motorPwmInversion == 0;
}

bool dshotBitbangMotorInit(uint8_t motorIndex, uint8_t reorderedMotorIndex, ioTag_t tag) {
    if (bbMotorInitCalls < MOTORS) {
        bbMotorInitIndex[bbMotorInitCalls] = motorIndex;
        bbMotorInitReordered[bbMotorInitCalls] = reorderedMotorIndex;
        bbMotorInitTag[bbMotorInitCalls] = tag;
    }
    const bool ok = bbMotorInitCalls != bbMotorInitFailAt;
    bbMotorInitCalls++;
    return ok;
}

bool dshotBitbangPostInit(const motorDevConfig_t *, uint8_t) {
    bbPostInitCalls++;
    return bbPostInitResult;
}

void dshotBitbangWriteInt(uint8_t motorIndex, uint16_t value) {
    if (bbWriteCalls < 16) {
        bbWriteIndex[bbWriteCalls] = motorIndex;
        bbWriteValue[bbWriteCalls] = value;
    }
    bbWriteCalls++;
}

void dshotBitbangUpdateComplete(uint8_t) { bbCompleteCalls++; }
void dshotBitbangRestore(void) { bbRestoreCalls++; }

// classic path fakes
const timerHardware_t *timerAllocate(ioTag_t, resourceOwner_e, uint8_t resourceIndex) {
    allocCalls++;
    return &fakeTimerHardware[resourceIndex];
}

bool pwmDshotMotorHardwareConfig(const timerHardware_t *, uint8_t, uint8_t, motorPwmProtocolTypes_e, uint8_t) {
    classicConfigCalls++;
    return true;
}

IO_t IOGetByTag(ioTag_t) { return nullptr; }
void IOInit(IO_t, resourceOwner_e, uint8_t) {}
void IOConfigGPIOAF(IO_t, ioConfig_t, uint8_t) {}
void pwmWriteDshotInt(uint8_t, uint16_t) {}
void pwmCompleteDshotMotorUpdate(uint8_t) {}
uint32_t timerClock(TIM_TypeDef *) { return 0; }
uint32_t micros(void) { return fakeMicros; }
void delayMicroseconds(uint32_t) {}
void TIM_OCStructInit(TIM_OCInitTypeDef *) {}
void configTimeBase(TIM_TypeDef *, uint16_t, uint32_t) {}
volatile timCCR_t *timerChCCR(const timerHardware_t *) { static timCCR_t ccr; return &ccr; }
void timerOCInit(TIM_TypeDef *, uint8_t, TIM_OCInitTypeDef *) {}
void timerOCPreloadConfig(TIM_TypeDef *, uint8_t, uint16_t) {}
void timerForceOverflow(TIM_TypeDef *) {}
motorDmaOutput_t *getMotorDmaOutput(uint8_t) { static motorDmaOutput_t motor; return &motor; }
void IOConfigGPIO(IO_t, ioConfig_t) {}
void TIM_Cmd(TIM_TypeDef *, FunctionalState) {}
void TIM_CtrlPWMOutputs(TIM_TypeDef *, FunctionalState) {}

}
