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

#include <stdint.h>
#include <string.h>

extern "C" {

#include "platform.h"
#include "drivers/io.h"
#include "drivers/pwm_output.h"
#include "drivers/timer.h"

    void motorDevInit(const motorDevConfig_t *motorDevConfig, uint16_t idlePulse, uint8_t motorCount);

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

namespace {

constexpr int MOTORS = 4;
int configCalls;
int failMotor; // index whose hardware config returns false; -1 = none
timerHardware_t fakeTimerHardware[MOTORS];

void resetFakes(int failAt) {
    configCalls = 0;
    failMotor = failAt;
    memset(fakeTimerHardware, 0, sizeof(fakeTimerHardware));
}

void initMotors(uint8_t count) {
    motorDevConfig_t config = {};
    config.motorPwmProtocol = PWM_TYPE_DSHOT600;
    for (int i = 0; i < MOTORS; i++) {
        config.ioTags[i] = IO_TAG(PA0) + i;
    }
    motorDevInit(&config, 0, count);
    pwmEnableMotors();
}

} // namespace

// The DShot command queue is static state in pwm_output.c, so the rejection test runs first
// (it needs an empty queue) and the acceptance test after it.
TEST(PwmOutputDshotInit, FailedInitRejectsDshotCommands) {
    resetFakes(1);
    initMotors(MOTORS);
    pwmWriteDshotCommand(ALL_MOTORS, MOTORS, DSHOT_CMD_BEACON1, false);
    EXPECT_FALSE(pwmDshotCommandIsQueued());
}

TEST(PwmOutputDshotInit, SuccessfulInitAcceptsDshotCommands) {
    resetFakes(-1);
    initMotors(MOTORS);
    pwmWriteDshotCommand(ALL_MOTORS, MOTORS, DSHOT_CMD_BEACON1, false);
    EXPECT_TRUE(pwmDshotCommandIsQueued());
}

// Expected values come from the IT #1507 specification: output is enabled only when every motor
// configured, and one failed motor stops all DShot output.
TEST(PwmOutputDshotInit, AllMotorsConfiguredEnablesOutput) {
    resetFakes(-1);
    initMotors(MOTORS);
    EXPECT_EQ(MOTORS, configCalls);
    EXPECT_TRUE(pwmAreMotorsEnabled());
}

TEST(PwmOutputDshotInit, FirstMotorFailureStopsOutputAndEndsInit) {
    resetFakes(0);
    initMotors(MOTORS);
    EXPECT_EQ(1, configCalls);
    EXPECT_FALSE(pwmAreMotorsEnabled());
}

TEST(PwmOutputDshotInit, MiddleMotorFailureStopsOutput) {
    resetFakes(2);
    initMotors(MOTORS);
    EXPECT_EQ(3, configCalls);
    EXPECT_FALSE(pwmAreMotorsEnabled());
}

TEST(PwmOutputDshotInit, LastMotorFailureStopsOutput) {
    resetFakes(MOTORS - 1);
    initMotors(MOTORS);
    EXPECT_EQ(MOTORS, configCalls);
    EXPECT_FALSE(pwmAreMotorsEnabled());
}

TEST(PwmOutputDshotInit, FailureBeyondMotorCountIsNotReached) {
    resetFakes(3);
    initMotors(3);
    EXPECT_EQ(3, configCalls);
    EXPECT_TRUE(pwmAreMotorsEnabled());
}

TEST(PwmOutputDshotInit, ZeroMotorsConfiguresNothing) {
    resetFakes(0);
    initMotors(0);
    EXPECT_EQ(0, configCalls);
}

extern "C" {

const timerHardware_t *timerAllocate(ioTag_t, resourceOwner_e, uint8_t resourceIndex) {
    return &fakeTimerHardware[resourceIndex];
}

bool pwmDshotMotorHardwareConfig(const timerHardware_t *, uint8_t motorIndex, motorPwmProtocolTypes_e, uint8_t) {
    configCalls++;
    return failMotor != motorIndex;
}

IO_t IOGetByTag(ioTag_t) { return nullptr; }
void IOInit(IO_t, resourceOwner_e, uint8_t) {}
void IOConfigGPIOAF(IO_t, ioConfig_t, uint8_t) {}
void pwmWriteDshotInt(uint8_t, uint16_t) {}
void pwmCompleteDshotMotorUpdate(uint8_t) {}
uint32_t timerClock(TIM_TypeDef *) { return 0; }
uint32_t micros(void) { return 0; }
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
