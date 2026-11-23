/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>

extern "C" {
#include "platform.h"

#include "build/debug.h"
#include "common/filter.h"
#include "common/time.h"
#include "config/feature.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "io/beeper.h"
#include "sensors/battery.h"

uint8_t debugMode;
int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t armingFlags;
float rcCommand[4];
controlRateConfig_t *currentControlRateProfile;

static uint16_t rawDeciVolt;
static biquadFilter_t vbatFilter;
static uint32_t simulatedMillis;
static unsigned profileCallCount;
static uint8_t lastProfileCellCount;

uint32_t millis(void) { return simulatedMillis; }
bool feature(uint32_t) { return false; }
void beeper(beeperMode_e) {}
throttleStatus_e calculateThrottleStatus(void) { return THROTTLE_LOW; }
void changePidProfileFromCellCount(uint8_t cellCount) { profileCallCount++; lastProfileCellCount = cellCount; }

void voltageMeterReset(voltageMeter_t *meter) { meter->filtered = 0; meter->unfiltered = 0; }
void voltageMeterADCInit(void) { biquadFilterInitLPF(&vbatFilter, GET_BATTERY_LPF_FREQUENCY(35), 20000); }
void voltageMeterADCRefresh(void) {}
void voltageMeterADCRead(voltageSensorADC_e, voltageMeter_t *meter)
{
    meter->unfiltered = rawDeciVolt;
    meter->filtered = (uint16_t)biquadFilterApply(&vbatFilter, rawDeciVolt);
}
void currentMeterReset(currentMeter_t *) {}
void currentMeterADCInit(void) {}
void currentMeterADCRefresh(int32_t) {}
void currentMeterADCRead(currentMeter_t *) {}
}

#include "gtest/gtest.h"

enum { VOLTAGE_TICK_US = 20000, ALERT_TICK_EVERY = 10 }; // 50 Hz voltage, 5 Hz presence/states

static timeUs_t nowUs;
static unsigned tickCount;

static void tick(uint16_t deciVolt)
{
    rawDeciVolt = deciVolt;
    nowUs += VOLTAGE_TICK_US;
    simulatedMillis = nowUs / 1000;
    batteryUpdateVoltage(nowUs);
    if (++tickCount % ALERT_TICK_EVERY == 0) {
        if (!(armingFlags & ARMED)) {
            batteryUpdatePresence();
        }
        batteryUpdateStates(nowUs);
    }
}

static void run(unsigned ms, uint16_t deciVolt)
{
    for (unsigned i = 0; i < ms / (VOLTAGE_TICK_US / 1000); i++) {
        tick(deciVolt);
    }
}

// linear ramp of the raw reading, e.g. a filter capacitor discharging
static void ramp(unsigned ms, uint16_t fromDeciVolt, uint16_t toDeciVolt)
{
    const unsigned steps = ms / (VOLTAGE_TICK_US / 1000);
    for (unsigned i = 1; i <= steps; i++) {
        tick((uint16_t)((int)fromDeciVolt + ((int)toDeciVolt - (int)fromDeciVolt) * (int)i / (int)steps));
    }
}

class BatteryPresenceTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        batteryConfigMutable()->vbatmaxcellvoltage = 43;
        batteryConfigMutable()->vbatmincellvoltage = 33;
        batteryConfigMutable()->vbatwarningcellvoltage = 35;
        batteryConfigMutable()->vbatnotpresentcellvoltage = 30;
        batteryConfigMutable()->voltageMeterSource = VOLTAGE_METER_ADC;
        batteryConfigMutable()->currentMeterSource = CURRENT_METER_NONE;
        batteryConfigMutable()->forceBatteryCellCount = 0;
        batteryConfigMutable()->vbatLpfPeriod = 35;
        batteryConfigMutable()->vbathysteresis = 1;
        armingFlags = 0;
        nowUs = 0;
        simulatedMillis = 0;
        tickCount = 0;
        rawDeciVolt = 0;
        profileCallCount = 0;
        lastProfileCellCount = 0;
        batteryInit();
    }
};

TEST_F(BatteryPresenceTest, DetectsSixCellsOnConnect)
{
    run(2000, 0);
    run(5000, 252);
    EXPECT_EQ(6, getBatteryCellCount());
    EXPECT_EQ(6, lastProfileCellCount);
    EXPECT_EQ(BATTERY_OK, getBatteryState());
}

// 22.8V is 3.8V per cell: counted from the lagging filter, a 2V stable window read 5 cells.
TEST_F(BatteryPresenceTest, PartlyUsedSixCellPackIsNotCountedAsFive)
{
    run(2000, 0);
    run(5000, 228);
    EXPECT_EQ(6, getBatteryCellCount());
    EXPECT_EQ(6, lastProfileCellCount);
}

TEST_F(BatteryPresenceTest, FourCellPackDetected)
{
    run(2000, 0);
    run(5000, 168);
    EXPECT_EQ(4, getBatteryCellCount());
    EXPECT_EQ(4, lastProfileCellCount);
}

TEST_F(BatteryPresenceTest, SixToFourSwapWithResidualVoltage)
{
    run(2000, 0);
    run(5000, 252);
    ASSERT_EQ(6, getBatteryCellCount());
    ramp(400, 252, 180);     // filter capacitor holds the old pack voltage
    run(5000, 168);          // 4S connected, never below the not-present window
    EXPECT_EQ(4, getBatteryCellCount());
    EXPECT_EQ(4, lastProfileCellCount);
}

TEST_F(BatteryPresenceTest, FourToSixSwapWithResidualVoltage)
{
    run(2000, 0);
    run(5000, 168);
    ASSERT_EQ(4, getBatteryCellCount());
    ramp(400, 168, 120);
    run(5000, 252);
    EXPECT_EQ(6, getBatteryCellCount());
    EXPECT_EQ(6, lastProfileCellCount);
}

TEST_F(BatteryPresenceTest, ShortUnplugInterval)
{
    run(2000, 0);
    run(5000, 252);
    ramp(600, 252, 100);
    run(1000, 100);          // 10V residual, above the USB window
    run(5000, 168);
    EXPECT_EQ(4, getBatteryCellCount());
    EXPECT_EQ(4, lastProfileCellCount);
}

TEST_F(BatteryPresenceTest, SlowCapacitorDischargeThenReconnect)
{
    run(2000, 0);
    run(5000, 252);
    const unsigned callsBefore = profileCallCount;
    ramp(20000, 252, 80);    // 25.2V to 8.0V in 20 s, never reaches 6V
    EXPECT_EQ(callsBefore, profileCallCount);  // a stable drift is not a swap
    EXPECT_EQ(6, getBatteryCellCount());
    run(5000, 168);
    EXPECT_EQ(4, getBatteryCellCount());
    EXPECT_EQ(4, lastProfileCellCount);
}

TEST_F(BatteryPresenceTest, FullDisconnectStillClearsCellCount)
{
    run(2000, 0);
    run(5000, 252);
    ramp(3000, 252, 0);
    run(3000, 0);
    EXPECT_EQ(0, getBatteryCellCount());
    EXPECT_EQ(BATTERY_NOT_PRESENT, getBatteryState());
    run(5000, 168);
    EXPECT_EQ(4, getBatteryCellCount());
}

TEST_F(BatteryPresenceTest, SameCountSwapKeepsCountAndProfile)
{
    run(2000, 0);
    run(5000, 168);
    const unsigned callsBefore = profileCallCount;
    ramp(400, 168, 140);
    run(5000, 168);
    EXPECT_EQ(4, getBatteryCellCount());
    EXPECT_EQ(4, lastProfileCellCount);
    EXPECT_GE(profileCallCount, callsBefore);
}

TEST_F(BatteryPresenceTest, NoiseAndSmallSagKeepCount)
{
    run(2000, 0);
    run(5000, 252);
    const unsigned callsBefore = profileCallCount;
    for (int i = 0; i < 200; i++) {
        tick(252 + (i % 3) - 1);
    }
    ramp(400, 252, 232);     // 2V motor-test sag while disarmed
    run(3000, 232);
    EXPECT_EQ(6, getBatteryCellCount());
    EXPECT_EQ(callsBefore, profileCallCount);
}

TEST_F(BatteryPresenceTest, SagAndRecoveryAfterFlightKeepCount)
{
    run(2000, 0);
    run(5000, 252);
    const unsigned callsBefore = profileCallCount;
    armingFlags = ARMED;
    ramp(1000, 252, 170);    // heavy load
    run(10000, 170);
    armingFlags = 0;
    run(300, 170);
    run(10000, 209);         // rest voltage of a depleted pack reads 5 cells by the formula
    EXPECT_EQ(6, getBatteryCellCount());
    EXPECT_EQ(callsBefore, profileCallCount);
}

TEST_F(BatteryPresenceTest, NoProfileChangeWhileArmed)
{
    run(2000, 0);
    run(5000, 252);
    const unsigned callsBefore = profileCallCount;
    armingFlags = ARMED;
    run(1000, 252);
    ramp(400, 252, 168);
    run(10000, 168);
    EXPECT_EQ(6, getBatteryCellCount());
    EXPECT_EQ(callsBefore, profileCallCount);
}

TEST_F(BatteryPresenceTest, ForcedCellCountWins)
{
    batteryConfigMutable()->forceBatteryCellCount = 3;
    run(2000, 0);
    run(5000, 252);
    EXPECT_EQ(3, getBatteryCellCount());
    ramp(400, 252, 168);
    run(5000, 168);
    EXPECT_EQ(3, getBatteryCellCount());
    EXPECT_EQ(0u, profileCallCount);
}

TEST_F(BatteryPresenceTest, UsbPowerOnlyIsNotABattery)
{
    run(5000, 50);           // USB voltage window
    EXPECT_EQ(0, getBatteryCellCount());
    EXPECT_EQ(0u, profileCallCount);
}

// A slow stable drift has no step, so it never recounts mid-way to a wrong count.
TEST_F(BatteryPresenceTest, SlowStableRampDoesNotRecountRepeatedly)
{
    run(2000, 0);
    run(5000, 252);
    const unsigned callsBefore = profileCallCount;
    ramp(30000, 252, 168);
    run(5000, 168);
    EXPECT_EQ(callsBefore, profileCallCount);
    EXPECT_EQ(6, getBatteryCellCount());
}

TEST_F(BatteryPresenceTest, SwapAfterLongDisarmedUptimeStillDetected)
{
    run(2000, 0);
    run(5000, 252);
    for (int i = 0; i < 40; i++) {
        run(60000, 252);     // 40 min disarmed, past the 32-bit microsecond half range
    }
    ramp(400, 252, 180);
    run(5000, 168);
    EXPECT_EQ(4, getBatteryCellCount());
    EXPECT_EQ(4, lastProfileCellCount);
}

TEST_F(BatteryPresenceTest, SwapClearsLowVoltageCutoff)
{
    batteryConfigMutable()->lvcPercentage = 50;
    run(2000, 0);
    run(5000, 168);
    armingFlags = ARMED;
    ramp(1000, 168, 120);    // 3.0V per cell under load
    run(5000, 120);
    ASSERT_TRUE(getLowVoltageCutoff()->enabled);
    armingFlags = 0;
    run(10000, 120);
    ramp(400, 120, 100);
    run(5000, 168);
    EXPECT_FALSE(getLowVoltageCutoff()->enabled);
    EXPECT_EQ(100, getLowVoltageCutoff()->percentage);
}

TEST_F(BatteryPresenceTest, EarlierBlipDoesNotArmALaterSlowDrift)
{
    run(2000, 0);
    run(5000, 252);
    const unsigned callsBefore = profileCallCount;
    run(6000, 252);          // holdoff over
    ramp(200, 252, 240);     // brief disturbance, settles back
    run(200, 240);
    run(5000, 252);
    ramp(30000, 252, 168);   // slow stable drift
    run(5000, 168);
    EXPECT_EQ(callsBefore, profileCallCount);
    EXPECT_EQ(6, getBatteryCellCount());
}
