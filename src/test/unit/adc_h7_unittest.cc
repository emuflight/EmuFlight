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

// Pin-to-ADC device masks below follow the H743 pin table (RM0433 Table 205).

extern "C" {

#include "platform.h"
#include "drivers/adc.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static constexpr uint8_t ADC1_ONLY = 1U << ADCDEV_1;
static constexpr uint8_t ADC3_ONLY = 1U << ADCDEV_3;
static constexpr uint8_t ADC12 = (1U << ADCDEV_1) | (1U << ADCDEV_2);
static constexpr uint8_t ADC123 = ADC12 | (1U << ADCDEV_3);

TEST(AdcH7Unittest, ConfiguredDeviceWinsWhenItReadsThePin)
{
    EXPECT_EQ(adcSelectDevice(ADC123, ADCDEV_3, ADC123), ADCDEV_3);
    EXPECT_EQ(adcSelectDevice(ADC12, ADCDEV_2, ADC123), ADCDEV_2);
}

TEST(AdcH7Unittest, Adc3OnlyPinMovesOffTheDefaultAdc1)
{
    EXPECT_EQ(adcSelectDevice(ADC3_ONLY, ADCDEV_1, ADC123), ADCDEV_3);
}

TEST(AdcH7Unittest, Adc1OnlyPinMovesOffAdc3)
{
    EXPECT_EQ(adcSelectDevice(ADC1_ONLY, ADCDEV_3, ADC123), ADCDEV_1);
}

TEST(AdcH7Unittest, FallbackTakesLowestUsableDevice)
{
    EXPECT_EQ(adcSelectDevice(ADC12, ADCDEV_3, ADC123), ADCDEV_1);
    EXPECT_EQ(adcSelectDevice(ADC12, ADCDEV_3, 1U << ADCDEV_2), ADCDEV_2);
}

TEST(AdcH7Unittest, FallbackSkipsDeviceWithoutDmaStream)
{
    EXPECT_EQ(adcSelectDevice(ADC3_ONLY, ADCDEV_1, ADC12), ADCINVALID);
}

TEST(AdcH7Unittest, UnsetOrOutOfRangeConfigFallsBack)
{
    EXPECT_EQ(adcSelectDevice(ADC3_ONLY, ADCINVALID, ADC123), ADCDEV_3);
    EXPECT_EQ(adcSelectDevice(ADC3_ONLY, (ADCDevice)ADCDEV_COUNT, ADC123), ADCDEV_3);
    EXPECT_EQ(adcSelectDevice(ADC3_ONLY, (ADCDevice)100, ADC123), ADCDEV_3);
}

TEST(AdcH7Unittest, NoCapableDeviceYieldsInvalid)
{
    EXPECT_EQ(adcSelectDevice(0, ADCDEV_1, ADC123), ADCINVALID);
    EXPECT_EQ(adcSelectDevice(ADC123, ADCDEV_1, 0), ADCDEV_1); // configured device is kept, stream checked later
    EXPECT_EQ(adcSelectDevice(ADC3_ONLY, ADCDEV_1, 0), ADCINVALID);
}
