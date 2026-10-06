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

// Expected values follow the datasheet relation Vdda = 3.3 V * VREFINT_CAL / VREFINT_DATA,
// and the temperature sample scaled by Vdda / 3.3 V before the two-point interpolation.

extern "C" {

#include "platform.h"
#include "drivers/adc.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(AdcInternalUnittest, VrefAtCalibrationVoltageIs3300)
{
    EXPECT_EQ(adcInternalCompensateVref(1500, 1500), 3300);
}

TEST(AdcInternalUnittest, LowerSupplyGivesHigherSampleAndLowerVref)
{
    // Vdda 3.0 V: sample = 1500 * 3.3 / 3.0 = 1650
    EXPECT_EQ(adcInternalCompensateVref(1500, 1650), 3000);
    // Vdda 3.6 V: sample = 1500 * 3.3 / 3.6 = 1375
    EXPECT_EQ(adcInternalCompensateVref(1500, 1375), 3600);
}

TEST(AdcInternalUnittest, ZeroInputGivesZeroVrefWithoutDividing)
{
    EXPECT_EQ(adcInternalCompensateVref(1500, 0), 0);
    EXPECT_EQ(adcInternalCompensateVref(0, 1500), 0);
}

TEST(AdcInternalUnittest, VrefClampsInsteadOfWrapping)
{
    EXPECT_EQ(adcInternalCompensateVref(65535, 1), UINT16_MAX);
}

// Calibration: 30 degC reads 1000, 110 degC reads 1300 at 3.3 V, so slopeK = 80000 / 300 = 266.
static constexpr uint16_t TS_CAL1 = 1000;
static constexpr int32_t SLOPE_K = 266;

TEST(AdcInternalUnittest, TemperatureAtCalibrationPoints)
{
    EXPECT_EQ(adcInternalComputeTemperature(1000, 3300, TS_CAL1, SLOPE_K), 30);
    EXPECT_EQ(adcInternalComputeTemperature(1300, 3300, TS_CAL1, SLOPE_K), 110);
}

TEST(AdcInternalUnittest, TemperatureIsIndependentOfSupplyVoltage)
{
    // 70 degC reads 1150 at 3.3 V. At 3.0 V the sample is 1150 * 3.3 / 3.0 = 1265.
    EXPECT_EQ(adcInternalComputeTemperature(1150, 3300, TS_CAL1, SLOPE_K), 70);
    EXPECT_EQ(adcInternalComputeTemperature(1265, 3000, TS_CAL1, SLOPE_K), 70);
}

// Physical model: at supply Vdda the ADC counts scale by 3.3 V / Vdda. Both samples are built from
// the model, not from the code under test, then run through the real Vref -> temperature chain.
TEST(AdcInternalUnittest, PipelineRecoversSupplyAndTemperatureAcrossSupplyRange)
{
    static constexpr uint16_t VREFINT_CAL = 1500;
    static constexpr int32_t TS_CAL2_MINUS_CAL1 = 300; // counts between 30 and 110 degC
    for (int vddaMv = 2800; vddaMv <= 3600; vddaMv += 100) {
        const uint16_t vrefintSample = (uint16_t)(VREFINT_CAL * 3300.0 / vddaMv);
        const uint16_t vrefMv = adcInternalCompensateVref(VREFINT_CAL, vrefintSample);
        EXPECT_NEAR(vrefMv, vddaMv, 3);
        for (int tempC = -10; tempC <= 110; tempC += 20) {
            const double countsAt3v3 = TS_CAL1 + (tempC - 30) * TS_CAL2_MINUS_CAL1 / 80.0;
            const uint16_t tempSample = (uint16_t)(countsAt3v3 * 3300.0 / vddaMv);
            EXPECT_NEAR(adcInternalComputeTemperature(tempSample, vrefMv, TS_CAL1, SLOPE_K), tempC, 2)
                << "Vdda " << vddaMv << " mV, " << tempC << " degC";
        }
    }
}

TEST(AdcInternalUnittest, MaximumInputsDoNotOverflow)
{
    // 16-bit samples, maximum Vref, zero calibration offset: the intermediate product must not wrap.
    const int16_t temp = adcInternalComputeTemperature(UINT16_MAX, UINT16_MAX, 0, 1);
    EXPECT_GT(temp, 30);
}

TEST(AdcInternalUnittest, ZeroSlopeGivesThirtyDegrees)
{
    // Equal calibration words set slopeK to 0; the result stays finite.
    EXPECT_EQ(adcInternalComputeTemperature(2000, 3300, 1000, 0), 30);
}
