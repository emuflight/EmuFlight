/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdint.h>

extern "C" {

#include "platform.h"
#include "target.h"
#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_bmp280.h"
#include "drivers/bus.h"

void bmp280_calculate(int32_t *pressure, int32_t *temperature);
void bmp280_get_up(baroDev_t *baro);

extern uint32_t bmp280_up;
extern uint32_t bmp280_ut;
extern bool bmp280SampleValid;



extern bmp280_calib_param_t bmp280_cal;
}


#include "unittest_macros.h"
#include "gtest/gtest.h"


TEST(baroBmp280Test, TestBmp280Calculate)
{
    // given
    int32_t pressure, temperature;
    bmp280_up = 415148; // Digital pressure value
    bmp280_ut = 519888; // Digital temperature value
    bmp280SampleValid = true;

    // and
    bmp280_cal.dig_T1 = 27504;
    bmp280_cal.dig_T2 = 26435;
    bmp280_cal.dig_T3 = -1000;
    bmp280_cal.dig_P1 = 36477;
    bmp280_cal.dig_P2 = -10685;
    bmp280_cal.dig_P3 = 3024;
    bmp280_cal.dig_P4 = 2855;
    bmp280_cal.dig_P5 = 140;
    bmp280_cal.dig_P6 = -7;
    bmp280_cal.dig_P7 = 15500;
    bmp280_cal.dig_P8 = -14600;
    bmp280_cal.dig_P9 = 6000;

    // when
    bmp280_calculate(&pressure, &temperature);

    // then
    EXPECT_EQ(100653, pressure); // 100653 Pa
    EXPECT_EQ(2508, temperature); // 25.08 degC
}

TEST(baroBmp280Test, TestBmp280CalculateHighP)
{
    // given
    int32_t pressure, temperature;
    bmp280_up = 215148; // Digital pressure value
    bmp280_ut = 519888; // Digital temperature value
    bmp280SampleValid = true;

    // and
    bmp280_cal.dig_T1 = 27504;
    bmp280_cal.dig_T2 = 26435;
    bmp280_cal.dig_T3 = -1000;
    bmp280_cal.dig_P1 = 36477;
    bmp280_cal.dig_P2 = -10685;
    bmp280_cal.dig_P3 = 3024;
    bmp280_cal.dig_P4 = 2855;
    bmp280_cal.dig_P5 = 140;
    bmp280_cal.dig_P6 = -7;
    bmp280_cal.dig_P7 = 15500;
    bmp280_cal.dig_P8 = -14600;
    bmp280_cal.dig_P9 = 6000;

    // when
    bmp280_calculate(&pressure, &temperature);

    // then
    EXPECT_EQ(135382, pressure); // 135385 Pa
    EXPECT_EQ(2508, temperature); // 25.08 degC
}

TEST(baroBmp280Test, TestBmp280CalculateZeroP)
{
    // given
    int32_t pressure, temperature;
    bmp280_up = 415148; // Digital pressure value
    bmp280_ut = 519888; // Digital temperature value
    bmp280SampleValid = true;

    // and
    bmp280_cal.dig_T1 = 27504;
    bmp280_cal.dig_T2 = 26435;
    bmp280_cal.dig_T3 = -1000;
    bmp280_cal.dig_P1 = 0;
    bmp280_cal.dig_P2 = -10685;
    bmp280_cal.dig_P3 = 3024;
    bmp280_cal.dig_P4 = 2855;
    bmp280_cal.dig_P5 = 140;
    bmp280_cal.dig_P6 = -7;
    bmp280_cal.dig_P7 = 15500;
    bmp280_cal.dig_P8 = -14600;
    bmp280_cal.dig_P9 = 6000;

    // when
    bmp280_calculate(&pressure, &temperature);

    // then
    EXPECT_EQ(0, pressure); // P1=0 trips pressure to 0 Pa, avoiding division by zero
    EXPECT_EQ(2508, temperature); // 25.08 degC
}

static bool testReadResult = true;
static uint8_t testReadData[6];

static void setDatasheetCalibration(void)
{
    bmp280_cal.dig_T1 = 27504;
    bmp280_cal.dig_T2 = 26435;
    bmp280_cal.dig_T3 = -1000;
    bmp280_cal.dig_P1 = 36477;
    bmp280_cal.dig_P2 = -10685;
    bmp280_cal.dig_P3 = 3024;
    bmp280_cal.dig_P4 = 2855;
    bmp280_cal.dig_P5 = 140;
    bmp280_cal.dig_P6 = -7;
    bmp280_cal.dig_P7 = 15500;
    bmp280_cal.dig_P8 = -14600;
    bmp280_cal.dig_P9 = 6000;
}

// Datasheet example: raw pressure 415148 = 0x655AC, raw temperature 519888 = 0x7EED0,
// each a 20-bit value split as MSB, LSB, XLSB[7:4].
static void setDatasheetFrame(void)
{
    testReadData[0] = 0x65; testReadData[1] = 0x5A; testReadData[2] = 0xC0;
    testReadData[3] = 0x7E; testReadData[4] = 0xED; testReadData[5] = 0x00;
}

TEST(baroBmp280Test, TestBmp280GetUpValidFrame)
{
    baroDev_t baro = {};
    testReadResult = true;
    setDatasheetFrame();
    setDatasheetCalibration();
    bmp280_up = 0;
    bmp280_ut = 0;

    bmp280_get_up(&baro);

    int32_t pressure = -1, temperature = -1;
    bmp280_calculate(&pressure, &temperature);
    EXPECT_EQ(415148, bmp280_up);
    EXPECT_EQ(519888, bmp280_ut);
    EXPECT_EQ(100653, pressure);
    EXPECT_EQ(2508, temperature);
}

TEST(baroBmp280Test, TestBmp280FailedReadYieldsInvalidPressureAndKeepsLastRaw)
{
    baroDev_t baro = {};
    setDatasheetCalibration();
    // a good sample first, so stale data exists
    testReadResult = true;
    setDatasheetFrame();
    bmp280_get_up(&baro);

    // then a failed read; the buffer must not be parsed
    testReadResult = false;
    bmp280_get_up(&baro);

    int32_t pressure = 555, temperature = 777;
    bmp280_calculate(&pressure, &temperature);
    EXPECT_EQ(0, pressure);
    EXPECT_EQ(777, temperature); // untouched
    EXPECT_EQ(415148, bmp280_up);
    EXPECT_EQ(519888, bmp280_ut);
}

TEST(baroBmp280Test, TestBmp280RecoversAfterFailedRead)
{
    baroDev_t baro = {};
    setDatasheetCalibration();
    testReadResult = false;
    bmp280_get_up(&baro);

    testReadResult = true;
    setDatasheetFrame();
    bmp280_get_up(&baro);

    int32_t pressure = 0, temperature = 0;
    bmp280_calculate(&pressure, &temperature);
    EXPECT_EQ(100653, pressure);
    EXPECT_EQ(2508, temperature);
}

TEST(baroBmp280Test, TestBmp280CalculateInvalidSampleAcceptsNullTemperature)
{
    bmp280SampleValid = false;
    int32_t pressure = 9;
    bmp280_calculate(&pressure, NULL);
    EXPECT_EQ(0, pressure);
    bmp280_calculate(NULL, NULL);
}

// STUBS

extern "C" {

void delay(uint32_t) {}
bool busReadRegisterBuffer(const extDevice_t*, uint8_t, uint8_t *data, uint8_t length)
{
    if (testReadResult) {
        for (uint8_t i = 0; i < length; i++) {
            data[i] = testReadData[i];
        }
    }
    return testReadResult;
}
bool busWriteRegister(const extDevice_t*, uint8_t, uint8_t) {return true;}

void spiSetDivisor() {
}

void spiPreinitCsByIO() {
}

void IOConfigGPIO() {
}

void IOHi() {
}

void IOInit() {
}

void IORelease() {
}


}
