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
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdint.h>
#include <stdbool.h>

extern "C" {
#include "platform.h"
#include "drivers/barometer/barometer.h"
#include "sensors/barometer.h"
#include "sensors/sensors.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// Spec: the bus calls are synchronous. A failed operation keeps the state and is retried
// no faster than once per conversion period (up_delay); a successful state change returns
// the driver delay of the state just entered, or 1 ms after a read or sample state.

extern "C" {
uint8_t detectedSensors[SENSOR_INDEX_COUNT];
void sensorsSet(uint32_t) {}
// the EF busBusy() always returns false (synchronous bus calls)
bool busBusy(const extDevice_t *, bool *) { return false; }
void schedulerIgnoreTaskStateTime(void) {}
void schedulerIgnoreTaskExecTime(void) {}
}

static const uint16_t UT_DELAY_US = 10000;
static const uint16_t UP_DELAY_US = 23000;

static int startUtCalls, readUtCalls, getUtCalls, startUpCalls, readUpCalls, getUpCalls, calcCalls;
static bool startUpOk, readUpOk, getUpOk;
static int32_t calcPressure;

static bool startUt(baroDev_t *) { startUtCalls++; return true; }
static bool readUt(baroDev_t *) { readUtCalls++; return true; }
static bool getUt(baroDev_t *) { getUtCalls++; return true; }
static bool startUp(baroDev_t *) { startUpCalls++; return startUpOk; }
static bool readUp(baroDev_t *) { readUpCalls++; return readUpOk; }
static bool getUp(baroDev_t *) { getUpCalls++; return getUpOk; }
static void calc(int32_t *pressure, int32_t *temperature) { calcCalls++; *pressure = calcPressure; *temperature = 2000; }

static void resetFakeBaro(bool combined)
{
    startUtCalls = readUtCalls = getUtCalls = startUpCalls = readUpCalls = getUpCalls = calcCalls = 0;
    startUpOk = readUpOk = getUpOk = true;
    calcPressure = 101325;
    baro.dev.combined_read = combined;
    baro.dev.ut_delay = UT_DELAY_US;
    baro.dev.up_delay = UP_DELAY_US;
    baro.dev.start_ut = startUt;
    baro.dev.read_ut = readUt;
    baro.dev.get_ut = getUt;
    baro.dev.start_up = startUp;
    baro.dev.read_up = readUp;
    baro.dev.get_up = getUp;
    baro.dev.calculate = calc;
}

// The machine state is static. With all operations succeeding and combined_read set, the
// first call that returns up_delay is PRESSURE_START; two more calls finish READ and SAMPLE,
// leaving the machine at PRESSURE_START again.
static void syncToPressureStart(void)
{
    resetFakeBaro(true);
    for (int i = 0; i < 8 && baroUpdate() != UP_DELAY_US; i++) {
    }
    baroUpdate();
    baroUpdate();
    resetFakeBaro(true);
}

TEST(baroUpdateTest, CombinedCycleSkipsTemperatureStates)
{
    syncToPressureStart();
    EXPECT_EQ(UP_DELAY_US, baroUpdate());   // PRESSURE_START
    EXPECT_EQ(1000u, baroUpdate());         // PRESSURE_READ
    EXPECT_EQ(1000u, baroUpdate());         // PRESSURE_SAMPLE
    EXPECT_EQ(1, calcCalls);
    EXPECT_EQ(UP_DELAY_US, baroUpdate());   // back at PRESSURE_START
    EXPECT_EQ(0, startUtCalls);
    EXPECT_EQ(2, startUpCalls);
}

TEST(baroUpdateTest, FailedReadBacksOffToConversionPeriodAndKeepsState)
{
    syncToPressureStart();
    EXPECT_EQ(UP_DELAY_US, baroUpdate());   // PRESSURE_START ok
    readUpOk = false;
    for (int i = 0; i < 3; i++) {
        EXPECT_EQ(UP_DELAY_US, baroUpdate()); // failed read: not the 1 ms pace
    }
    EXPECT_EQ(3, readUpCalls);
    EXPECT_EQ(1, startUpCalls);             // state kept: no new conversion start
    EXPECT_EQ(0, getUpCalls);
    readUpOk = true;
    EXPECT_EQ(1000u, baroUpdate());         // recovers: read ok
    EXPECT_EQ(4, readUpCalls);
}

TEST(baroUpdateTest, FailedStartRetriesAtConversionPeriod)
{
    syncToPressureStart();
    startUpOk = false;
    EXPECT_EQ(UP_DELAY_US, baroUpdate());   // PRESSURE_START fails
    EXPECT_EQ(UP_DELAY_US, baroUpdate());   // retried at the conversion period
    EXPECT_EQ(2, startUpCalls);
    EXPECT_EQ(0, readUpCalls);
    startUpOk = true;
    EXPECT_EQ(UP_DELAY_US, baroUpdate());   // start ok: conversion wait
    EXPECT_EQ(3, startUpCalls);
    EXPECT_EQ(1000u, baroUpdate());
    EXPECT_EQ(1, readUpCalls);
}

TEST(baroUpdateTest, FailedGetKeepsStateAndBacksOff)
{
    syncToPressureStart();
    EXPECT_EQ(UP_DELAY_US, baroUpdate());   // PRESSURE_START
    EXPECT_EQ(1000u, baroUpdate());         // PRESSURE_READ
    getUpOk = false;
    EXPECT_EQ(UP_DELAY_US, baroUpdate());   // PRESSURE_SAMPLE: get fails
    EXPECT_EQ(UP_DELAY_US, baroUpdate());
    EXPECT_EQ(0, calcCalls);                // no calculation from a failed read
    getUpOk = true;
    EXPECT_EQ(1000u, baroUpdate());
    EXPECT_EQ(1, calcCalls);
}

TEST(baroUpdateTest, SeparateTemperatureCycleWhenNotCombined)
{
    syncToPressureStart();
    resetFakeBaro(false);
    EXPECT_EQ(UP_DELAY_US, baroUpdate());   // PRESSURE_START
    EXPECT_EQ(1000u, baroUpdate());         // PRESSURE_READ
    EXPECT_EQ(1000u, baroUpdate());         // PRESSURE_SAMPLE -> TEMPERATURE_START
    EXPECT_EQ(UT_DELAY_US, baroUpdate());   // TEMPERATURE_START
    EXPECT_EQ(1, startUtCalls);
    EXPECT_EQ(1000u, baroUpdate());         // TEMPERATURE_READ
    EXPECT_EQ(1000u, baroUpdate());         // TEMPERATURE_SAMPLE -> PRESSURE_START
    EXPECT_EQ(UP_DELAY_US, baroUpdate());
}
