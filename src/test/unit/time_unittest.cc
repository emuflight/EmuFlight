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
 * but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// Expected Unix seconds are calendar facts (for example 2^31 - 1 = 2038-01-19T03:14:07Z),
// not values read back from the code under test.

#include <stdint.h>
#include <string.h>

extern "C" {
    #include "platform.h"
    #include "common/time.h"
    #include "pg/pg.h"
    #include "drivers/serial.h"

    static uint32_t testMillis = 0;
    uint32_t millis(void) { return testMillis; }
    // printf.c links these for tfp_printf; this test only uses tfp_sprintf.
    bool isSerialTransmitBufferEmpty(const serialPort_t *) { return true; }
    void serialWrite(serialPort_t *, uint8_t) {}
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static dateTime_t makeDt(unsigned y, unsigned mo, unsigned d, unsigned h, unsigned mi, unsigned s, unsigned ms = 0)
{
    dateTime_t dt;
    dt.year = y;
    dt.month = mo;
    dt.day = d;
    dt.hours = h;
    dt.minutes = mi;
    dt.seconds = s;
    dt.millis = ms;
    return dt;
}

static void resetRtc(void)
{
    testMillis = 0;
    timeConfigMutable()->tz_offsetMinutes = 0;
    dateTime_t dt = makeDt(2020, 1, 1, 0, 0, 0);
    ASSERT_TRUE(rtcSetDateTime(&dt));
}

// RTC value in Unix milliseconds. millis() is 0, so rtcGet() returns the stored time.
static int64_t rtcMs(void)
{
    rtcTime_t t = -1;
    EXPECT_TRUE(rtcGet(&t));
    return t;
}

static void expectDt(const dateTime_t &dt, unsigned y, unsigned mo, unsigned d, unsigned h, unsigned mi, unsigned s, unsigned ms)
{
    EXPECT_EQ(y, dt.year);
    EXPECT_EQ(mo, dt.month);
    EXPECT_EQ(d, dt.day);
    EXPECT_EQ(h, dt.hours);
    EXPECT_EQ(mi, dt.minutes);
    EXPECT_EQ(s, dt.seconds);
    EXPECT_EQ(ms, dt.millis);
}

TEST(TimeUnittest, Start2000IsAccepted)
{
    resetRtc();
    dateTime_t dt = makeDt(2000, 1, 1, 0, 0, 0);
    EXPECT_TRUE(rtcSetDateTime(&dt));
    EXPECT_EQ(946684800000LL, rtcMs());
    dateTime_t out;
    EXPECT_TRUE(rtcGetDateTime(&out));
    expectDt(out, 2000, 1, 1, 0, 0, 0, 0);
}

TEST(TimeUnittest, DateBefore2000IsRejectedAndRtcKept)
{
    resetRtc();
    const int64_t before = rtcMs();
    const dateTime_t bad[] = {
        makeDt(1999, 12, 31, 23, 59, 59, 999),
        makeDt(1970, 1, 1, 0, 0, 0),
        makeDt(0, 1, 1, 0, 0, 0),
    };
    for (const dateTime_t &dt : bad) {
        dateTime_t copy = dt;
        EXPECT_FALSE(rtcSetDateTime(&copy));
        EXPECT_EQ(before, rtcMs());
    }
}

TEST(TimeUnittest, Overflow2038BoundaryIs64Bit)
{
    resetRtc();
    dateTime_t last32 = makeDt(2038, 1, 19, 3, 14, 7);   // 2^31 - 1
    dateTime_t first64 = makeDt(2038, 1, 19, 3, 14, 8);  // 2^31
    EXPECT_TRUE(rtcSetDateTime(&last32));
    EXPECT_EQ(2147483647000LL, rtcMs());
    EXPECT_TRUE(rtcSetDateTime(&first64));
    EXPECT_EQ(2147483648000LL, rtcMs());
    dateTime_t out;
    EXPECT_TRUE(rtcGetDateTime(&out));
    expectDt(out, 2038, 1, 19, 3, 14, 8, 0);

    dateTime_t later = makeDt(2038, 12, 31, 23, 59, 59, 250);
    EXPECT_TRUE(rtcSetDateTime(&later));
    EXPECT_TRUE(rtcGetDateTime(&out));
    expectDt(out, 2038, 12, 31, 23, 59, 59, 250);
}

TEST(TimeUnittest, LastSupportedInstantIs2099)
{
    resetRtc();
    dateTime_t last = makeDt(2099, 12, 31, 23, 59, 59, 999);
    EXPECT_TRUE(rtcSetDateTime(&last));
    EXPECT_EQ(4102444799999LL, rtcMs());
    dateTime_t out;
    EXPECT_TRUE(rtcGetDateTime(&out));
    expectDt(out, 2099, 12, 31, 23, 59, 59, 999);

    // 2100 is not a leap year; the 4-year day table is not valid there.
    const int64_t before = rtcMs();
    dateTime_t y2100 = makeDt(2100, 1, 1, 0, 0, 0);
    EXPECT_FALSE(rtcSetDateTime(&y2100));
    EXPECT_EQ(before, rtcMs());
}

TEST(TimeUnittest, MalformedFieldsAreRejectedWithoutTableOverrun)
{
    resetRtc();
    const int64_t before = rtcMs();
    const dateTime_t bad[] = {
        makeDt(2024, 0, 1, 0, 0, 0),       // month 0 made an unsigned index wrap
        makeDt(2024, 13, 1, 0, 0, 0),
        makeDt(2024, 255, 1, 0, 0, 0),
        makeDt(2024, 1, 0, 0, 0, 0),
        makeDt(2024, 1, 32, 0, 0, 0),
        makeDt(2024, 1, 1, 24, 0, 0),
        makeDt(2024, 1, 1, 0, 60, 0),
        makeDt(2024, 1, 1, 0, 0, 60),
        makeDt(2024, 1, 1, 0, 0, 0, 1000),
        makeDt(65535, 12, 31, 23, 59, 59, 999),
        makeDt(2024, 4, 31, 0, 0, 0),      // April has 30 days
        makeDt(2023, 2, 29, 0, 0, 0),      // not a leap year
        makeDt(2099, 2, 29, 0, 0, 0),
        makeDt(2024, 2, 30, 0, 0, 0),
    };
    for (const dateTime_t &dt : bad) {
        dateTime_t copy = dt;
        EXPECT_FALSE(rtcSetDateTime(&copy)) << dt.year << "-" << (int)dt.month << "-" << (int)dt.day;
        EXPECT_EQ(before, rtcMs());
    }
}

TEST(TimeUnittest, LeapDaysAreAccepted)
{
    resetRtc();
    dateTime_t out;
    dateTime_t leap2024 = makeDt(2024, 2, 29, 12, 0, 0);
    EXPECT_TRUE(rtcSetDateTime(&leap2024));
    EXPECT_EQ(1709208000000LL, rtcMs());
    EXPECT_TRUE(rtcGetDateTime(&out));
    expectDt(out, 2024, 2, 29, 12, 0, 0, 0);

    dateTime_t leap2000 = makeDt(2000, 2, 29, 0, 0, 0);
    EXPECT_TRUE(rtcSetDateTime(&leap2000));
    dateTime_t leap2096 = makeDt(2096, 2, 29, 0, 0, 0);
    EXPECT_TRUE(rtcSetDateTime(&leap2096));
    EXPECT_TRUE(rtcGetDateTime(&out));
    expectDt(out, 2096, 2, 29, 0, 0, 0, 0);
}

TEST(TimeUnittest, RawRtcTimeOutsideRangeIsRejected)
{
    resetRtc();
    const int64_t before = rtcMs();
    const rtcTime_t bad[] = {
        946684799999LL,   // 1999-12-31T23:59:59.999Z
        0,
        -1,
        INT64_MIN,
        4102444800000LL,  // 2100-01-01T00:00:00Z
        INT64_MAX,
    };
    for (rtcTime_t t : bad) {
        EXPECT_FALSE(rtcSet(&t));
        EXPECT_EQ(before, rtcMs());
    }
    rtcTime_t ok = 946684800000LL;
    EXPECT_TRUE(rtcSet(&ok));
    ok = 4102444799999LL;
    EXPECT_TRUE(rtcSet(&ok));
}

TEST(TimeUnittest, EveryDay2000To2099RoundTripsAndAdvancesBy86400s)
{
    resetRtc();
    // Independent calendar: walk one day at a time with the usual 4-year leap rule.
    static const unsigned mdays[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    unsigned y = 2000, mo = 1, d = 1;
    int64_t expectedMs = 946684800000LL + (12 * 3600 + 34 * 60 + 56) * 1000LL + 789;
    while (y <= 2099) {
        dateTime_t dt = makeDt(y, mo, d, 12, 34, 56, 789);
        ASSERT_TRUE(rtcSetDateTime(&dt)) << y << "-" << mo << "-" << d;
        ASSERT_EQ(expectedMs, rtcMs()) << y << "-" << mo << "-" << d;
        dateTime_t out;
        ASSERT_TRUE(rtcGetDateTime(&out));
        expectDt(out, y, mo, d, 12, 34, 56, 789);
        if (::testing::Test::HasFailure()) {
            return;
        }
        unsigned len = mdays[mo - 1] + ((mo == 2 && y % 4 == 0) ? 1 : 0);
        if (++d > len) {
            d = 1;
            if (++mo > 12) {
                mo = 1;
                ++y;
            }
        }
        expectedMs += 86400000LL;
    }
}

TEST(TimeUnittest, TimezoneOffsetCrossesBoundariesWithoutWrap)
{
    resetRtc();
    // UTC 2038-01-19T03:14:07 plus 60 minutes: 32-bit seconds overflow here.
    dateTime_t utc = makeDt(2038, 1, 19, 3, 14, 7);
    dateTime_t local;
    timeConfigMutable()->tz_offsetMinutes = 60;
    dateTimeUTCToLocal(&utc, &local);
    expectDt(local, 2038, 1, 19, 4, 14, 7, 0);

    char buf[FORMATTED_DATE_TIME_BUFSIZE];
    EXPECT_TRUE(dateTimeFormatLocal(buf, &utc));
    EXPECT_STREQ("2038-01-19T04:14:07.000+01:00", buf);

    // UTC 2000-01-01T00:30 minus 60 minutes is before 2000: invalid, not a wrapped date.
    timeConfigMutable()->tz_offsetMinutes = -60;
    dateTime_t start = makeDt(2000, 1, 1, 0, 30, 0);
    dateTimeUTCToLocal(&start, &local);
    EXPECT_EQ(0, local.year);
    EXPECT_FALSE(dateTimeFormatLocal(buf, &start));
    EXPECT_STREQ("0000-01-01T00:00:00.000-01:00", buf);

    // Offset that stays in range still works near the start.
    dateTime_t noon = makeDt(2000, 1, 1, 12, 0, 0);
    dateTimeUTCToLocal(&noon, &local);
    expectDt(local, 2000, 1, 1, 11, 0, 0, 0);
    timeConfigMutable()->tz_offsetMinutes = 0;
}

TEST(TimeUnittest, FormatRejectsInvalidDate)
{
    resetRtc();
    char buf[FORMATTED_DATE_TIME_BUFSIZE];
    dateTime_t bad = makeDt(1999, 12, 31, 23, 59, 59);
    EXPECT_FALSE(dateTimeFormatUTC(buf, &bad));
    dateTime_t good = makeDt(2038, 1, 19, 3, 14, 8, 5);
    EXPECT_TRUE(dateTimeFormatUTC(buf, &good));
    EXPECT_STREQ("2038-01-19T03:14:08.005+00:00", buf);
}

TEST(TimeUnittest, FormatChecksMonthLengthWithZeroOffset)
{
    resetRtc();
    char buf[FORMATTED_DATE_TIME_BUFSIZE];
    dateTime_t apr31 = makeDt(2024, 4, 31, 0, 0, 0);
    EXPECT_FALSE(dateTimeFormatUTC(buf, &apr31));
    dateTime_t apr30 = makeDt(2024, 4, 30, 0, 0, 0);
    EXPECT_TRUE(dateTimeFormatUTC(buf, &apr30));
    dateTime_t feb29 = makeDt(2023, 2, 29, 0, 0, 0);
    EXPECT_FALSE(dateTimeFormatUTC(buf, &feb29));
    dateTime_t leap = makeDt(2024, 2, 29, 0, 0, 0);
    EXPECT_TRUE(dateTimeFormatUTC(buf, &leap));
}

TEST(TimeUnittest, RtcTimeHelpersKeep64BitSeconds)
{
    rtcTime_t t = rtcTimeMake(2147483648LL, 7);
    EXPECT_EQ(2147483648007LL, t);
    EXPECT_EQ(2147483648LL, rtcTimeGetSeconds(&t));
    EXPECT_EQ(7, rtcTimeGetMillis(&t));
}
