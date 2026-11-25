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
#include <stdio.h>
#include <string>

extern "C" {
#include "platform.h"

#include "common/gps_conversion.h"
#include "common/time.h"
#include "fc/runtime_config.h"
#include "io/gps.h"

void testGpsNmeaReset(void);
unsigned testGpsNmeaFeed(const char *text);
uint32_t testGpsNmeaPacketCount(void);
bool testGpsNmeaFix(void);
bool testGpsNmeaRtc(dateTime_t *dt);

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// NMEA 0183: the checksum is the XOR of every character between '$' and '*'.
static std::string frame(const std::string &body)
{
    uint8_t x = 0;
    for (char c : body) {
        x ^= static_cast<uint8_t>(c);
    }
    char tail[8];
    snprintf(tail, sizeof(tail), "*%02X\r\n", x);
    return "$" + body + tail;
}

static const char *const GGA_FIELDS = ",123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";

class GpsNmeaTest : public ::testing::Test {
protected:
    void SetUp() override { testGpsNmeaReset(); }
};

// Reference frame from the NMEA 0183 description: 48 07.038 N, 11 31.000 E, 8 satellites.
TEST_F(GpsNmeaTest, GpTalkerGgaAccepted)
{
    EXPECT_EQ(1u, testGpsNmeaFeed(frame(std::string("GPGGA") + GGA_FIELDS).c_str()));
    EXPECT_TRUE(testGpsNmeaFix());
    EXPECT_EQ(8, gpsSol.numSat);
    EXPECT_EQ(481173000, gpsSol.llh.lat);   // 48 + 7.038/60 degrees, 1e7 scale
    EXPECT_EQ(115166666, gpsSol.llh.lon);   // 11 + 31.000/60 degrees, 1e7 scale
}

TEST_F(GpsNmeaTest, GnTalkerGgaAccepted)
{
    EXPECT_EQ(1u, testGpsNmeaFeed(frame(std::string("GNGGA") + GGA_FIELDS).c_str()));
    EXPECT_TRUE(testGpsNmeaFix());
    EXPECT_EQ(8, gpsSol.numSat);
    EXPECT_EQ(481173000, gpsSol.llh.lat);
    EXPECT_EQ(115166666, gpsSol.llh.lon);
}

TEST_F(GpsNmeaTest, GnTalkerRmcAccepted)
{
    // 22.4 knots = 1152 cm/s (1 knot = 51.4444 cm/s); course 84.4 degrees as 844.
    testGpsNmeaFeed(frame("GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W").c_str());
    EXPECT_EQ(1u, testGpsNmeaPacketCount());
    EXPECT_EQ(1152, gpsSol.groundSpeed);
    EXPECT_EQ(844, gpsSol.groundCourse);
}

TEST_F(GpsNmeaTest, OtherTalkerIdsAccepted)
{
    EXPECT_EQ(1u, testGpsNmeaFeed(frame(std::string("GLGGA") + GGA_FIELDS).c_str()));
    EXPECT_EQ(1u, testGpsNmeaFeed(frame(std::string("BDGGA") + GGA_FIELDS).c_str()));
}

TEST_F(GpsNmeaTest, UnknownSentenceTypeIgnored)
{
    testGpsNmeaFeed(frame("GNGLL,4916.45,N,12311.12,W,225444,A").c_str());
    EXPECT_EQ(1u, testGpsNmeaPacketCount());   // checksum valid, counted
    EXPECT_FALSE(testGpsNmeaFix());
    EXPECT_EQ(0, gpsSol.numSat);
}

TEST_F(GpsNmeaTest, BadChecksumRejected)
{
    std::string f = frame(std::string("GNGGA") + GGA_FIELDS);
    f.replace(f.find('*') + 1, 2, "00");   // the correct value is 59
    EXPECT_EQ(0u, testGpsNmeaFeed(f.c_str()));
    EXPECT_EQ(0u, testGpsNmeaPacketCount());
    EXPECT_EQ(0, gpsSol.numSat);
}

// A field of 15 or more characters must not corrupt the parser; the next frame still decodes.
TEST_F(GpsNmeaTest, OverlongFieldDoesNotCorruptParser)
{
    const char *longField = "12345678901234567890123456789012345678901234567890";
    testGpsNmeaFeed(frame(std::string("GPGGA,123519,4807.038,N,01131.000,E,1,08,") + longField + ",545.4,M,46.9,M,,").c_str());
    testGpsNmeaFeed(frame(std::string("GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,") + std::string(15, '9') + ",M,46.9,M,,").c_str());
    testGpsNmeaReset();
    EXPECT_EQ(1u, testGpsNmeaFeed(frame(std::string("GNGGA") + GGA_FIELDS).c_str()));
    EXPECT_EQ(8, gpsSol.numSat);
    EXPECT_EQ(481173000, gpsSol.llh.lat);
}

// A one-character sentence ID must not match a stale type left by the previous frame.
TEST_F(GpsNmeaTest, ShortSentenceIdIgnored)
{
    testGpsNmeaFeed(frame(std::string("GPGGA") + GGA_FIELDS).c_str());
    testGpsNmeaReset();
    testGpsNmeaFeed(frame(std::string("X") + GGA_FIELDS).c_str());
    EXPECT_FALSE(testGpsNmeaFix());
    EXPECT_EQ(0, gpsSol.numSat);
    EXPECT_EQ(0, gpsSol.llh.lat);
}

// GSV: only the GP talker fills the satellite arrays.
TEST_F(GpsNmeaTest, GsvOnlyFromGpTalker)
{
    GPS_numCh = 0;
    testGpsNmeaFeed(frame("GLGSV,1,1,05").c_str());
    EXPECT_EQ(0, GPS_numCh);
    testGpsNmeaFeed(frame("GPGSV,1,1,07").c_str());
    EXPECT_EQ(7, GPS_numCh);
}

// NMEA 0183 RMC time is hhmmss.s...: the fraction has any digit count. 23 March 2026 is 230326.
static bool rmcTime(const char *time, dateTime_t *dt)
{
    testGpsNmeaReset();
    testGpsNmeaFeed(frame(std::string("GNRMC,") + time + ",A,4807.038,N,01131.000,E,022.4,084.4,230326,003.1,W").c_str());
    return testGpsNmeaRtc(dt);
}

TEST_F(GpsNmeaTest, RmcTimeFractionDigits)
{
    const struct { const char *text; uint16_t millis; } cases[] = {
        {"123519", 0}, {"123519.", 0}, {"123519.0", 0}, {"123519.5", 500}, {"123519.05", 50},
        {"123519.50", 500}, {"123519.25", 250}, {"123519.500", 500}, {"123519.123", 123},
        {"123519.1234", 123}, {"123519.12345678", 123},
    };
    for (const auto &c : cases) {
        dateTime_t dt;
        ASSERT_TRUE(rmcTime(c.text, &dt)) << c.text;
        EXPECT_EQ(2026, dt.year) << c.text;
        EXPECT_EQ(3, dt.month) << c.text;
        EXPECT_EQ(23, dt.day) << c.text;
        EXPECT_EQ(12, dt.hours) << c.text;
        EXPECT_EQ(35, dt.minutes) << c.text;
        EXPECT_EQ(19, dt.seconds) << c.text;
        EXPECT_EQ(c.millis, dt.millis) << c.text;
    }
}

TEST_F(GpsNmeaTest, RmcTimeWithGpTalker)
{
    testGpsNmeaFeed(frame("GPRMC,235959.99,A,4807.038,N,01131.000,E,022.4,084.4,311299,003.1,W").c_str());
    dateTime_t dt;
    ASSERT_TRUE(testGpsNmeaRtc(&dt));
    EXPECT_EQ(23, dt.hours);
    EXPECT_EQ(59, dt.minutes);
    EXPECT_EQ(59, dt.seconds);
    EXPECT_EQ(990, dt.millis);
}

TEST_F(GpsNmeaTest, TruncatedSentenceIgnored)
{
    std::string f = frame("GNRMC,123519.5,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W");
    f.erase(f.find('*'));   // checksum never arrives
    EXPECT_EQ(0u, testGpsNmeaFeed(f.c_str()));
    EXPECT_EQ(0u, testGpsNmeaPacketCount());
    dateTime_t dt;
    EXPECT_FALSE(testGpsNmeaRtc(&dt));
}

TEST_F(GpsNmeaTest, RmcBadChecksumDoesNotSetClock)
{
    std::string f = frame("GNRMC,123519.5,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W");
    f.replace(f.find('*') + 1, 2, "00");
    testGpsNmeaFeed(f.c_str());
    dateTime_t dt;
    EXPECT_FALSE(testGpsNmeaRtc(&dt));
}

// A number field with its '.' near the end of the buffer must not write past it (ASan checks this build).
TEST_F(GpsNmeaTest, LateDecimalPointInFieldStaysInBounds)
{
    for (int dot = 10; dot <= 14; dot++) {
        std::string field(dot, '1');
        field += ".5";
        testGpsNmeaFeed(frame("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9," + field + ",M,46.9,M,,").c_str());
        testGpsNmeaFeed(frame("GPRMC,123519," + field + ",A,4807.038,N,01131.000,E," + field + ",084.4,230394,003.1,W").c_str());
    }
    testGpsNmeaReset();
    EXPECT_EQ(1u, testGpsNmeaFeed(frame(std::string("GNGGA") + GGA_FIELDS).c_str()));
}
