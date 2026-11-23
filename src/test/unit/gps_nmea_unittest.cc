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
#include "fc/runtime_config.h"
#include "io/gps.h"

void testGpsNmeaReset(void);
unsigned testGpsNmeaFeed(const char *text);
uint32_t testGpsNmeaPacketCount(void);
bool testGpsNmeaFix(void);

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
    testGpsNmeaFeed(frame(std::string("GNGGA,123519,4807.038,N,01131.000,E,1,08,") + longField + ",545.4,M,46.9,M,,").c_str());
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

// GSV: only the GP talker fills the satellite arrays; the satellite count never exceeds the array size.
TEST_F(GpsNmeaTest, GsvOnlyFromGpTalker)
{
    GPS_numCh = 0;
    testGpsNmeaFeed(frame("GLGSV,1,1,05").c_str());
    EXPECT_EQ(0, GPS_numCh);
    testGpsNmeaFeed(frame("GPGSV,1,1,07").c_str());
    EXPECT_EQ(7, GPS_numCh);
}

TEST_F(GpsNmeaTest, GsvSatelliteCountCappedToArraySize)
{
    testGpsNmeaFeed(frame("GPGSV,3,1,18").c_str());
    EXPECT_EQ(16, GPS_numCh);
}
