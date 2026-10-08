/*
 * This file is part of EmuFlight. It is derived from Betaflight.
 *
 * This is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <string.h>
#include <vector>

extern "C" {
    #include "platform.h"
    #include "pg/pg.h"
    #include "io/gps.h"
    #include "io/serial.h"
    #include "fc/runtime_config.h"
    #include "io/dashboard.h"
    #include "drivers/serial.h"

    extern bool gpsNewFrame(uint8_t c);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// Frames are built from the u-blox protocol spec (offsets and Fletcher-8 checksum), not from the parser.
static std::vector<uint8_t> ubxFrame(uint8_t cls, uint8_t id, const std::vector<uint8_t> &payload)
{
    std::vector<uint8_t> f = {0xB5, 0x62, cls, id, (uint8_t)(payload.size() & 0xFF), (uint8_t)(payload.size() >> 8)};
    f.insert(f.end(), payload.begin(), payload.end());
    uint8_t a = 0, b = 0;
    for (size_t i = 2; i < f.size(); i++) {
        a += f[i];
        b += a;
    }
    f.push_back(a);
    f.push_back(b);
    return f;
}

static std::vector<uint8_t> navPvtPayload(uint8_t numSV, uint16_t pDop)
{
    std::vector<uint8_t> p(92, 0);
    p[20] = 3;                // fixType: 3D
    p[21] = 0x01;             // flags: gnssFixOK
    p[23] = numSV;            // numSV, offset 23
    p[76] = pDop & 0xFF;      // pDOP, offset 76
    p[77] = pDop >> 8;
    return p;
}

static std::vector<uint8_t> navSolPayload(uint8_t numSV, uint16_t pDop)
{
    std::vector<uint8_t> p(52, 0);
    p[10] = 3;                // gpsFix: 3D
    p[11] = 0x01;             // flags: gpsFixOk
    p[44] = pDop & 0xFF;      // pDOP, offset 44
    p[45] = pDop >> 8;
    p[47] = numSV;            // numSV, offset 47
    return p;
}

static void feed(const std::vector<uint8_t> &f)
{
    for (uint8_t c : f) {
        gpsNewFrame(c);
    }
}

class GpsUbloxTest : public ::testing::Test {
protected:
    void SetUp() override {
        gpsConfigMutable()->provider = GPS_UBLOX;
        memset(&gpsSol, 0, sizeof(gpsSol));
    }
};

TEST_F(GpsUbloxTest, NavPvtSetsSatelliteCountAndDop)
{
    feed(ubxFrame(0x01, 0x07, navPvtPayload(14, 123)));
    EXPECT_EQ(14, gpsSol.numSat);
    EXPECT_EQ(123, gpsSol.hdop);
}

TEST_F(GpsUbloxTest, NavPvtZeroSatellitesOverwritesPreviousCount)
{
    gpsSol.numSat = 9;
    feed(ubxFrame(0x01, 0x07, navPvtPayload(0, 0)));
    EXPECT_EQ(0, gpsSol.numSat);
}

TEST_F(GpsUbloxTest, NavPvtMaxSatelliteCountIsNotTruncated)
{
    feed(ubxFrame(0x01, 0x07, navPvtPayload(255, 0)));
    EXPECT_EQ(255, gpsSol.numSat);
}

TEST_F(GpsUbloxTest, NavPvtBadChecksumIsRejected)
{
    std::vector<uint8_t> f = ubxFrame(0x01, 0x07, navPvtPayload(14, 0));
    f.back() ^= 0xFF;
    gpsSol.numSat = 3;
    feed(f);
    EXPECT_EQ(3, gpsSol.numSat);
    // Parser must resync on the next good frame.
    feed(ubxFrame(0x01, 0x07, navPvtPayload(11, 0)));
    EXPECT_EQ(11, gpsSol.numSat);
}

TEST_F(GpsUbloxTest, NavPvtShortPayloadIsRejected)
{
    std::vector<uint8_t> p = navPvtPayload(14, 0);
    p.resize(24);             // numSV present but frame is shorter than the 92-byte spec length
    gpsSol.numSat = 3;
    feed(ubxFrame(0x01, 0x07, p));
    EXPECT_EQ(3, gpsSol.numSat);
}

TEST_F(GpsUbloxTest, MessageId7InOtherClassIsIgnored)
{
    gpsSol.numSat = 3;
    feed(ubxFrame(0x0A, 0x07, navPvtPayload(14, 0)));   // class MON, id 0x07
    EXPECT_EQ(3, gpsSol.numSat);
}

TEST_F(GpsUbloxTest, NavSolStillSetsSatelliteCount)
{
    feed(ubxFrame(0x01, 0x06, navSolPayload(8, 150)));
    EXPECT_EQ(8, gpsSol.numSat);
    EXPECT_EQ(150, gpsSol.hdop);
}

// Stubs
extern "C" {
    uint8_t stateFlags;
    uint8_t armingFlags;
    const uint32_t baudRates[] = {0, 9600, 19200, 38400, 57600, 115200, 230400, 250000, 400000};

    uint32_t millis(void) { return 0; }
    uint32_t micros(void) { return 0; }
    bool feature(uint32_t) { return false; }
    bool sensors(uint32_t) { return false; }
    void sensorsSet(uint32_t) {}
    void sensorsClear(uint32_t) {}
    void dashboardUpdate(timeUs_t) {}
    void dashboardShowFixedPage(pageId_e) {}
    serialPortConfig_t *findSerialPortConfig(serialPortFunction_e) { return NULL; }
    serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e) { return NULL; }
    void waitForSerialPortToFinishTransmitting(serialPort_t *) {}
    baudRate_e lookupBaudRateIndex(uint32_t) { return BAUD_AUTO; }
    void serialPassthrough(serialPort_t *, serialPort_t *, serialConsumer *, serialConsumer *) {}
    void serialWrite(serialPort_t *, uint8_t) {}
    uint32_t serialRxBytesWaiting(const serialPort_t *) { return 0; }
    uint8_t serialRead(serialPort_t *) { return 0; }
    void serialSetBaudRate(serialPort_t *, uint32_t) {}
    void serialSetMode(serialPort_t *, portMode_e) {}
    bool isSerialTransmitBufferEmpty(const serialPort_t *) { return true; }
    void serialPrint(serialPort_t *, const char *) {}
    uint32_t serialGetBaudRate(serialPort_t *) { return 0; }
}
