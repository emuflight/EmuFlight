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
#include <algorithm>
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
    extern void gpsInitUblox(void);
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

// NAV-SAT per the u-blox spec: 8-byte header (iTOW, version, numSvs, reserved), then 12 bytes per satellite:
// gnssId, svId, cno, elev, azim (I2), prRes (I2), flags (U4, bits 0-2 = qualityInd).
static std::vector<uint8_t> navSatPayload(uint8_t numSvsField, uint8_t countInPayload, uint8_t firstSvId, uint8_t gnssId = 0)
{
    std::vector<uint8_t> p(8 + 12 * countInPayload, 0);
    p[4] = 1;                 // version
    p[5] = numSvsField;       // numSvs
    for (uint8_t i = 0; i < countInPayload; i++) {
        uint8_t *sv = &p[8 + 12 * i];
        sv[0] = gnssId;                   // gnssId (0 = GPS)
        sv[1] = firstSvId + i;            // svId
        sv[2] = 20 + i;                   // cno
        sv[8] = 0x0D;                     // flags low byte: qualityInd 5 (bits 0-2), svUsed (bit 3)
    }
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
        GPS_numCh = 0;
        memset(GPS_svinfo_chn, 0, sizeof(GPS_svinfo_chn));
        memset(GPS_svinfo_svid, 0, sizeof(GPS_svinfo_svid));
        memset(GPS_svinfo_quality, 0, sizeof(GPS_svinfo_quality));
        memset(GPS_svinfo_cno, 0, sizeof(GPS_svinfo_cno));
        stateFlags = 0;
        armingFlags = 0;
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

TEST_F(GpsUbloxTest, NavPvt84ByteU7VariantIsAccepted)
{
    std::vector<uint8_t> p = navPvtPayload(9, 77);
    p.resize(84);             // u-blox 7 NAV-PVT length per its protocol spec
    feed(ubxFrame(0x01, 0x07, p));
    EXPECT_EQ(9, gpsSol.numSat);
    EXPECT_EQ(77, gpsSol.hdop);
}

TEST_F(GpsUbloxTest, NavPvtOneByteTooShortForPdopIsRejected)
{
    std::vector<uint8_t> p = navPvtPayload(9, 77);
    p.resize(77);             // pDOP occupies bytes 76-77 of the payload; 77 bytes cuts it
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

    uint32_t testMillis = 0;
    uint32_t millis(void) { return testMillis; }
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
    std::vector<uint8_t> *testTx = NULL;
    void serialWrite(serialPort_t *, uint8_t c) { if (testTx) testTx->push_back(c); }
    uint32_t serialRxBytesWaiting(const serialPort_t *) { return 0; }
    uint8_t serialRead(serialPort_t *) { return 0; }
    void serialSetBaudRate(serialPort_t *, uint32_t) {}
    void serialSetMode(serialPort_t *, portMode_e) {}
    bool isSerialTransmitBufferEmpty(const serialPort_t *) { return true; }
    void serialPrint(serialPort_t *, const char *) {}
    uint32_t serialGetBaudRate(serialPort_t *) { return 0; }
}

TEST_F(GpsUbloxTest, NavSatFillsSignalTable)
{
    feed(ubxFrame(0x01, 0x35, navSatPayload(3, 3, 5, 6)));
    EXPECT_EQ(3, GPS_numCh);
    for (int i = 0; i < 3; i++) {
        EXPECT_EQ(6, GPS_svinfo_chn[i]);   // gnssId 6 = GLONASS
        EXPECT_EQ(5 + i, GPS_svinfo_svid[i]);
        EXPECT_EQ(20 + i, GPS_svinfo_cno[i]);
        EXPECT_EQ(5, GPS_svinfo_quality[i]);
    }
}

TEST_F(GpsUbloxTest, NavSatLongerThanReceiveBufferKeepsFirst16)
{
    // 40 satellites = 488-byte payload, above the 344-byte receive buffer
    feed(ubxFrame(0x01, 0x35, navSatPayload(40, 40, 1)));
    EXPECT_EQ(16, GPS_numCh);
    EXPECT_EQ(1, GPS_svinfo_svid[0]);
    EXPECT_EQ(16, GPS_svinfo_svid[15]);
    EXPECT_EQ(35, GPS_svinfo_cno[15]);
}

TEST_F(GpsUbloxTest, NavSatCountFieldLargerThanPayloadIsClamped)
{
    // header claims 10 satellites, payload holds 2
    feed(ubxFrame(0x01, 0x35, navSatPayload(10, 2, 7)));
    EXPECT_EQ(2, GPS_numCh);
}

TEST_F(GpsUbloxTest, NavSatBadChecksumOnLongFrameIsRejected)
{
    std::vector<uint8_t> f = ubxFrame(0x01, 0x35, navSatPayload(40, 40, 1));
    f.back() ^= 0xFF;
    GPS_numCh = 4;
    feed(f);
    EXPECT_EQ(4, GPS_numCh);
    // Parser must resync on the next good frame.
    feed(ubxFrame(0x01, 0x35, navSatPayload(2, 2, 9)));
    EXPECT_EQ(2, GPS_numCh);
}

TEST_F(GpsUbloxTest, NavSatShorterThanHeaderIsIgnored)
{
    GPS_numCh = 4;
    feed(ubxFrame(0x01, 0x35, std::vector<uint8_t>(5, 0)));
    EXPECT_EQ(4, GPS_numCh);
}

TEST_F(GpsUbloxTest, MessageId35InOtherClassIsIgnored)
{
    GPS_numCh = 4;
    feed(ubxFrame(0x02, 0x35, navSatPayload(3, 3, 5)));
    EXPECT_EQ(4, GPS_numCh);
}

TEST_F(GpsUbloxTest, OversizedNonSatMessageIsStillSkipped)
{
    // only NAV-SAT may exceed the buffer; a 400-byte NAV-PVT frame must not be parsed
    std::vector<uint8_t> p = navPvtPayload(14, 0);
    p.resize(400, 0);
    gpsSol.numSat = 3;
    feed(ubxFrame(0x01, 0x07, p));
    EXPECT_EQ(3, gpsSol.numSat);
}

TEST_F(GpsUbloxTest, NavSatShorterMessageClearsEntriesOfEarlierOne)
{
    feed(ubxFrame(0x01, 0x35, navSatPayload(10, 10, 1)));
    EXPECT_EQ(10, GPS_numCh);
    feed(ubxFrame(0x01, 0x35, navSatPayload(3, 3, 50)));
    EXPECT_EQ(3, GPS_numCh);
    for (int i = 3; i < 16; i++) {
        EXPECT_EQ(0, GPS_svinfo_svid[i]);
        EXPECT_EQ(0, GPS_svinfo_cno[i]);
        EXPECT_EQ(0, GPS_svinfo_quality[i]);
        EXPECT_EQ(0, GPS_svinfo_chn[i]);
    }
}

TEST_F(GpsUbloxTest, NavSatSixteenAndSeventeenEntriesAreCappedAtSixteen)
{
    feed(ubxFrame(0x01, 0x35, navSatPayload(16, 16, 1)));
    EXPECT_EQ(16, GPS_numCh);
    EXPECT_EQ(16, GPS_svinfo_svid[15]);
    // distinct svIds: the table must come from this frame, not the previous one
    feed(ubxFrame(0x01, 0x35, navSatPayload(17, 17, 101)));
    EXPECT_EQ(16, GPS_numCh);
    EXPECT_EQ(101, GPS_svinfo_svid[0]);
    EXPECT_EQ(116, GPS_svinfo_svid[15]);
}

TEST_F(GpsUbloxTest, NavSatPayloadsOfExactlyAndJustOverBufferSize)
{
    feed(ubxFrame(0x01, 0x35, navSatPayload(28, 28, 1)));   // 8 + 28*12 = 344 bytes
    EXPECT_EQ(16, GPS_numCh);
    EXPECT_EQ(1, GPS_svinfo_svid[0]);
    feed(ubxFrame(0x01, 0x35, navSatPayload(29, 29, 101)));   // 356 bytes, distinct svIds
    EXPECT_EQ(16, GPS_numCh);
    EXPECT_EQ(101, GPS_svinfo_svid[0]);
    EXPECT_EQ(116, GPS_svinfo_svid[15]);
    // a valid frame right after the long one must still parse
    feed(ubxFrame(0x01, 0x35, navSatPayload(2, 2, 9)));
    EXPECT_EQ(2, GPS_numCh);
    EXPECT_EQ(9, GPS_svinfo_svid[0]);
}

TEST_F(GpsUbloxTest, NavSatZeroEntriesAndPartialLastEntry)
{
    feed(ubxFrame(0x01, 0x35, navSatPayload(5, 5, 1)));
    feed(ubxFrame(0x01, 0x35, navSatPayload(0, 0, 1)));
    EXPECT_EQ(0, GPS_numCh);
    std::vector<uint8_t> p = navSatPayload(3, 2, 20);
    p.insert(p.end(), 5, 0);                                // 5 bytes of a third entry
    feed(ubxFrame(0x01, 0x35, p));
    EXPECT_EQ(2, GPS_numCh);
}

// Mixed constellations: svId repeats per constellation, so the pair (gnssId, svId) is what tells satellites apart.
TEST_F(GpsUbloxTest, NavSatKeepsGnssIdPerEntry)
{
    std::vector<uint8_t> p = navSatPayload(2, 2, 4);
    p[8 + 0] = 0;  p[8 + 1] = 4;    // GPS 4
    p[8 + 12] = 6; p[8 + 13] = 4;   // GLONASS 4
    feed(ubxFrame(0x01, 0x35, p));
    EXPECT_EQ(2, GPS_numCh);
    EXPECT_EQ(0, GPS_svinfo_chn[0]);
    EXPECT_EQ(6, GPS_svinfo_chn[1]);
    EXPECT_EQ(4, GPS_svinfo_svid[0]);
    EXPECT_EQ(4, GPS_svinfo_svid[1]);
}

// UBX-MON-VER: swVersion[30], hwVersion[10], extension strings (30 bytes each).
static std::vector<uint8_t> monVerPayload(const char *hw, size_t extensions = 2)
{
    std::vector<uint8_t> p(40 + 30 * extensions, 0);
    strncpy((char *)&p[0], "EXT CORE 1.00", 30);
    strncpy((char *)&p[30], hw, 10);
    return p;
}

static void expectMonVer(const char *hw, ubloxVersion_e expected)
{
    gpsData.platformVersion = UBX_VERSION_UNDEF;
    feed(ubxFrame(0x0A, 0x04, monVerPayload(hw)));
    EXPECT_EQ(expected, gpsData.platformVersion) << hw;
}

TEST_F(GpsUbloxTest, MonVerMapsHardwareVersionToGeneration)
{
    expectMonVer("00040005", UBX_VERSION_M5);
    expectMonVer("00040007", UBX_VERSION_M6);
    expectMonVer("00070000", UBX_VERSION_M7);
    expectMonVer("00080000", UBX_VERSION_M8);
    expectMonVer("00190000", UBX_VERSION_M9);
    expectMonVer("000A0000", UBX_VERSION_M10);
}

TEST_F(GpsUbloxTest, MonVerUnknownHardwareStaysUndefined)
{
    expectMonVer("00ABCDEF", UBX_VERSION_UNDEF);
    expectMonVer("ZZZZZZZZ", UBX_VERSION_UNDEF);
}

TEST_F(GpsUbloxTest, MonVerShorterThanFixedPartIsIgnored)
{
    gpsData.platformVersion = UBX_VERSION_UNDEF;
    std::vector<uint8_t> p = monVerPayload("000A0000");
    p.resize(39);
    feed(ubxFrame(0x0A, 0x04, p));
    EXPECT_EQ(UBX_VERSION_UNDEF, gpsData.platformVersion);
}

TEST_F(GpsUbloxTest, MonVerHardwareFieldWithoutNulTerminatorIsParsed)
{
    // all ten bytes used, no NUL: "000A000012" would be read as a different number if the copy were unbounded
    gpsData.platformVersion = UBX_VERSION_UNDEF;
    std::vector<uint8_t> p = monVerPayload("000A0000");
    p[38] = '1'; p[39] = '2';
    feed(ubxFrame(0x0A, 0x04, p));
    EXPECT_EQ(UBX_VERSION_UNDEF, gpsData.platformVersion);   // 0x000A000012 is not a known hardware value
}

TEST_F(GpsUbloxTest, MonVerLongerThanReceiveBufferIsStillParsed)
{
    // many extension strings, as a receiver with several modules reports
    gpsData.platformVersion = UBX_VERSION_UNDEF;
    feed(ubxFrame(0x0A, 0x04, monVerPayload("00080000", 20)));
    EXPECT_EQ(UBX_VERSION_M8, gpsData.platformVersion);
}

TEST_F(GpsUbloxTest, MonVerOfAnotherClassIsIgnored)
{
    gpsData.platformVersion = UBX_VERSION_UNDEF;
    feed(ubxFrame(0x01, 0x04, monVerPayload("000A0000")));
    EXPECT_EQ(UBX_VERSION_UNDEF, gpsData.platformVersion);
}

// Drive the configure state machine and capture what it writes.
class GpsUbloxConfigTest : public GpsUbloxTest {
protected:
    std::vector<uint8_t> tx;
    void SetUp() override {
        GpsUbloxTest::SetUp();
        tx.clear();
        testTx = &tx;
        testMillis = 0;
        gpsConfigMutable()->autoConfig = GPS_AUTOCONFIG_ON;
        gpsConfigMutable()->gps_ublox_use_galileo = false;
        gpsData.state = 3;                        // GPS_CONFIGURE
        gpsData.state_position = 0;
        gpsData.messageState = GPS_MESSAGE_STATE_IDLE;
        gpsData.platformVersion = UBX_VERSION_UNDEF;
    }
    void TearDown() override { testTx = NULL; }
    // run until the configure state is left; `answer` is delivered when the version poll has been sent
    void run(const char *hw) {
        for (int i = 0; i < 4000 && gpsData.state == 3; i++) {
            gpsInitUblox();
            testMillis += 300;
            if (hw && !tx.empty() && gpsData.messageState == GPS_MESSAGE_STATE_VERSION && gpsData.platformVersion == UBX_VERSION_UNDEF) {
                feed(ubxFrame(0x0A, 0x04, monVerPayload(hw)));
            }
        }
    }
    // rate for a NAV message in the CFG-MSG frames found in tx, or -1 when none was sent
    int rateOf(uint8_t msgId) const {
        for (size_t i = 0; i + 11 <= tx.size(); i++) {
            if (tx[i] == 0xB5 && tx[i + 1] == 0x62 && tx[i + 2] == 0x06 && tx[i + 3] == 0x01 &&
                tx[i + 4] == 0x03 && tx[i + 5] == 0x00 && tx[i + 6] == 0x01 && tx[i + 7] == msgId) {
                // checksum over class..payload
                uint8_t a = 0, b = 0;
                for (size_t k = 2; k < 9; k++) { a += tx[i + k]; b += a; }
                EXPECT_EQ(a, tx[i + 9]);
                EXPECT_EQ(b, tx[i + 10]);
                return tx[i + 8];
            }
        }
        return -1;
    }
    bool polled() const {
        static const uint8_t poll[] = {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};
        return std::search(tx.begin(), tx.end(), poll, poll + sizeof(poll)) != tx.end();
    }
};

TEST_F(GpsUbloxConfigTest, M10GetsPvtAndSatOnly)
{
    run("000A0000");
    EXPECT_TRUE(polled());
    EXPECT_EQ(1, rateOf(0x07));    // PVT
    EXPECT_EQ(5, rateOf(0x35));    // SAT
    EXPECT_EQ(-1, rateOf(0x06));   // no SOL
    EXPECT_EQ(-1, rateOf(0x30));   // no SVINFO
    EXPECT_EQ(UBX_VERSION_M10, gpsData.platformVersion);
}

TEST_F(GpsUbloxConfigTest, M8DisablesSolAndSvinfo)
{
    run("00080000");
    EXPECT_EQ(1, rateOf(0x07));
    EXPECT_EQ(5, rateOf(0x35));
    EXPECT_EQ(0, rateOf(0x06));
    EXPECT_EQ(0, rateOf(0x30));
}

TEST_F(GpsUbloxConfigTest, M7UsesPvtAndSvinfo)
{
    run("00070000");
    EXPECT_EQ(1, rateOf(0x07));
    EXPECT_EQ(5, rateOf(0x30));
    EXPECT_EQ(0, rateOf(0x06));
    EXPECT_EQ(0, rateOf(0x35));
}

TEST_F(GpsUbloxConfigTest, M6UsesSolAndSvinfoOnly)
{
    run("00040007");
    EXPECT_EQ(1, rateOf(0x06));
    EXPECT_EQ(5, rateOf(0x30));
    EXPECT_EQ(0, rateOf(0x07));
    EXPECT_EQ(0, rateOf(0x35));
}

TEST_F(GpsUbloxConfigTest, SilentReceiverGetsEveryMessageAfterBoundedPolling)
{
    run(NULL);
    EXPECT_EQ(UBX_VERSION_UNDEF, gpsData.platformVersion);
    EXPECT_EQ(1, rateOf(0x06));
    EXPECT_EQ(1, rateOf(0x07));
    EXPECT_EQ(5, rateOf(0x30));
    EXPECT_EQ(5, rateOf(0x35));
    EXPECT_NE(3, gpsData.state);   // the state machine left GPS_CONFIGURE
}

TEST_F(GpsUbloxConfigTest, AutoConfigOffSendsNothing)
{
    gpsConfigMutable()->autoConfig = GPS_AUTOCONFIG_OFF;
    run("000A0000");
    EXPECT_TRUE(tx.empty());
}
