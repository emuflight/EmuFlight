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
#include <string.h>

extern "C" {
#include "platform.h"
#include "pg/pg.h"
#include "pg/rx.h"
#include "drivers/serial.h"
#include "drivers/time.h"
#include "io/serial.h"
#include "rx/rx.h"
#include "rx/sumd.h"
#include "telemetry/telemetry.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
    serialPort_t *telemetrySharedPort = NULL;
    bool telemetryCheckRxPortShared(const serialPortConfig_t *) { return false; }
    uint32_t microseconds_stub_value = 0;
    uint32_t micros(void) { return microseconds_stub_value; }
}

static serialPort_t serialTestInstance;
static serialPortConfig_t serialTestInstanceConfig = {};
static serialReceiveCallbackPtr stub_serialRxCallback;

serialPortConfig_t *findSerialPortConfig(serialPortFunction_e)
{
    return &serialTestInstanceConfig;
}

serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e,
    serialReceiveCallbackPtr callback, void *, uint32_t, portMode_e, portOptions_e)
{
    stub_serialRxCallback = callback;
    return &serialTestInstance;
}

// SUMD frame: 0xA8, status, channel count N, N big-endian 16-bit channels, CRC16-CCITT (poly 0x1021, init 0) over all prior bytes, big endian.
static uint16_t crc16Xmodem(const uint8_t *data, unsigned len)
{
    uint16_t crc = 0;
    for (unsigned i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

// 12000 in SUMD units is 1500 us (8000..16000 maps to 1000..2000 us)
static unsigned makeFrame(uint8_t *out, uint8_t count, uint16_t channelValue)
{
    out[0] = 0xA8;
    out[1] = 0x01;
    out[2] = count;
    for (unsigned i = 0; i < count; i++) {
        out[3 + 2 * i] = channelValue >> 8;
        out[4 + 2 * i] = channelValue & 0xFF;
    }
    const unsigned len = 3 + 2 * count;
    const uint16_t crc = crc16Xmodem(out, len);
    out[len] = crc >> 8;
    out[len + 1] = crc & 0xFF;
    return len + 2;
}

class SumdFrameTest : public ::testing::Test
{
protected:
    rxRuntimeConfig_t runtime = {};

    virtual void SetUp()
    {
        const rxConfig_t config = {};
        microseconds_stub_value += 100000;
        ASSERT_TRUE(sumdInit(&config, &runtime));
        ASSERT_FALSE(NULL == stub_serialRxCallback);
        // flush any state left by a previous test: one gap longer than the inter-byte timeout
        microseconds_stub_value += 100000;
        stub_serialRxCallback(0x00, NULL);
        runtime.rcFrameStatusFn(&runtime);
    }

    // returns the status read after the last byte; bytes are 100 us apart
    uint8_t send(const uint8_t *data, unsigned len)
    {
        for (unsigned i = 0; i < len; i++) {
            microseconds_stub_value += 100;
            stub_serialRxCallback(data[i], NULL);
        }
        return runtime.rcFrameStatusFn(&runtime);
    }
};

TEST_F(SumdFrameTest, AcceptsValidSixChannelFrame)
{
    uint8_t frame[3 + 2 * 16 + 2];
    const unsigned len = makeFrame(frame, 6, 12000);
    EXPECT_EQ(RX_FRAME_COMPLETE, send(frame, len));
    for (unsigned ch = 0; ch < 6; ch++) {
        EXPECT_EQ(1500, runtime.rcReadRawFn(&runtime, ch));
    }
}

TEST_F(SumdFrameTest, AcceptsMaximumChannelCount)
{
    uint8_t frame[3 + 2 * 16 + 2];
    const unsigned len = makeFrame(frame, 16, 12000);
    EXPECT_EQ(RX_FRAME_COMPLETE, send(frame, len));
    EXPECT_EQ(1500, runtime.rcReadRawFn(&runtime, 15));
}

TEST_F(SumdFrameTest, RejectsCountJustAboveMaximumThenAcceptsValidFrame)
{
    uint8_t bad[3 + 2 * 17 + 2];
    const unsigned badLen = makeFrame(bad, 17, 12000);
    EXPECT_EQ(RX_FRAME_PENDING, send(bad, badLen));

    uint8_t good[3 + 2 * 16 + 2];
    const unsigned goodLen = makeFrame(good, 6, 10000);
    EXPECT_EQ(RX_FRAME_COMPLETE, send(good, goodLen));
    EXPECT_EQ(1250, runtime.rcReadRawFn(&runtime, 0));
}

TEST_F(SumdFrameTest, RejectsOversizedFrameAndIgnoresSyncByteInItsPayload)
{
    // count 40: 85 bytes in all. A complete valid 6-channel frame sits inside the rejected payload.
    uint8_t bad[3 + 2 * 40 + 2] = { 0xA8, 0x01, 40 };
    uint8_t inner[3 + 2 * 16 + 2];
    const unsigned innerLen = makeFrame(inner, 6, 12000);
    memcpy(&bad[3], inner, innerLen);
    EXPECT_EQ(RX_FRAME_PENDING, send(bad, sizeof(bad)));

    uint8_t good[3 + 2 * 16 + 2];
    const unsigned goodLen = makeFrame(good, 6, 10000);
    EXPECT_EQ(RX_FRAME_COMPLETE, send(good, goodLen));
    EXPECT_EQ(1250, runtime.rcReadRawFn(&runtime, 0));
}

TEST_F(SumdFrameTest, InterFrameGapEndsSkipOfCorruptedCount)
{
    // a corrupted count of 255 starts a 512-byte skip; a gap above 4 ms must end it
    const uint8_t header[] = { 0xA8, 0x01, 0xFF };
    EXPECT_EQ(RX_FRAME_PENDING, send(header, sizeof(header)));

    microseconds_stub_value += 5000;
    uint8_t good[3 + 2 * 16 + 2];
    const unsigned goodLen = makeFrame(good, 6, 12000);
    EXPECT_EQ(RX_FRAME_COMPLETE, send(good, goodLen));
}
