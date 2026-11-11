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
#include "rx/xbus.h"
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

// CRC16-CCITT (poly 0x1021, init 0), big endian in the frame.
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

// RJ01 outer CRC8: reflected polynomial 0x8C, init 0, over all bytes but the last.
static uint8_t crc8Rj01(const uint8_t *data, unsigned len)
{
    uint8_t crc = 0;
    for (unsigned i = 0; i < len; i++) {
        uint8_t in = data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = ((crc ^ in) & 1) ? (uint8_t)((crc >> 1) ^ 0x8C) : (uint8_t)(crc >> 1);
            in >>= 1;
        }
    }
    return crc;
}

// MODE B: start byte, N 16-bit big-endian channels (0x800 = 1500 us), CRC16 over everything before it.
static void makeModeB(uint8_t *out, uint8_t startByte, unsigned channels)
{
    const unsigned len = 1 + 2 * channels;
    out[0] = startByte;
    for (unsigned i = 0; i < channels; i++) {
        out[1 + 2 * i] = 0x08;
        out[2 + 2 * i] = 0x00;
    }
    const uint16_t crc = crc16Xmodem(out, len);
    out[len] = crc >> 8;
    out[len + 1] = crc & 0xFF;
}

// RJ01: outer 33 bytes = 0xA1, 30 (message length), 0, 27-byte MODE B frame, 0, 0, CRC8.
static void makeRj01(uint8_t *out)
{
    memset(out, 0, 33);
    out[0] = 0xA1;
    out[1] = 30;
    makeModeB(&out[3], 0xA1, 12);
    out[32] = crc8Rj01(out, 32);
}

class XBusFrameTest : public ::testing::Test
{
protected:
    rxRuntimeConfig_t runtime = {};

    void init(uint8_t provider)
    {
        rxConfig_t config = {};
        config.serialrx_provider = provider;
        microseconds_stub_value += 100000;
        ASSERT_TRUE(xBusInit(&config, &runtime));
        ASSERT_FALSE(NULL == stub_serialRxCallback);
    }

    uint8_t send(const uint8_t *data, unsigned len)
    {
        for (unsigned i = 0; i < len; i++) {
            microseconds_stub_value += 50;
            stub_serialRxCallback(data[i], NULL);
        }
        return runtime.rcFrameStatusFn(&runtime);
    }
};

TEST_F(XBusFrameTest, Rj01AcceptsFull33ByteFrame)
{
    init(SERIALRX_XBUS_MODE_B_RJ01);
    uint8_t frame[33];
    makeRj01(frame);
    EXPECT_EQ(RX_FRAME_COMPLETE, send(frame, sizeof(frame)));
    for (unsigned ch = 0; ch < 12; ch++) {
        EXPECT_EQ(1500, runtime.rcReadRawFn(&runtime, ch));
    }
}

TEST_F(XBusFrameTest, Rj01RejectsBadOuterCrcThenRecovers)
{
    init(SERIALRX_XBUS_MODE_B_RJ01);
    uint8_t frame[33];
    makeRj01(frame);
    frame[32] ^= 1;
    EXPECT_EQ(RX_FRAME_PENDING, send(frame, sizeof(frame)));

    microseconds_stub_value += 20000;
    makeRj01(frame);
    EXPECT_EQ(RX_FRAME_COMPLETE, send(frame, sizeof(frame)));
}

TEST_F(XBusFrameTest, Rj01RejectsBadInnerCrc)
{
    init(SERIALRX_XBUS_MODE_B_RJ01);
    uint8_t frame[33];
    makeRj01(frame);
    frame[28] ^= 1;
    frame[32] = crc8Rj01(frame, 32);
    EXPECT_EQ(RX_FRAME_PENDING, send(frame, sizeof(frame)));
}

TEST_F(XBusFrameTest, Rj01DoesNotCompleteOnPartialFrame)
{
    init(SERIALRX_XBUS_MODE_B_RJ01);
    uint8_t frame[33];
    makeRj01(frame);
    EXPECT_EQ(RX_FRAME_PENDING, send(frame, 32));
}

TEST_F(XBusFrameTest, Rj01IgnoresA2StartByte)
{
    init(SERIALRX_XBUS_MODE_B_RJ01);
    uint8_t frame[35];
    makeModeB(frame, 0xA2, 16);
    EXPECT_EQ(RX_FRAME_PENDING, send(frame, sizeof(frame)));
}

TEST_F(XBusFrameTest, ModeBAccepts12ChannelFrame)
{
    init(SERIALRX_XBUS_MODE_B);
    uint8_t frame[27];
    makeModeB(frame, 0xA1, 12);
    EXPECT_EQ(RX_FRAME_COMPLETE, send(frame, sizeof(frame)));
    EXPECT_EQ(1500, runtime.rcReadRawFn(&runtime, 11));
}

TEST_F(XBusFrameTest, ModeBAccepts16ChannelFrame)
{
    init(SERIALRX_XBUS_MODE_B);
    uint8_t frame[35];
    makeModeB(frame, 0xA2, 16);
    EXPECT_EQ(RX_FRAME_COMPLETE, send(frame, sizeof(frame)));
    EXPECT_EQ(1500, runtime.rcReadRawFn(&runtime, 0));
}
