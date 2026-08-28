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

#include <cstring>

extern "C" {

#include "platform.h"

#if !defined(UNUSED)
#define UNUSED(x) (void)(x)
#endif

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "pg/sdcard.h"

#include "drivers/sdcard.h"
#include "drivers/sdcard_standard.h"

// Real driver under test calls into this fake bus_spi.h/io.h/time.h surface instead of
// bus_spi.c: sdcard.c only ever reaches the SPI bus through spiSequence()'s segment API, so
// scripting responses at that one boundary is enough to drive the real command/data-block state
// machine -- same technique bus_spi_unittest.cc uses one layer down (fake DMA registration
// instead of fake bus_spi.h calls) to let its own target file link and run host-side.

static timeMs_t fakeMillis;

timeMs_t millis(void) {
    return fakeMillis;
}

timeUs_t micros(void) {
    return fakeMillis * 1000;
}

void delay(timeMs_t ms) {
    fakeMillis += ms;
}

void delayMicroseconds(timeUs_t us) {
    UNUSED(us);
}

static void resetFakeMillis(void) {
    fakeMillis = 0;
}

static void testAdvanceMillis(timeMs_t delta) {
    fakeMillis += delta;
}

IO_t IOGetByTag(ioTag_t tag) {
    UNUSED(tag);
    return (IO_t)1;
}

void IOInit(IO_t io, resourceOwner_e owner, uint8_t index) {
    UNUSED(io); UNUSED(owner); UNUSED(index);
}

void IOConfigGPIO(IO_t io, ioConfig_t cfg) {
    UNUSED(io); UNUSED(cfg);
}

void IOHi(IO_t io) {
    UNUSED(io);
}

void IOLo(IO_t io) {
    UNUSED(io);
}

bool IORead(IO_t io) {
    UNUSED(io);
    return true;
}

// Scripted card-response byte stream. sdcard.c's own idleCount-bounded retries (not a loop cap
// here) guarantee spiSequence() below always terminates.
static uint8_t responseBytes[128];
static int responseLen;
static int responseIndex;

static void resetResponseScript(void) {
    responseLen = 0;
    responseIndex = 0;
}

static void scriptBytes(const uint8_t *bytes, int count) {
    for (int i = 0; i < count; i++) {
        responseBytes[responseLen++] = bytes[i];
    }
}

// Once the script is exhausted, behave like a silent bus that never leaves the idle state --
// matches a card that has stopped responding, and is what drives the adversarial test's
// escalation to SDCARD_STATE_NOT_PRESENT rather than a hang.
static uint8_t popResponseByte(void) {
    if (responseIndex < responseLen) {
        return responseBytes[responseIndex++];
    }
    return SDCARD_IDLE_TOKEN;
}

static busDevice_t fakeBus;

bool spiSetBusInstance(extDevice_t *dev, uint32_t device) {
    UNUSED(device);
    dev->bus = &fakeBus;
    return true;
}

void spiSetClkDivisor(const extDevice_t *dev, uint16_t divisor) {
    UNUSED(dev); UNUSED(divisor);
}

uint16_t spiCalculateDivider(uint32_t freq) {
    UNUSED(freq);
    return 2;
}

// Mirrors the polled segment loop in bus_spi_ll.c: read segments pull from the scripted
// response queue, write-only/filler segments don't touch it, and BUS_BUSY/BUS_ABORT/BUS_READY
// are honored exactly as the real bus does.
void spiSequence(const extDevice_t *dev, busSegment_t *segments) {
    busDevice_t *bus = dev->bus;
    bus->curSegment = segments;

    while (bus->curSegment->len) {
        busSegment_t *segment = (busSegment_t *)bus->curSegment;

        if (segment->u.buffers.rxData) {
            for (int i = 0; i < segment->len; i++) {
                segment->u.buffers.rxData[i] = popResponseByte();
            }
        }

        bool segmentComplete = true;
        if (segment->callback) {
            switch (segment->callback(dev->callbackArg)) {
            case BUS_BUSY:
                segmentComplete = false;
                break;
            case BUS_ABORT:
                return;
            case BUS_READY:
            default:
                break;
            }
        }

        if (segmentComplete) {
            bus->curSegment = segment + 1;
        }
    }
}

void spiWait(const extDevice_t *dev) {
    UNUSED(dev);
}

void spiRelease(const extDevice_t *dev) {
    UNUSED(dev);
}

bool spiIsBusy(const extDevice_t *dev) {
    UNUSED(dev);
    // spiSequence() above always runs a segment chain to completion synchronously.
    return false;
}

bool spiUseDMA(const extDevice_t *dev) {
    UNUSED(dev);
    return false;
}

void spiReadWriteBuf(const extDevice_t *dev, uint8_t *txData, uint8_t *rxData, int len) {
    UNUSED(dev); UNUSED(txData);
    if (rxData) {
        for (int i = 0; i < len; i++) {
            rxData[i] = popResponseByte();
        }
    }
}

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

namespace {

sdcardConfig_t testConfig;

void resetSdcardTestState() {
    resetFakeMillis();
    resetResponseScript();
    memset(&fakeBus, 0, sizeof(fakeBus));
    memset(&testConfig, 0, sizeof(testConfig));
    testConfig.enabled = 1;
    testConfig.device = 0;
    testConfig.chipSelectTag = 0;
    testConfig.cardDetectTag = 0; // no detect pin -> sdcard_isInserted() always true
}

// CMD0/CMD8/ACMD41/CMD58/CMD9/CMD10 init sequence for a version-2 (SDHC) card, scripted
// against the SD physical layer spec's R1/R7/data-token wire format -- not copied from any
// driver output. Each command's [idle-wait byte, R1 reply byte] pair precedes its data phase.
void scriptSuccessfulHighCapacityInit(void) {
    static const uint8_t cmd0[] = {0xFF, 0x01};                          // GO_IDLE_STATE -> idle
    static const uint8_t cmd8[] = {0xFF, 0x01};                          // SEND_IF_COND -> idle
    static const uint8_t ifCondReply[] = {0x00, 0x00, 0x01, 0xAB};       // echoed voltage + check pattern
    static const uint8_t cmd55[] = {0xFF, 0x01};                         // APP_CMD (response ignored)
    static const uint8_t cmd41[] = {0xFF, 0x00};                         // ACMD41 -> init done
    static const uint8_t cmd58[] = {0xFF, 0x00};                         // READ_OCR -> ok
    static const uint8_t ocr[] = {0x40, 0x00, 0x00, 0x00};               // bit 30 set: high capacity
    static const uint8_t cmd9[] = {0xFF, 0x00};                          // SEND_CSD -> ok
    static const uint8_t csdToken[] = {0xFE};
    // CSD v2: bits[0:1]=01 (structure v2), CSIZE(bits 58:79)=0 -> (0+1)*1024 = 1024 blocks,
    // bit 127 (TRAILER, LSB of last byte) = 1. All other fields unused by sdcard_fetchCSD().
    static const uint8_t csd[16] = {0x40, 0, 0, 0, 0, 0, 0, 0x00, 0x00, 0x00, 0, 0, 0, 0, 0, 0x01};
    static const uint8_t csdCrc[] = {0x00, 0x00};
    static const uint8_t cmd10[] = {0xFF, 0x00};                         // SEND_CID -> ok
    static const uint8_t cidToken[] = {0xFE};
    static const uint8_t cid[16] = {
        0x03,                         // manufacturerID
        0x54, 0x4D,                   // oemID = "TM"
        0x53, 0x44, 0x54, 0x53, 0x54, // productName = "SDTST"
        0x10,                         // revision 1.0
        0x12, 0x34, 0x56, 0x78,       // productSerial
        0xA1, 0x86,                   // productionYear=2024, productionMonth=6
        0x00,                         // CRC (unused)
    };
    static const uint8_t cidCrc[] = {0x00, 0x00};

    scriptBytes(cmd0, sizeof(cmd0));
    scriptBytes(cmd8, sizeof(cmd8));
    scriptBytes(ifCondReply, sizeof(ifCondReply));
    scriptBytes(cmd55, sizeof(cmd55));
    scriptBytes(cmd41, sizeof(cmd41));
    scriptBytes(cmd58, sizeof(cmd58));
    scriptBytes(ocr, sizeof(ocr));
    scriptBytes(cmd9, sizeof(cmd9));
    scriptBytes(csdToken, sizeof(csdToken));
    scriptBytes(csd, sizeof(csd));
    scriptBytes(csdCrc, sizeof(csdCrc));
    scriptBytes(cmd10, sizeof(cmd10));
    scriptBytes(cidToken, sizeof(cidToken));
    scriptBytes(cid, sizeof(cid));
    scriptBytes(cidCrc, sizeof(cidCrc));
}

} // namespace

TEST(SdcardUnittest, HighCapacityCardReachesReadyInOnePoll) {
    resetSdcardTestState();
    scriptSuccessfulHighCapacityInit();

    sdcard_init(&testConfig);
    ASSERT_TRUE(sdcard_isFunctional());

    EXPECT_TRUE(sdcard_poll());
    EXPECT_TRUE(sdcard_isInitialized());

    const sdcardMetadata_t *metadata = sdcard_getMetadata();
    EXPECT_EQ(1024u, metadata->numBlocks);
    EXPECT_EQ(0x03, metadata->manufacturerID);
    EXPECT_EQ(0x544D, metadata->oemID);
    EXPECT_EQ(2024, metadata->productionYear);
    EXPECT_EQ(6, metadata->productionMonth);
}

TEST(SdcardUnittest, SilentBusNeverReachesReadyAndEscalatesToNotPresent) {
    resetSdcardTestState();
    // Empty script: popResponseByte() always returns SDCARD_IDLE_TOKEN, so every reply-wait
    // times out after SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY retries without ever reporting
    // the R1 idle-status bit sdcard_poll() needs to advance past SDCARD_STATE_RESET.

    sdcard_init(&testConfig);
    ASSERT_TRUE(sdcard_isFunctional());

    // SDCARD_MAX_CONSECUTIVE_FAILURES resets, each gated by SDCARD_TIMEOUT_INIT_MILLIS.
    for (int attempt = 0; attempt < 8; attempt++) {
        EXPECT_FALSE(sdcard_poll());
        EXPECT_FALSE(sdcard_isInitialized());
        testAdvanceMillis(250);
    }
    sdcard_poll();

    EXPECT_FALSE(sdcard_isFunctional());
    EXPECT_FALSE(sdcard_isInitialized());
}
