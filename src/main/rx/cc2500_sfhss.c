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

#include <stdbool.h>

#include "platform.h"

#ifdef USE_RX_SFHSS_SPI

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"
#include "pg/rx_spi.h"

#include "drivers/rx/rx_cc2500.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "fc/config.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"

#include "rx/cc2500_common.h"
#include "rx/cc2500_frsky_common.h"
#include "rx/cc2500_sfhss.h"

//void cliPrintLinef(const char *format, ...);

#define BIND_CH 15
#define SFHSS_PACKET_LEN   15
#define BIND_TUNE_STEP 4

#define SFHSSCH2CHANNR(ch)   (ch * 6 + 16)
#define GET_CHAN(x)     ((int)((x[5]>>3) & 0x1f))
#define GET_CODE(x)     (((x[11] & 0x7)<<2 ) | ((x[12]>>6) & 0x3))
#define GET_COMMAND(x)  (x[12] & 0xf)
#define GET_CH1(x)      ((uint16_t)(((x[5] & 0x07)<<9 | x[6]<<1) | (x[7] & 0x80)>>7))
#define GET_CH2(x)      (uint16_t)(((x[7] & 0x7f)<<5 | (x[8] & 0xf8)>>3))
#define GET_CH3(x)      (uint16_t)(((x[8] & 0x07)<<9 | x[9]<<1) | (x[10] & 0x80)>>7)
#define GET_CH4(x)      (uint16_t)(((x[10] & 0x7f)<<5 | (x[11] & 0xf8)>>3))
#define GET_TXID1(x)    (uint8_t)(x[1])
#define GET_TXID2(x)    (uint8_t)(x[2])
#define SET_STATE(x)    {protocolState = x; DEBUG_SET(DEBUG_RX_SFHSS_SPI, DEBUG_DATA_STATE, x);}

#define NEXT_CH_TIME_HUNT       500000      /* hunt */
#define NEXT_CH_TIME_SYNC0      6800        /* sync no recv */
#define NEXT_CH_TIME_SYNC1      3500        /* sync ch1-4 recv */
#define NEXT_CH_TIME_SYNC2      500         /* sync ch5-8 recv */

static int8_t sfhss_channel = 0;
static int8_t sfhss_code = 0;

static timeMs_t start_time;
static uint8_t protocolState;

static uint32_t missingPackets;

static uint8_t calData[32][3];
static timeMs_t timeTunedMs;
static int8_t bindOffset_max = 0;
static int8_t bindOffset_min = 0;

static void initialise() {
    cc2500Reset();
    cc2500WriteReg(CC2500_02_IOCFG0,   0x01);
    cc2500WriteReg(CC2500_03_FIFOTHR,  0x07);
    cc2500WriteReg(CC2500_04_SYNC1,    0xD3);
    cc2500WriteReg(CC2500_05_SYNC0,    0x91);
    cc2500WriteReg(CC2500_06_PKTLEN,   0x0D);
    cc2500WriteReg(CC2500_07_PKTCTRL1, 0x04);
    cc2500WriteReg(CC2500_08_PKTCTRL0, 0x0C);
    cc2500WriteReg(CC2500_09_ADDR,     0x29);
    cc2500WriteReg(CC2500_0B_FSCTRL1,  0x06);
    cc2500WriteReg(CC2500_0C_FSCTRL0,  (rxFrSkySpiConfigMutable()->bindOffset));
    cc2500WriteReg(CC2500_0D_FREQ2,    0x5C);
    cc2500WriteReg(CC2500_0E_FREQ1,    0x4E);
    cc2500WriteReg(CC2500_0F_FREQ0,    0xC4);
    cc2500WriteReg(CC2500_10_MDMCFG4,  0x7C);
    cc2500WriteReg(CC2500_11_MDMCFG3,  0x43);
    cc2500WriteReg(CC2500_12_MDMCFG2,  0x03);
    cc2500WriteReg(CC2500_13_MDMCFG1,  0x23);
    cc2500WriteReg(CC2500_14_MDMCFG0,  0x3B);
    cc2500WriteReg(CC2500_15_DEVIATN,  0x44);
    cc2500WriteReg(CC2500_17_MCSM1,    0x0F);
    cc2500WriteReg(CC2500_18_MCSM0,    0x08);
    cc2500WriteReg(CC2500_19_FOCCFG,   0x1D);
    cc2500WriteReg(CC2500_1A_BSCFG,    0x6C);
    cc2500WriteReg(CC2500_1B_AGCCTRL2, 0x03);
    cc2500WriteReg(CC2500_1C_AGCCTRL1, 0x40);
    cc2500WriteReg(CC2500_1D_AGCCTRL0, 0x91);
    cc2500WriteReg(CC2500_21_FREND1,   0x56);
    cc2500WriteReg(CC2500_22_FREND0,   0x10);
    cc2500WriteReg(CC2500_23_FSCAL3,   0xA9);
    cc2500WriteReg(CC2500_24_FSCAL2,   0x0A);
    cc2500WriteReg(CC2500_25_FSCAL1,   0x00);
    cc2500WriteReg(CC2500_26_FSCAL0,   0x11);
    cc2500WriteReg(CC2500_29_FSTEST,   0x59);
    cc2500WriteReg(CC2500_2C_TEST2,    0x88);
    cc2500WriteReg(CC2500_2D_TEST1,    0x31);
    cc2500WriteReg(CC2500_2E_TEST0,    0x0B);
    cc2500WriteReg(CC2500_3E_PATABLE,  0xFF);
    for (unsigned c = 0; c < 30; c++) {
        //calibrate all channels
        cc2500Strobe(CC2500_SIDLE);
        cc2500WriteReg(CC2500_0A_CHANNR, SFHSSCH2CHANNR(c));
        cc2500Strobe(CC2500_SCAL);
        delayMicroseconds(900);
        calData[c][0] = cc2500ReadReg(CC2500_23_FSCAL3);
        calData[c][1] = cc2500ReadReg(CC2500_24_FSCAL2);
        calData[c][2] = cc2500ReadReg(CC2500_25_FSCAL1);
    }
}

static bool sfhssRecv(uint8_t *packet) {
    uint8_t ccLen;
    if (!(cc2500getGdo())) {
        return false;
    }
    ccLen = cc2500ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
    if (ccLen < SFHSS_PACKET_LEN) {
        return false;
    }
    cc2500ReadFifo(packet, SFHSS_PACKET_LEN);
    return true;
}

static bool sfhssPacketParse(uint8_t *packet, bool check_txid) {
    if (!(packet[SFHSS_PACKET_LEN - 1] & 0x80)) {
        return false;      /* crc fail */
    }
    if (packet[0] != 0x81) {
        return false;              /* sfhss header fail */
    }
    if (GET_CHAN(packet) != sfhss_channel) {
        return false;           /* channel fail */
    }
    if (check_txid) {
        if ((rxFrSkySpiConfigMutable()->bindTxId[0] != GET_TXID1(packet)) ||
                (rxFrSkySpiConfigMutable()->bindTxId[1] != GET_TXID2(packet))) {
            return false;           /* txid fail */
        }
    }
    cc2500setRssiDbm(packet[SFHSS_PACKET_LEN - 2]);
    sfhss_code = GET_CODE(packet);
    return true;
}

void sfhssRx(void) {
    cc2500Strobe(CC2500_SIDLE);
    cc2500WriteReg(CC2500_23_FSCAL3, calData[sfhss_channel][0]);
    cc2500WriteReg(CC2500_24_FSCAL2, calData[sfhss_channel][1]);
    cc2500WriteReg(CC2500_25_FSCAL1, calData[sfhss_channel][2]);
    cc2500WriteReg(CC2500_0A_CHANNR, SFHSSCH2CHANNR(sfhss_channel));
    cc2500Strobe(CC2500_SFRX);
    cc2500Strobe(CC2500_SRX);
}

static void initTuneRx(void) {
    timeTunedMs = millis();
    bindOffset_min = -64;
    DEBUG_SET(DEBUG_RX_SFHSS_SPI, DEBUG_DATA_OFFSET_MIN, bindOffset_min);
    cc2500WriteReg(CC2500_0C_FSCTRL0, (uint8_t)bindOffset_min);
    sfhss_channel = BIND_CH;
    sfhssRx();
}

static bool tune1Rx(uint8_t *packet) {
    if (bindOffset_min >= 126) {
        bindOffset_min = -126;
        DEBUG_SET(DEBUG_RX_SFHSS_SPI, DEBUG_DATA_OFFSET_MIN, bindOffset_min);
    }
    if ((millis() - timeTunedMs) > 220) {   // 220ms
        timeTunedMs = millis();
        bindOffset_min += BIND_TUNE_STEP << 2;
        DEBUG_SET(DEBUG_RX_SFHSS_SPI, DEBUG_DATA_OFFSET_MIN, bindOffset_min);
        cc2500WriteReg(CC2500_0C_FSCTRL0, (uint8_t)bindOffset_min);
        cc2500Strobe(CC2500_SRX);
    }
    if (sfhssRecv(packet)) {
        if (sfhssPacketParse(packet, false)) {
            if ((packet[SFHSS_PACKET_LEN - 1] & 0x7F) > 40 ) {    /* lqi */
                rxFrSkySpiConfigMutable()->bindTxId[0] = GET_TXID1(packet);
                rxFrSkySpiConfigMutable()->bindTxId[1] = GET_TXID2(packet);
                bindOffset_max = bindOffset_min;
                DEBUG_SET(DEBUG_RX_SFHSS_SPI, DEBUG_DATA_OFFSET_MAX, bindOffset_max);
                cc2500Strobe(CC2500_SRX);
                timeTunedMs = millis();
                return true;
            }
        }
        cc2500Strobe(CC2500_SRX);
    }
    return false;
}

static bool tune2Rx(uint8_t *packet) {
    cc2500LedBlink(100);
    if (((millis() - timeTunedMs) > 880) || bindOffset_max > (126 - BIND_TUNE_STEP)) {   // 220ms *4
        timeTunedMs = millis();
        cc2500WriteReg(CC2500_0C_FSCTRL0, (uint8_t)bindOffset_min);
        cc2500Strobe(CC2500_SRX);
        return true;
    }
    if (sfhssRecv(packet)) {
        if (sfhssPacketParse(packet, true)) {
            timeTunedMs = millis();
            bindOffset_max += BIND_TUNE_STEP;
            DEBUG_SET(DEBUG_RX_SFHSS_SPI, DEBUG_DATA_OFFSET_MAX, bindOffset_max);
            cc2500WriteReg(CC2500_0C_FSCTRL0, (uint8_t)bindOffset_max);
        }
        cc2500Strobe(CC2500_SRX);
    }
    return false;
}

static bool tune3Rx(uint8_t *packet) {
    cc2500LedBlink(100);
    if (((millis() - timeTunedMs) > 880) || bindOffset_min < (-126 + BIND_TUNE_STEP)) {   // 220ms *4
        return true;
    }
    if (sfhssRecv(packet)) {
        if (sfhssPacketParse(packet, true)) {
            timeTunedMs = millis();
            bindOffset_min -= BIND_TUNE_STEP;
            DEBUG_SET(DEBUG_RX_SFHSS_SPI, DEBUG_DATA_OFFSET_MIN, bindOffset_min);
            cc2500WriteReg(CC2500_0C_FSCTRL0, (uint8_t)bindOffset_min);
        }
        cc2500Strobe(CC2500_SRX);
    }
    return false;
}

void sfhssnextChannel(void) {
    do {
        sfhss_channel += sfhss_code + 2;
        if (sfhss_channel > 29) {
            sfhss_channel -= 31;
        }
    } while ( sfhss_channel < 0);
    sfhssRx();
}

void sfhssSpiSetRcData(uint16_t *rcData, const uint8_t *payload) {
    if ( GET_COMMAND(payload) & 0x8 ) {
        rcData[4] = GET_CH1(payload);
        rcData[5] = GET_CH2(payload);
        rcData[6] = GET_CH3(payload);
        rcData[7] = GET_CH4(payload);
    } else {
        rcData[0] = GET_CH1(payload);
        rcData[1] = GET_CH2(payload);
        rcData[2] = GET_CH3(payload);
        rcData[3] = GET_CH4(payload);
    }
}

rx_spi_received_e sfhssSpiDataReceived(uint8_t *packet) {
    static uint16_t dataMissingFrame = 0;
    static timeUs_t nextFrameReceiveStartTime = 0;
    static uint8_t frame_recvd = 0;
    timeUs_t currentPacketReceivedTime;
    rx_spi_received_e ret = RX_SPI_RECEIVED_NONE;
    currentPacketReceivedTime = micros();
    switch (protocolState) {
    case STATE_INIT:
        if ((millis() - start_time) > 10) {
            cc2500LedOff();
            dataMissingFrame = 0;
            initialise();
            SET_STATE(STATE_BIND);
            DEBUG_SET(DEBUG_RX_SFHSS_SPI, DEBUG_DATA_MISSING_FRAME, dataMissingFrame);
        }
        break;
    case STATE_BIND:
        if (cc2500checkBindRequested(true)) {
            cc2500LedOn();
            initTuneRx();
            SET_STATE(STATE_BIND_TUNING1);
        } else {
            SET_STATE(STATE_HUNT);
            sfhssnextChannel();
            setRssiDirect(0, RSSI_SOURCE_RX_PROTOCOL);
            nextFrameReceiveStartTime = currentPacketReceivedTime + NEXT_CH_TIME_HUNT;
        }
        break;
    case STATE_BIND_TUNING1:
        if (tune1Rx(packet)) {
            SET_STATE(STATE_BIND_TUNING2);
        }
        break;
    case STATE_BIND_TUNING2:
        if (tune2Rx(packet)) {
            SET_STATE(STATE_BIND_TUNING3);
        }
        break;
    case STATE_BIND_TUNING3:
        if (tune3Rx(packet)) {
            if (((int16_t)bindOffset_max - (int16_t)bindOffset_min) <= 2) {
                initTuneRx();
                SET_STATE(STATE_BIND_TUNING1);    // retry
            } else {
                rxFrSkySpiConfigMutable()->bindOffset = ((int16_t)bindOffset_max + (int16_t)bindOffset_min) / 2 ;
                SET_STATE(STATE_BIND_COMPLETE);
            }
        }
        break;
    case STATE_BIND_COMPLETE:
        writeEEPROM();
        ret = RX_SPI_RECEIVED_BIND;
        SET_STATE(STATE_INIT);
        break;
    case STATE_HUNT:
        if (sfhssRecv(packet)) {
            if (sfhssPacketParse(packet, true)) {
                if (GET_COMMAND(packet) & 0x8) {       /* ch=5-8 */
                    missingPackets = 0;
                    cc2500LedOn();
                    frame_recvd = 0x3;
                    SET_STATE(STATE_SYNC);
                    nextFrameReceiveStartTime = currentPacketReceivedTime + NEXT_CH_TIME_SYNC2;
                    return RX_SPI_RECEIVED_NONE;
                }
            }
            cc2500Strobe(CC2500_SRX);
        } else if (cmpTimeUs(currentPacketReceivedTime, nextFrameReceiveStartTime) > 0) {
            cc2500LedBlink(500);
#if defined(USE_RX_CC2500_SPI_PA_LNA) && defined(USE_RX_CC2500_SPI_DIVERSITY) // SE4311 chip
            cc2500switchAntennae();
#endif
            sfhssnextChannel();
            nextFrameReceiveStartTime += NEXT_CH_TIME_HUNT;
        } else if (cc2500checkBindRequested(false)) {
            SET_STATE(STATE_INIT);
            break;
        }
        break;
    case STATE_SYNC:
        if (sfhssRecv(packet)) {
            if (sfhssPacketParse(packet, true)) {
                missingPackets = 0;
                if ( GET_COMMAND(packet) & 0x8 ) {
                    nextFrameReceiveStartTime = currentPacketReceivedTime + NEXT_CH_TIME_SYNC2;
                    frame_recvd |= 0x2;     /* ch5-8 */
                } else {
                    nextFrameReceiveStartTime = currentPacketReceivedTime + NEXT_CH_TIME_SYNC1;
                    cc2500Strobe(CC2500_SRX);
                    frame_recvd |= 0x1;     /* ch1-4 */
                }
                if (GET_COMMAND(packet) & 0x4) {
                    return RX_SPI_RECEIVED_NONE; /* failsafe data */
                }
                return RX_SPI_RECEIVED_DATA;
            }
            cc2500Strobe(CC2500_SRX);
        } else if (cmpTimeUs(currentPacketReceivedTime, nextFrameReceiveStartTime) > 0) {
            nextFrameReceiveStartTime += NEXT_CH_TIME_SYNC0;
            if (frame_recvd != 0x3) {
                DEBUG_SET(DEBUG_RX_SFHSS_SPI, DEBUG_DATA_MISSING_FRAME, ++dataMissingFrame);
            }
            if (frame_recvd == 0) {
                if (++missingPackets > MAX_MISSING_PKT) {
                    SET_STATE(STATE_HUNT);
                    sfhssnextChannel();
                    setRssiDirect(0, RSSI_SOURCE_RX_PROTOCOL);
                    nextFrameReceiveStartTime = currentPacketReceivedTime + NEXT_CH_TIME_HUNT;
                    break;
                }
#if defined(USE_RX_CC2500_SPI_PA_LNA) && defined(USE_RX_CC2500_SPI_DIVERSITY) // SE4311 chip
                if (missingPackets >= 2) {
                    cc2500switchAntennae();
                }
#endif
            }
            frame_recvd = 0;
            sfhssnextChannel();
        } else if (cc2500checkBindRequested(false)) {
            SET_STATE(STATE_INIT);
            break;
        }
        break;
    }
    return ret;
}

bool sfhssSpiInit(const rxSpiConfig_t *rxSpiConfig, rxRuntimeConfig_t *rxRuntimeConfig) {
    UNUSED(rxSpiConfig);
    cc2500SpiInit();
    rxRuntimeConfig->channelCount = RC_CHANNEL_COUNT_SFHSS;
    start_time = millis();
    SET_STATE(STATE_INIT);
    return true;
}
#endif
