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

/* Created by jflyper */

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>

#include "platform.h"

#if defined(USE_VTX_BEESIGN) && defined(USE_VTX_CONTROL)

#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "cms/cms_menu_vtx_tramp.h"

#include "drivers/beesign.h"
#include "drivers/time.h"
#include "drivers/vtx_common.h"

#include "io/serial.h"
#include "io/vtx_tramp.h"
#include "io/vtx_control.h"
#include "io/vtx.h"
#include "io/vtx_string.h"
#include "io/vtx_beesign.h"

#define BEESIGN_CMD_BUFF_SIZE           (8192*2)

static uint8_t beesignCmdQueue[BEESIGN_CMD_BUFF_SIZE];
static uint8_t *beesignBuffPointer = beesignCmdQueue;
static uint8_t *beesignSendPointer = beesignCmdQueue;
static uint16_t beesignCmdCount;
static uint8_t receiveBuffer[BEESIGN_FM_MAX_LEN];
static uint8_t receiveFrame[BEESIGN_FM_MAX_LEN];
static uint8_t receiveFrameValid = 0;
static serialPort_t *beesignSerialPort = NULL;

#define MAX_CHARS2UPDATE        100
#define POLY                    (0xB2)
#define CALC_CRC(crc, data)     do {                                                        \
                                    crc ^= (data);                                          \
                                    crc = (crc & 1) ? ((crc >> 1) ^ POLY) : (crc >> 1);     \
                                    crc = (crc & 1) ? ((crc >> 1) ^ POLY) : (crc >> 1);     \
                                    crc = (crc & 1) ? ((crc >> 1) ^ POLY) : (crc >> 1);     \
                                    crc = (crc & 1) ? ((crc >> 1) ^ POLY) : (crc >> 1);     \
                                    crc = (crc & 1) ? ((crc >> 1) ^ POLY) : (crc >> 1);     \
                                    crc = (crc & 1) ? ((crc >> 1) ^ POLY) : (crc >> 1);     \
                                    crc = (crc & 1) ? ((crc >> 1) ^ POLY) : (crc >> 1);     \
                                    crc = (crc & 1) ? ((crc >> 1) ^ POLY) : (crc >> 1);     \
                                } while (0)

beeSignDevice_t bsDevice = {
    .version = 1,
    .channel = -1,
    .power = -1,
    .mode = 0,
    .freq = 0,
    .porfreq = 0,
};

typedef enum {
    BS_STATE_HDR = 0,
    BS_STATE_TYPE,
    BS_STATE_LEN,
    BS_STATE_PAYLOAD,
    BS_STATE_CRC
} bsState_e;

static uint8_t beesignCRC(const beesign_frame_t *pPackage) {
    uint8_t i;
    uint8_t crc = 0;
    CALC_CRC(crc, pPackage->hdr);
    CALC_CRC(crc, pPackage->type);
    CALC_CRC(crc, pPackage->len);
    for (i = 0; i < pPackage->len; i++) {
        CALC_CRC(crc, *(pPackage->payload + i));
    }
    return crc;
}

static uint8_t beesignChkID(uint8_t id) {
    UNUSED(id);
    return BEESIGN_OK;
}

uint8_t beesignReceive(uint8_t **pRcvFrame) {
    if (!receiveFrameValid) {
        (void)pRcvFrame;
        return BEESIGN_ERROR;
    }
    *pRcvFrame = receiveFrame;
    receiveFrameValid = 0;
    return BEESIGN_OK;
}

static uint8_t BSProcessResponse(uint8_t type, uint8_t *pRcvFrame) {
    if (beesignReceive(&pRcvFrame) != BEESIGN_ERROR) {
        if (BEESIGN_PKT_TYPE(pRcvFrame) == (type | BEESIGN_TYPE_MASK_FM)) {
            switch (BEESIGN_PKT_TYPE(pRcvFrame)) {
            case BEESIGN_V_GET_STATUS | BEESIGN_TYPE_MASK_FM:
                bsDevice.channel = BEESIGN_PKT_DATA(pRcvFrame, 0);
                bsDevice.freq = (uint16_t)BEESIGN_PKT_DATA(pRcvFrame, 2) + (BEESIGN_PKT_DATA(pRcvFrame, 1) << 8);
                bsDevice.power = BEESIGN_PKT_DATA(pRcvFrame, 3) + 1;
                bsDevice.mode = BEESIGN_PKT_DATA(pRcvFrame, 4);
                break;
            case BEESIGN_O_GET_STATUS | BEESIGN_TYPE_MASK_FM:
                break;
            case BEESIGN_A_GET_STATUS | BEESIGN_TYPE_MASK_FM:
                break;
            case BEESIGN_S_GET_STATUS | BEESIGN_TYPE_MASK_FM:
                break;
            default :
                break;
            }
            return 0;
        }
    }
    return 1;
}

void bsReceiveFrame(uint8_t ch) {
    static bsState_e state = BS_STATE_HDR;
    static uint8_t idx;
    static uint8_t len;
    switch (state) {
    case BS_STATE_HDR:
        if (ch == BEESIGN_HDR) {
            state = BS_STATE_TYPE;
            idx = 0;
            receiveBuffer[idx++] = ch;
        }
        break;
    case BS_STATE_TYPE:
        state = BS_STATE_LEN;
        receiveBuffer[idx++] = ch;
        break;
    case BS_STATE_LEN:
        if (ch <= BEESIGN_PL_MAX_LEN) {
            state = BS_STATE_PAYLOAD;
            len = ch;
            receiveBuffer[idx++] = ch;
        } else {
            state = BS_STATE_HDR;
        }
        break;
    case BS_STATE_PAYLOAD:
        receiveBuffer[idx++] = ch;
        if (--len == 0) {
            state = BS_STATE_CRC;
        }
        break;
    case BS_STATE_CRC:
        do {
            beesign_frame_t package = { .hdr = receiveBuffer[0],
                                        .type = receiveBuffer[1],
                                        .len = receiveBuffer[2],
                                        .payload = &receiveBuffer[3]
                                      };
            package.crc = beesignCRC(&package);
            if (package.crc == ch) {
                receiveFrameValid = 1;
                memcpy(receiveFrame, receiveBuffer, receiveBuffer[2] + 3);
                BSProcessResponse(package.type, receiveFrame);
            }
        } while (0);
        state = BS_STATE_HDR;
        break;
    default:
        state = BS_STATE_HDR;
        break;
    }
}

static uint8_t *beesignCmdAfterPointer(uint8_t *p, uint8_t afterLen) {
    if (p + afterLen >= (beesignCmdQueue + BEESIGN_CMD_BUFF_SIZE)) {
        return p + afterLen - BEESIGN_CMD_BUFF_SIZE;
    } else {
        return p + afterLen;
    }
}

static uint8_t beesignCmdGoNextPointer(uint8_t **p, uint8_t data) {
    uint8_t res = **p;
    **p = data;
    (*p)++;
    if (*p >= (beesignCmdQueue + BEESIGN_CMD_BUFF_SIZE)) {
        (*p) = beesignCmdQueue;
    }
    return res;
}

static uint8_t beesignAddCmd(uint8_t id, uint8_t len, uint8_t *pData) {
    beesign_frame_t package = { .hdr = BEESIGN_HDR,
                                .type = id,
                                .len = len,
                                .payload = pData
                              };
    if ((len >= BEESIGN_PL_MAX_LEN) ||
            (pData == 0)) {
        return BEESIGN_ERROR;
    }
    if ((beesignBuffPointer - beesignSendPointer > 0 && beesignSendPointer + BEESIGN_CMD_BUFF_SIZE - beesignBuffPointer < len) ||
            (beesignSendPointer - beesignBuffPointer > 0 && beesignSendPointer - beesignBuffPointer < len)) {                   // don't have enough buff
        return BEESIGN_ERROR;
    }
    package.crc = beesignCRC(&package);
    beesignCmdCount++;
    beesignCmdGoNextPointer(&beesignBuffPointer, package.hdr);
    beesignCmdGoNextPointer(&beesignBuffPointer, package.type);
    beesignCmdGoNextPointer(&beesignBuffPointer, package.len);
    for (uint8_t i = 0; i < len; i++) {
        beesignCmdGoNextPointer(&beesignBuffPointer, *(package.payload + i));
    }
    beesignCmdGoNextPointer(&beesignBuffPointer, package.crc);
    return BEESIGN_OK;
}

uint8_t beesignSendCmd(void) {
    uint8_t cmdLen = 0;
    if (beesignSerialPort) {
        if (beesignCmdCount > 0) {
            if(*beesignSendPointer == BEESIGN_HDR) {
                uint8_t crc = 0;
                uint8_t crcCheck = 0xff;
                cmdLen = *beesignCmdAfterPointer(beesignSendPointer, 2);
                for (uint8_t i = 0; i < cmdLen + 3; i++) {
                    CALC_CRC(crc, *beesignCmdAfterPointer(beesignSendPointer, i));
                    crcCheck = *beesignCmdAfterPointer(beesignSendPointer, i + 1);
                }
                if (crc == crcCheck) {
                    for (uint8_t i = 0; i < cmdLen + 4; i++) {
                        serialWrite(beesignSerialPort, beesignCmdGoNextPointer(&beesignSendPointer, 0));
                    }
                    beesignCmdCount--;
                }
            } else {
                beesignSendPointer++;
            }
        }
    }
    return cmdLen;
}

static uint8_t beesignSend(uint8_t id, uint8_t len, uint8_t *pData, uint8_t cmd) {
    if (cmd == BEESIGN_CMD_ADD_BUFF) {
        return beesignAddCmd(id, len, pData);
    } else {
        beesign_frame_t package = { .hdr = BEESIGN_HDR,
                                    .type = id,
                                    .len = len,
                                    .payload = pData
                                  };
        if ((len >= BEESIGN_PL_MAX_LEN) ||
                (pData == 0)) {
            return BEESIGN_ERROR;
        }
        package.crc = beesignCRC(&package);
        serialWrite(beesignSerialPort, package.hdr);
        serialWrite(beesignSerialPort, package.type);
        serialWrite(beesignSerialPort, package.len);
        for (uint8_t i = 0; i < len; i++) {
            serialWrite(beesignSerialPort, *(package.payload + i));
        }
        serialWrite(beesignSerialPort, package.crc);
        return BEESIGN_OK;
    }
}

// uint8_t beesignSend(uint8_t id, uint8_t len, uint8_t *pData) {
//     beesign_frame_t package = { .hdr = BEESIGN_HDR,
//                                 .type = id,
//                                 .len = len,
//                                 .payload = pData };

//     if ((len >= BEESIGN_PL_MAX_LEN) ||
//         (pData == 0) ||
//         (beesignChkID(id) != BEESIGN_OK)) {
//         return BEESIGN_ERROR;
//     }

//     package.crc = beesignCRC(&package);

//     serialWrite(beesignSerialPort, package.hdr);
//     serialWrite(beesignSerialPort, package.type);
//     serialWrite(beesignSerialPort, package.len);
//     for (uint8_t i = 0; i < len; i++) {
//         serialWrite(beesignSerialPort, *(package.payload + i));
//     }
//     serialWrite(beesignSerialPort, package.crc);
//     delayMicroseconds(50000);
//     return BEESIGN_OK;
// }

/********************************** BEESIGN VTX ********************************************/
void bsSetVTxUnlock(void) {
    uint8_t vtxSaveData = 0;
    uint16_t unlock = BEESIGN_VTX_UNLOCK;
    uint8_t unlockData[2] = {unlock >> 8, unlock};
    beesignSend(BEESIGN_V_UNLOCK, 2, unlockData, BEESIGN_CMD_ADD_BUFF);
    beesignSend(BEESIGN_M_SAVE_SETTING, 1, &vtxSaveData, BEESIGN_CMD_ADD_BUFF);
}

void bsSetVTxLock(void) {
    uint8_t vtxSaveData = 0;
    uint16_t lock = BEESIGN_VTX_LOCK;
    uint8_t lockData[2] = {lock >> 8, lock};
    beesignSend(BEESIGN_V_LOCK, 2, lockData, BEESIGN_CMD_ADD_BUFF);
    beesignSend(BEESIGN_M_SAVE_SETTING, 1, &vtxSaveData, BEESIGN_CMD_ADD_BUFF);
}

bool bsValidateBandAndChannel(uint8_t band, uint8_t channel) {
    return (band >= BEESIGN_MIN_BAND && band <= BEESIGN_MAX_BAND &&
            channel >= BEESIGN_MIN_CHANNEL && channel <= BEESIGN_MAX_CHANNEL);
}

void bsSetBandAndChannel(uint8_t band, uint8_t channel) {
    uint8_t vtxSaveData = 0;
    uint8_t deviceChannel = BS_BANDCHAN_TO_DEVICE_CHVAL(band, channel);
    // bsDevice.channel = deviceChannel;
    // bsDevice.freq = beesignTable[band][channel];
    beesignSend(BEESIGN_V_SET_CHAN, 1, &deviceChannel, BEESIGN_CMD_ADD_BUFF);
#ifndef USE_OSD_BEESIGN
    beesignSend(BEESIGN_M_SAVE_SETTING, 1, &vtxSaveData, BEESIGN_CMD_ADD_BUFF);
#endif
    bsGetVtxState();
}

void bsSetPower(uint8_t index) {
    uint8_t vtxSaveData = 0;
    if (index > BEESIGN_POWER_COUNT) {
        return;
    }
    // bsDevice.power = index;
    index -= 1;
    beesignSend(BEESIGN_V_SET_PWR, 1, &index, BEESIGN_CMD_ADD_BUFF);
#ifndef USE_OSD_BEESIGN
    beesignSend(BEESIGN_M_SAVE_SETTING, 1, &vtxSaveData, BEESIGN_CMD_ADD_BUFF);
#endif
    bsGetVtxState();
}

void bsSetVtxMode(uint8_t mode) {
    uint8_t vtxSaveData = 0;
    if (mode > 2) return;
    // bsDevice.mode = mode;
    beesignSend(BEESIGN_V_SET_MODE, 1, &mode, BEESIGN_CMD_ADD_BUFF);
#ifndef USE_OSD_BEESIGN
    beesignSend(BEESIGN_M_SAVE_SETTING, 1, &vtxSaveData, BEESIGN_CMD_ADD_BUFF);
#endif
    bsGetVtxState();
}

bool bsValidateFreq(uint16_t freq) {
    return (freq >= BEESIGN_MIN_FREQUENCY_MHZ && freq <= BEESIGN_MAX_FREQUENCY_MHZ);
}

void bsSetFreq(uint16_t freq) {
    uint8_t buf[2];
    uint8_t vtxSaveData = 0;
    buf[0] = (freq >> 8) & 0xff;
    buf[1] = freq & 0xff;
    // bsDevice.freq = freq;
    // bsDevice.channel = BEESIGN_ERROR_CHANNEL;
    beesignSend(BEESIGN_V_SET_FREQ, 2, buf, BEESIGN_CMD_ADD_BUFF);
#ifndef USE_OSD_BEESIGN
    beesignSend(BEESIGN_M_SAVE_SETTING, 1, &vtxSaveData, BEESIGN_CMD_ADD_BUFF);
#endif
    bsGetVtxState();
}

void bsGetVtxState(void) {
    uint8_t buf = '0';
    beesignSend(BEESIGN_V_GET_STATUS, 1, &buf, BEESIGN_CMD_SEND);
}

void bsCloseOsd(void) {
    uint8_t mode = BEESIGN_OSD_MODE_OFF;
    beesignSend(BEESIGN_O_SET_MODE, 1, &mode, BEESIGN_CMD_ADD_BUFF);
}

/******************************** BEESIGN VTX END ******************************************/

/********************************** BEESIGN OSD ********************************************/
#if defined(USE_OSD_BEESIGN)

static uint8_t bsScreenBuffer[BEESIGN_CHARS_PER_SCREEN];
static uint8_t bsShadowBuffer[BEESIGN_CHARS_PER_SCREEN];

void bsSetOsdMode(uint8_t mode) {
    if (mode > BEESIGN_OSD_MODE_CUSTOM) {
        mode = BEESIGN_OSD_MODE_OFF;
    }
    beesignSend(BEESIGN_O_SET_MODE, 1, &mode, BEESIGN_CMD_ADD_BUFF);
}

void bsSetOsdHosOffset(uint8_t offset) {
    offset += 4;
    if (offset > BEESIGN_OSD_HOS_MAX) {
        offset = BEESIGN_OSD_HOS_MAX;
    }
    beesignSend(BEESIGN_O_SET_HOS, 1, &offset, BEESIGN_CMD_ADD_BUFF);
}

void bsSetOsdVosOffset(uint8_t offset) {
    offset += 20;
    if (offset > BEESIGN_OSD_VOS_MAX) {
        offset = BEESIGN_OSD_VOS_MAX;
    }
    beesignSend(BEESIGN_O_SET_VOS, 1, &offset, BEESIGN_CMD_ADD_BUFF);
}

void bsClearDisplay(void) {
    uint8_t clrData = 0;
    beesignSend(BEESIGN_O_CLR_DISPLAY, 1, &clrData, BEESIGN_CMD_ADD_BUFF);
}

void bsSetDisplayContentOneFrame(uint8_t pos, uint8_t *data, uint8_t len) {
    uint8_t s[BEESIGN_PL_MAX_LEN];
    if (pos >= BEESIGN_CHARS_PER_SCREEN) {
        return;
    }
    if (len > BEESIGN_PL_MAX_LEN - 1) {
        len = BEESIGN_PL_MAX_LEN - 1;
    }
    s[0] = pos;
    for (int i = 0; i < len; i++) {
        s[i + 1] = data[i];
    }
    beesignSend(BEESIGN_O_SET_DISPLAY, len + 1, s, BEESIGN_CMD_ADD_BUFF);
}

void bsSetDisplayOneChar(uint8_t x, uint8_t y, uint8_t data) {
    if (y >= BEESIGN_LINES_PER_SCREEN) {
        return;
    }
    if (x >= BEESIGN_CHARS_PER_LINE) {
        return;
    }
    bsSetDisplayContentOneFrame(y * BEESIGN_CHARS_PER_LINE + x, &data, 1);
}

void bsSetDisplayInOneRow(uint8_t x, uint8_t y, uint8_t *data) {
    int i = 0;
    if (y >= BEESIGN_LINES_PER_SCREEN) {
        return;
    }
    for (i = 0; * (data + i); i++) {
        if (x + i > BEESIGN_CHARS_PER_LINE) {
            break;
        }
    }
    bsSetDisplayContentOneFrame(y * BEESIGN_CHARS_PER_LINE + x, data, i);
}

void bsClearScreenBuff(void) {
    memset(bsScreenBuffer, 0x20, BEESIGN_CHARS_PER_SCREEN);
}

void bsCleanScreen(void) {
    memset(bsScreenBuffer, 0x20, BEESIGN_CHARS_PER_SCREEN);
    memset(bsShadowBuffer, 0x20, BEESIGN_CHARS_PER_SCREEN);
    // beesignCmdCount = 0;
    bsClearDisplay();
}

void bsWriteBuffChar(uint8_t x, uint8_t y, uint8_t c) {
    if (y >= BEESIGN_LINES_PER_SCREEN) {
        return;
    }
    if (x >= BEESIGN_CHARS_PER_LINE) {
        return;
    }
    bsScreenBuffer[y * BEESIGN_CHARS_PER_LINE + x] = c;
}

void bsWriteBuffRow(uint8_t x, uint8_t y, const char *buff) {
    if (y >= BEESIGN_LINES_PER_SCREEN) {
        return;
    }
    if (x >= BEESIGN_CHARS_PER_LINE) {
        return;
    }
    for (int i = 0; * (buff + i); i++) {
        if (x + i < BEESIGN_CHARS_PER_LINE) { // Do not write over screen
            bsScreenBuffer[y * BEESIGN_CHARS_PER_LINE + x + i] = *(buff + i);
        }
    }
}

void bsDisplay(void) {
    uint8_t buffStartPos = 0xFF;
    uint8_t buffEndPos = 0xFF;
    uint8_t seriaBuff[BEESIGN_CHARS_PER_LINE + 1];
    for (int i = 0; i < BEESIGN_CHARS_PER_SCREEN; i++) {
        if (bsScreenBuffer[i] != bsShadowBuffer[i]) {
            if (buffStartPos == 0xFF) {
                buffStartPos = i;
            }
            seriaBuff[i - buffStartPos + 1] = bsScreenBuffer[i];
            if (i - buffStartPos + 1 >= BEESIGN_CHARS_PER_LINE) {
                seriaBuff[0] = buffStartPos;
                beesignSend(BEESIGN_O_SET_DISPLAY, i - buffStartPos + 2, seriaBuff, BEESIGN_CMD_ADD_BUFF);
                buffStartPos = 0xFF;
            }
            buffEndPos = i;
            bsShadowBuffer[i] = bsScreenBuffer[i];
        } else {
            if ((buffStartPos != 0xFF)) {
                if (i - buffStartPos + 1 < 10) {
                    seriaBuff[i - buffStartPos + 1] = bsScreenBuffer[i];
                } else {
                    seriaBuff[0] = buffStartPos;
                    beesignSend(BEESIGN_O_SET_DISPLAY, buffEndPos - buffStartPos + 2, seriaBuff, BEESIGN_CMD_ADD_BUFF);
                    buffStartPos = 0xFF;
                }
            }
        }
    }
    if (buffStartPos != 0xFF) {
        seriaBuff[0] = buffStartPos;
        beesignSend(BEESIGN_O_SET_DISPLAY, buffEndPos - buffStartPos + 2, seriaBuff, BEESIGN_CMD_ADD_BUFF);
        buffStartPos = 0xFF;
    }
}

void bsDisplayAllScreen(void) {
    for (int i = 0; i < BEESIGN_LINES_PER_SCREEN; i++) {
        bsSetDisplayContentOneFrame(i * BEESIGN_CHARS_PER_LINE, &bsScreenBuffer[BEESIGN_CHARS_PER_LINE * i], BEESIGN_CHARS_PER_LINE);
    }
    memcpy(bsShadowBuffer, bsScreenBuffer, BEESIGN_CHARS_PER_SCREEN);
}

bool bsBuffersSynced(void) {
    for (int i = 0; i < BEESIGN_CHARS_PER_SCREEN; i++) {
        if (bsScreenBuffer[i] != bsShadowBuffer[i]) {
            return false;
        }
    }
    return true;
}

void bsUpdateCharacterFont(uint8_t id, uint8_t *data) {
    uint8_t sendData[BEESIGN_CHARS_FONT_LEN + 1];
    uint16_t unlockData = BEESIGN_CHARS_UNLOCK;
    if (id > BEESIGN_CHARS_MAX) {
        return;
    }
    if (id < BEESIGN_CHARS_UNLOCK_ADDR_MIN) {
        uint8_t buf[2] = {unlockData >> 8, unlockData & 0x00FF};
        beesignSend(BEESIGN_O_FONT_UNLOCK, 2, buf, BEESIGN_CMD_ADD_BUFF);
    }
    *sendData = id;
    memcpy(&sendData[1], data, BEESIGN_CHARS_FONT_LEN);
    beesignSend(BEESIGN_O_UDT_FONT, BEESIGN_CHARS_FONT_LEN + 1, sendData, BEESIGN_CMD_ADD_BUFF);
}


#endif //defined(USE_OSD_BEESIGN)
/******************************** BEESIGN OSD END ******************************************/

bool beesignInit(void) {
#if defined(USE_BEESIGN_UART)
    beesignSerialPort = openSerialPort(USE_BEESIGN_UART, FUNCTION_VTX_BEESIGN, NULL, NULL, 115200, MODE_RXTX, SERIAL_BIDIR | SERIAL_BIDIR_PP | SERIAL_BIDIR_NOPULL);
#else
    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_VTX_BEESIGN);
    if (portConfig) {
        beesignSerialPort = openSerialPort(portConfig->identifier, FUNCTION_VTX_BEESIGN, NULL, NULL, 115200, MODE_RXTX, SERIAL_BIDIR | SERIAL_BIDIR_PP | SERIAL_BIDIR_NOPULL);
    }
#endif
    if(!beesignSerialPort) {
        return false;
    }
#ifndef USE_OSD_BEESIGN
    bsCloseOsd();
#endif
    return true;
}

bool checkBeesignSerialPort(void) {
    if (!beesignSerialPort) {
        return false;
    }
    return true;
}

void beesignUpdate(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs);
    if (checkBeesignSerialPort) {
#ifdef USE_OSD_BEESIGN
        static uint32_t beesignTaskCounter = 0;
        static uint32_t beesignSendNextCounterPoint = 0;
        static uint32_t beesignSendLineNextCounterPoint = 0;
        static uint8_t sendScreenLine = 0;
        // static uint32_t beesignClearShadowBufferPoint = 60;
        if (beesignTaskCounter >= beesignSendNextCounterPoint) {
            beesignSendNextCounterPoint += beesignSendCmd() / 8 + 2;       // send command and get next send time
        }
#else
        while (serialRxBytesWaiting(beesignSerialPort) > 0) {
            const uint8_t ch = serialRead(beesignSerialPort);
            bsReceiveFrame(ch);
        }
        beesignSendCmd();
#endif
        // send every line to prevent data loss
#ifdef USE_OSD_BEESIGN
        if (beesignTaskCounter >= beesignSendLineNextCounterPoint) {
            beesignSendLineNextCounterPoint += 15;       // send command and get next send time
            bsSetDisplayContentOneFrame(sendScreenLine * BEESIGN_CHARS_PER_LINE, &bsScreenBuffer[BEESIGN_CHARS_PER_LINE * sendScreenLine], BEESIGN_CHARS_PER_LINE);
            sendScreenLine++;
            if (sendScreenLine >= BEESIGN_LINES_PER_SCREEN) sendScreenLine = 0;
        }
        beesignTaskCounter++;
#endif
    }
}

#endif // USE_VTX_BEESIGN