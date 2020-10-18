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
#include <stdint.h>
#include <string.h>

#include "common/crc.h"
#include "common/maths.h"
#include "common/streambuf.h"

#include "drivers/time.h"

#include "io/serial.h"

#include "pg/rcdevice.h"

#include "rcdevice.h"

#ifdef USE_RCDEVICE


typedef struct runcamDeviceExpectedResponseLength_s {
    uint8_t command;
    uint8_t reponseLength;
} runcamDeviceExpectedResponseLength_t;

static runcamDeviceExpectedResponseLength_t expectedResponsesLength[] = {
    { RCDEVICE_PROTOCOL_COMMAND_GET_DEVICE_INFO,            5},
    { RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_PRESS,      2},
    { RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_RELEASE,    2},
    { RCDEVICE_PROTOCOL_COMMAND_5KEY_CONNECTION,            3},
};

rcdeviceWaitingResponseQueue watingResponseQueue;
static uint8_t recvBuf[RCDEVICE_PROTOCOL_MAX_PACKET_SIZE]; // all the response contexts using same recv buffer

static uint8_t runcamDeviceGetRespLen(uint8_t command) {
    for (unsigned int i = 0; i < ARRAYLEN(expectedResponsesLength); i++) {
        if (expectedResponsesLength[i].command == command) {
            return expectedResponsesLength[i].reponseLength;
        }
    }
    return 0;
}

static bool rcdeviceRespCtxQueuePush(rcdeviceWaitingResponseQueue *queue, rcdeviceResponseParseContext_t *respCtx) {
    if (queue == NULL || (queue->itemCount + 1) > MAX_WAITING_RESPONSES) {
        return false;
    }
    queue->buffer[queue->tailPos] = *respCtx;
    int newTailPos = queue->tailPos + 1;
    if (newTailPos >= MAX_WAITING_RESPONSES) {
        newTailPos = 0;
    }
    queue->itemCount += 1;
    queue->tailPos = newTailPos;
    return true;
}

static rcdeviceResponseParseContext_t* rcdeviceRespCtxQueuePeekFront(rcdeviceWaitingResponseQueue *queue) {
    if (queue == NULL || queue->itemCount == 0) {
        return NULL;
    }
    rcdeviceResponseParseContext_t *ctx = &queue->buffer[queue->headPos];
    return ctx;
}

STATIC_UNIT_TESTED rcdeviceResponseParseContext_t* rcdeviceRespCtxQueueShift(rcdeviceWaitingResponseQueue *queue) {
    if (queue == NULL || queue->itemCount == 0) {
        return NULL;
    }
    rcdeviceResponseParseContext_t *ctx = &queue->buffer[queue->headPos];
    int newHeadPos = queue->headPos + 1;
    if (newHeadPos >= MAX_WAITING_RESPONSES) {
        newHeadPos = 0;
    }
    queue->itemCount -= 1;
    queue->headPos = newHeadPos;
    return ctx;
}

// every time send packet to device, and want to get something from device,
// it'd better call the method to clear the rx buffer before the packet send,
// else may be the useless data in rx buffer will cause the response decoding
// failed.
static void runcamDeviceFlushRxBuffer(runcamDevice_t *device) {
    while (serialRxBytesWaiting(device->serialPort) > 0) {
        serialRead(device->serialPort);
    }
}

// a common way to send packet to device
static void runcamDeviceSendPacket(runcamDevice_t *device, uint8_t command, uint8_t *paramData, int paramDataLen) {
    // is this device open?
    if (!device->serialPort) {
        return;
    }
    sbuf_t buf;
    // prepare pointer
    buf.ptr = device->buffer;
    buf.end = ARRAYEND(device->buffer);
    sbufWriteU8(&buf, RCDEVICE_PROTOCOL_HEADER);
    sbufWriteU8(&buf, command);
    if (paramData) {
        sbufWriteData(&buf, paramData, paramDataLen);
    }
    // add crc over (all) data
    crc8_dvb_s2_sbuf_append(&buf, device->buffer);
    // switch to reader
    sbufSwitchToReader(&buf, device->buffer);
    // send data if possible
    serialWriteBuf(device->serialPort, sbufPtr(&buf), sbufBytesRemaining(&buf));
}

// a common way to send a packet to device, and get response from the device.
static void runcamDeviceSendRequestAndWaitingResp(runcamDevice_t *device, uint8_t commandID, uint8_t *paramData, uint8_t paramDataLen, timeMs_t tiemout, int maxRetryTimes, void *userInfo, rcdeviceRespParseFunc parseFunc) {
    runcamDeviceFlushRxBuffer(device);
    rcdeviceResponseParseContext_t responseCtx;
    memset(&responseCtx, 0, sizeof(rcdeviceResponseParseContext_t));
    responseCtx.recvBuf = recvBuf;
    responseCtx.command = commandID;
    responseCtx.maxRetryTimes = maxRetryTimes;
    responseCtx.expectedRespLen = runcamDeviceGetRespLen(commandID);
    responseCtx.timeout = tiemout;
    responseCtx.timeoutTimestamp = millis() + tiemout;
    responseCtx.parserFunc = parseFunc;
    responseCtx.device = device;
    responseCtx.protocolVer = RCDEVICE_PROTOCOL_VERSION_1_0;
    if (paramData != NULL) {
        memcpy(responseCtx.paramData, paramData, paramDataLen);
        responseCtx.paramDataLen = paramDataLen;
    }
    responseCtx.userInfo = userInfo;
    rcdeviceRespCtxQueuePush(&watingResponseQueue, &responseCtx);
    // send packet
    runcamDeviceSendPacket(device, commandID, paramData, paramDataLen);
}

static void runcamDeviceParseV1DeviceInfo(rcdeviceResponseParseContext_t *ctx) {
    if (ctx->result != RCDEVICE_RESP_SUCCESS) {
        return;
    }
    runcamDevice_t *device = ctx->device;
    device->info.protocolVer = RCDEVICE_PROTOCOL_RCSPLIT_VERSION;
    device->info.features = RCDEVICE_PROTOCOL_FEATURE_SIMULATE_POWER_BUTTON | RCDEVICE_PROTOCOL_FEATURE_SIMULATE_WIFI_BUTTON | RCDEVICE_PROTOCOL_FEATURE_CHANGE_MODE;
    device->isReady = true;
}

static uint8_t crc8HighFirst(uint8_t *ptr, uint8_t len) {
    uint8_t crc = 0x00;
    while (len--) {
        crc ^= *ptr++;
        for (unsigned i = 8; i > 0; --i) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc = (crc << 1);
        }
    }
    return (crc);
}

// for the rcsplits that firmware <= 1.1.0
static void runcamSplitSendCommand(runcamDevice_t *device, uint8_t argument) {
    if (!device->serialPort) {
        return;
    }
    uint8_t uart_buffer[5] = {0};
    uint8_t crc = 0;
    uart_buffer[0] = RCSPLIT_PACKET_HEADER;
    uart_buffer[1] = RCSPLIT_PACKET_CMD_CTRL;
    uart_buffer[2] = argument;
    uart_buffer[3] = RCSPLIT_PACKET_TAIL;
    crc = crc8HighFirst(uart_buffer, 4);
    // build up a full request [header]+[command]+[argument]+[crc]+[tail]
    uart_buffer[3] = crc;
    uart_buffer[4] = RCSPLIT_PACKET_TAIL;
    // write to device
    serialWriteBuf(device->serialPort, uart_buffer, 5);
}

static void runcamDeviceParseV2DeviceInfo(rcdeviceResponseParseContext_t *ctx) {
    if (ctx->result != RCDEVICE_RESP_SUCCESS) {
        runcamDeviceFlushRxBuffer(ctx->device);
        rcdeviceResponseParseContext_t responseCtx;
        memset(&responseCtx, 0, sizeof(rcdeviceResponseParseContext_t));
        responseCtx.recvBuf = recvBuf;
        responseCtx.command = 0xFF;
        responseCtx.maxRetryTimes = rcdeviceConfig()->initDeviceAttempts;
        responseCtx.expectedRespLen = 5;
        responseCtx.timeout = rcdeviceConfig()->initDeviceAttemptInterval;
        responseCtx.timeoutTimestamp = millis() + rcdeviceConfig()->initDeviceAttemptInterval;
        responseCtx.parserFunc = runcamDeviceParseV1DeviceInfo;
        responseCtx.device = ctx->device;
        responseCtx.protocolVer = RCDEVICE_PROTOCOL_RCSPLIT_VERSION;
        rcdeviceRespCtxQueuePush(&watingResponseQueue, &responseCtx);
        runcamSplitSendCommand(ctx->device, 0xFF);
        return;
    }
    runcamDevice_t *device = ctx->device;
    device->info.protocolVer = ctx->recvBuf[1];
    uint8_t featureLowBits = ctx->recvBuf[2];
    uint8_t featureHighBits = ctx->recvBuf[3];
    device->info.features = (featureHighBits << 8) | featureLowBits;
    device->isReady = true;
}

// get the device info(firmware version, protocol version and features, see the
// definition of runcamDeviceInfo_t to know more)
static void runcamDeviceGetDeviceInfo(runcamDevice_t *device) {
    runcamDeviceSendRequestAndWaitingResp(device, RCDEVICE_PROTOCOL_COMMAND_GET_DEVICE_INFO, NULL, 0, rcdeviceConfig()->initDeviceAttemptInterval, rcdeviceConfig()->initDeviceAttempts, NULL, runcamDeviceParseV2DeviceInfo);
}

// init the runcam device, it'll search the UART port with FUNCTION_RCDEVICE id
// this function will delay 400ms in the first loop to wait the device prepared,
// as we know, there are has some camera need about 200~400ms to initialization,
// and then we can send/receive from it.
void runcamDeviceInit(runcamDevice_t *device) {
    device->isReady = false;
    serialPortFunction_e portID = FUNCTION_RCDEVICE;
    serialPortConfig_t *portConfig = findSerialPortConfig(portID);
    if (portConfig != NULL) {
        device->serialPort = openSerialPort(portConfig->identifier, portID, NULL, NULL, 115200, MODE_RXTX, SERIAL_NOT_INVERTED);
        if (device->serialPort != NULL) {
            // send RCDEVICE_PROTOCOL_COMMAND_GET_DEVICE_INFO to device to retrive
            // device info, e.g protocol version, supported features
            runcamDeviceGetDeviceInfo(device);
        }
    }
}


bool runcamDeviceSimulateCameraButton(runcamDevice_t *device, uint8_t operation) {
    if (device->info.protocolVer == RCDEVICE_PROTOCOL_VERSION_1_0) {
        runcamDeviceSendPacket(device, RCDEVICE_PROTOCOL_COMMAND_CAMERA_CONTROL, &operation, sizeof(operation));
    } else if (device->info.protocolVer == RCDEVICE_PROTOCOL_RCSPLIT_VERSION) {
        runcamSplitSendCommand(device, operation + 1);
    } else {
        return false;
    }
    return true;
}

// every time start to control the OSD menu of camera, must call this method to
// camera
void runcamDeviceOpen5KeyOSDCableConnection(runcamDevice_t *device, rcdeviceRespParseFunc parseFunc) {
    uint8_t operation = RCDEVICE_PROTOCOL_5KEY_CONNECTION_OPEN;
    runcamDeviceSendRequestAndWaitingResp(device, RCDEVICE_PROTOCOL_COMMAND_5KEY_CONNECTION, &operation, sizeof(uint8_t), 400, 2, NULL, parseFunc);
}

// when the control was stop, must call this method to the camera to disconnect
// with camera.
void runcamDeviceClose5KeyOSDCableConnection(runcamDevice_t *device, rcdeviceRespParseFunc parseFunc) {
    uint8_t operation = RCDEVICE_PROTOCOL_5KEY_CONNECTION_CLOSE;
    runcamDeviceSendRequestAndWaitingResp(device, RCDEVICE_PROTOCOL_COMMAND_5KEY_CONNECTION, &operation, sizeof(uint8_t), 400, 2, NULL, parseFunc);
}

// simulate button press event of 5 key osd cable with special button
void runcamDeviceSimulate5KeyOSDCableButtonPress(runcamDevice_t *device, uint8_t operation, rcdeviceRespParseFunc parseFunc) {
    if (operation == RCDEVICE_PROTOCOL_5KEY_SIMULATION_NONE) {
        return;
    }
    runcamDeviceSendRequestAndWaitingResp(device, RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_PRESS, &operation, sizeof(uint8_t), 400, 2, NULL, parseFunc);
}

// simulate button release event of 5 key osd cable
void runcamDeviceSimulate5KeyOSDCableButtonRelease(runcamDevice_t *device, rcdeviceRespParseFunc parseFunc) {
    runcamDeviceSendRequestAndWaitingResp(device, RCDEVICE_PROTOCOL_COMMAND_5KEY_SIMULATION_RELEASE, NULL, 0, 400, 2, NULL, parseFunc);
}

static rcdeviceResponseParseContext_t* getWaitingResponse(timeMs_t currentTimeMs) {
    rcdeviceResponseParseContext_t *respCtx = rcdeviceRespCtxQueuePeekFront(&watingResponseQueue);
    while (respCtx != NULL && respCtx->timeoutTimestamp != 0 && currentTimeMs > respCtx->timeoutTimestamp) {
        if (respCtx->maxRetryTimes > 0) {
            if (respCtx->protocolVer == RCDEVICE_PROTOCOL_VERSION_1_0) {
                runcamDeviceSendPacket(respCtx->device, respCtx->command, respCtx->paramData, respCtx->paramDataLen);
            } else if (respCtx->protocolVer == RCDEVICE_PROTOCOL_RCSPLIT_VERSION) {
                runcamSplitSendCommand(respCtx->device, respCtx->command);
            }
            respCtx->recvRespLen = 0;
            respCtx->timeoutTimestamp = currentTimeMs + respCtx->timeout;
            respCtx->maxRetryTimes -= 1;
            respCtx = NULL;
            break;
        } else {
            respCtx->result = RCDEVICE_RESP_TIMEOUT;
            if (respCtx->parserFunc != NULL) {
                respCtx->parserFunc(respCtx);
            }
            // dequeue and get next waiting response context
            rcdeviceRespCtxQueueShift(&watingResponseQueue);
            respCtx = rcdeviceRespCtxQueuePeekFront(&watingResponseQueue);
        }
    }
    return respCtx;
}

void rcdeviceReceive(timeUs_t currentTimeUs) {
    UNUSED(currentTimeUs);
    rcdeviceResponseParseContext_t *respCtx = NULL;
    while ((respCtx = getWaitingResponse(millis())) != NULL && serialRxBytesWaiting(respCtx->device->serialPort)) {
        const uint8_t c = serialRead(respCtx->device->serialPort);
        if (respCtx->recvRespLen == 0) {
            // Only start receiving packet when we found a header
            if ((respCtx->protocolVer == RCDEVICE_PROTOCOL_VERSION_1_0 && c != RCDEVICE_PROTOCOL_HEADER) || (respCtx->protocolVer == RCDEVICE_PROTOCOL_RCSPLIT_VERSION && c != RCSPLIT_PACKET_HEADER)) {
                continue;
            }
        }
        respCtx->recvBuf[respCtx->recvRespLen] = c;
        respCtx->recvRespLen += 1;
        // if data received done, trigger callback to parse response data, and update rcdevice state
        if (respCtx->recvRespLen == respCtx->expectedRespLen) {
            // verify the crc value
            if (respCtx->protocolVer == RCDEVICE_PROTOCOL_VERSION_1_0) {
                uint8_t crc = 0;
                for (int i = 0; i < respCtx->recvRespLen; i++) {
                    crc = crc8_dvb_s2(crc, respCtx->recvBuf[i]);
                }
                respCtx->result = (crc == 0) ? RCDEVICE_RESP_SUCCESS : RCDEVICE_RESP_INCORRECT_CRC;
            } else if (respCtx->protocolVer == RCDEVICE_PROTOCOL_RCSPLIT_VERSION) {
                // do nothing, just call parserFunc
                respCtx->result = RCDEVICE_RESP_SUCCESS;
            }
            if (respCtx->parserFunc != NULL) {
                respCtx->parserFunc(respCtx);
            }
            // dequeue current response context
            rcdeviceRespCtxQueueShift(&watingResponseQueue);
        }
    }
}

#endif
