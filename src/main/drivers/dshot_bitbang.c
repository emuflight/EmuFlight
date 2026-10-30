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

#include "platform.h"

#ifdef USE_DSHOT_BITBANG

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/dshot_bitbang.h"
#include "drivers/dshot_bitbang_impl.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/pwm_output.h"
#include "drivers/time.h"
#include "drivers/timer.h"

// A stream disable takes a few AHB cycles to show in EN
#define BB_DMA_STOP_SPINS 100

#define BB_PORT_BUF_WORDS MOTOR_DSHOT_BUF_LENGTH

#define BB_CH(n) CC_CHANNEL_FROM_INDEX((n) - 1)

typedef struct bbPacerRow_s {
    TIM_TypeDef *tim;
    uint8_t channel;
    int8_t dmaopt;
} bbPacerRow_t;

// A timer channel only paces the DMA; it needs no pin. dmaopt picks the DMA stream, -1 tries all.
static const bbPacerRow_t bbPacerRows[] = {
#if defined(STM32F4) || defined(STM32F7)
#if !defined(STM32F411xE)
    { TIM8, BB_CH(1), 1 },
    { TIM8, BB_CH(2), 1 },
    { TIM8, BB_CH(3), 1 },
    { TIM8, BB_CH(4), 0 },
#endif
    { TIM1, BB_CH(1), 1 },
    { TIM1, BB_CH(1), 2 },
    { TIM1, BB_CH(2), 1 },
    { TIM1, BB_CH(3), 1 },
    { TIM1, BB_CH(4), 0 },
#elif defined(STM32H7)
    { TIM8, BB_CH(1), -1 },
    { TIM8, BB_CH(2), -1 },
    { TIM8, BB_CH(3), -1 },
    { TIM8, BB_CH(4), -1 },
    { TIM1, BB_CH(1), -1 },
    { TIM1, BB_CH(2), -1 },
    { TIM1, BB_CH(3), -1 },
    { TIM1, BB_CH(4), -1 },
#else
#error MCU dependent code required
#endif
};

static bbPacer_t bbPacers[MAX_MOTOR_PACERS];
static int usedMotorPacers;
static bbPort_t bbPorts[MAX_SUPPORTED_MOTOR_PORTS];
static int usedMotorPorts;
static bbMotor_t bbMotors[MAX_SUPPORTED_MOTORS];
static DMA_DATA_ZERO_INIT uint32_t bbOutputBuffer[BB_PORT_BUF_WORDS * MAX_SUPPORTED_MOTOR_PORTS];
static dshotBitbangStatus_e bbStatus;
static bool bbRunning;

dshotBitbangStatus_e dshotBitbangGetStatus(void)
{
    return bbStatus;
}

bool dshotBitbangIsRunning(void)
{
    return bbRunning;
}

bool dshotBitbangSupported(uint8_t motorPwmInversion)
{
    // the bitbang frame is never inverted: keep classic DShot, which honors motor_pwm_inversion
    if (motorPwmInversion) {
        bbStatus = DSHOT_BITBANG_STATUS_INVERSION;
        return false;
    }
#if defined(STM32F4) && defined(USE_GYRO_IMUF9001)
    // the IMU-F SPI DMA sits on DMA2, and bitbang writes GPIO through DMA2 (F4 errata 2.1.10)
    bbStatus = DSHOT_BITBANG_STATUS_SPI_DMA2;
    return false;
#else
    return true;
#endif
}

static bbPacer_t *bbFindMotorPacer(TIM_TypeDef *tim)
{
    for (int i = 0; i < usedMotorPacers; i++) {
        if (bbPacers[i].tim == tim) {
            return &bbPacers[i];
        }
    }
    if (usedMotorPacers >= MAX_MOTOR_PACERS) {
        return NULL;
    }
    bbPacers[usedMotorPacers].tim = tim;
    return &bbPacers[usedMotorPacers++];
}

static bbPort_t *bbFindMotorPort(int portIndex)
{
    for (int i = 0; i < usedMotorPorts; i++) {
        if (bbPorts[i].portIndex == portIndex) {
            return &bbPorts[i];
        }
    }
    return NULL;
}

static void bbDMASaveRegs(bbPort_t *bbPort)
{
    const DMA_Stream_TypeDef *stream = bbPort->dmaRef;

    bbPort->dmaRegs.CR = stream->CR;
    bbPort->dmaRegs.FCR = stream->FCR;
    bbPort->dmaRegs.NDTR = stream->NDTR;
    bbPort->dmaRegs.PAR = stream->PAR;
    bbPort->dmaRegs.M0AR = stream->M0AR;
}

// The stream registers are rewritten each frame because NDTR is spent and a transfer error may leave the stream half-configured
static void bbDMALoadRegs(const bbPort_t *bbPort)
{
    DMA_Stream_TypeDef *stream = bbPort->dmaRef;

    stream->CR = bbPort->dmaRegs.CR;
    stream->FCR = bbPort->dmaRegs.FCR;
    stream->NDTR = bbPort->dmaRegs.NDTR;
    stream->PAR = bbPort->dmaRegs.PAR;
    stream->M0AR = bbPort->dmaRegs.M0AR;
}

static bool bbDMAIsEnabled(const bbPort_t *bbPort)
{
    return (bbPort->dmaRef->CR & DMA_SxCR_EN) != 0;
}

// Every event flag must be clear before the stream is enabled again
#ifdef DMA_IT_FEIF
#define BB_DMA_ALL_FLAGS (DMA_IT_TCIF | DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF)
#else
#define BB_DMA_ALL_FLAGS (DMA_IT_TCIF | DMA_IT_HTIF | DMA_IT_TEIF)
#endif

static void bbDMAClearFlags(const bbPort_t *bbPort)
{
    dmaChannelDescriptor_t *descriptor = dmaGetDescriptorByIdentifier(bbPort->dmaIdentifier);

    DMA_CLEAR_FLAG(descriptor, BB_DMA_ALL_FLAGS);
}

// Stops the stream and its pacer request, then waits for EN to clear. False if it did not clear.
static bool bbPortStop(const bbPort_t *bbPort)
{
    bbDMACmd(bbPort, false);
    bbTIMDMACmd(bbPort->tim, bbPort->dmaSource, false);
    for (int spin = 0; spin < BB_DMA_STOP_SPINS && bbDMAIsEnabled(bbPort); spin++) {
    }
    return !bbDMAIsEnabled(bbPort);
}

// True while the port's buffer and registers must stay untouched. A stream still enabled
// after twice the frame time is stuck: it is stopped here, before anything changes.
static bool bbPortIsBusy(const bbPort_t *bbPort)
{
    if (!bbDMAIsEnabled(bbPort)) {
        return false;
    }
    if (cmpTimeUs(micros(), bbPort->startUs) < (int32_t)(2 * bbPort->frameUs)) {
        return true;
    }
    if (!bbPortStop(bbPort)) {
        return true;
    }
    bbDMAClearFlags(bbPort);
    return false;
}

static uint32_t getDshotBaseFrequency(uint8_t protocol)
{
    switch (protocol) {
    case PWM_TYPE_DSHOT600:
        return 600000 * MOTOR_DSHOT_STATE_PER_SYMBOL;
    case PWM_TYPE_DSHOT300:
        return 300000 * MOTOR_DSHOT_STATE_PER_SYMBOL;
    default:
    case PWM_TYPE_DSHOT150:
        return 150000 * MOTOR_DSHOT_STATE_PER_SYMBOL;
    }
}

// Picks a free pacer timer channel and its free DMA stream for a new port group
static bool bbFindPacer(bbPort_t *bbPort, uint8_t timerSelect)
{
    for (unsigned row = 0; row < ARRAYLEN(bbPacerRows); row++) {
        TIM_TypeDef *tim = bbPacerRows[row].tim;
        const uint8_t channel = bbPacerRows[row].channel;
        const int8_t timNumber = timerGetTIMNumber(tim);

        if ((timerSelect == DSHOT_BITBANGED_TIMER_TIM1 && timNumber != 1)
            || (timerSelect == DSHOT_BITBANGED_TIMER_TIM8 && timNumber != 8)) {
            continue;
        }

        // another user on the timer would have its time base replaced
        bool conflict = false;
        for (int ch = 0; ch < CC_CHANNELS_PER_TIMER && !conflict; ch++) {
            conflict = timerGetAllocatedByNumberAndChannel(timNumber, CC_CHANNEL_FROM_INDEX(ch)) != NULL;
        }
        for (int i = 0; i < usedMotorPorts && !conflict; i++) {
            conflict = &bbPorts[i] != bbPort && bbPorts[i].tim == tim && bbPorts[i].timChannel == channel;
        }
        if (conflict) {
            continue;
        }

        // a negative dmaopt tries every option: H7 DMAMUX lets any free stream serve the request
        const int8_t firstOpt = bbPacerRows[row].dmaopt < 0 ? 0 : bbPacerRows[row].dmaopt;
        const int8_t lastOpt = bbPacerRows[row].dmaopt < 0 ? MAX_TIMER_DMA_OPTIONS - 1 : bbPacerRows[row].dmaopt;
        for (int8_t opt = firstOpt; opt <= lastOpt; opt++) {
            // the H7 spec table is shared between options: read it before the next lookup
            const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByTimerValue(tim, channel, opt);
            if (!dmaSpec || !dmaSpec->ref) {
                continue;
            }
            DMA_Stream_TypeDef *dmaRef = (DMA_Stream_TypeDef *)dmaSpec->ref;
            const uint32_t dmaChannel = dmaSpec->channel;
            const dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(dmaRef);
            if (dmaIdentifier == DMA_NONE || dmaGetOwner(dmaIdentifier) != OWNER_FREE) {
                continue;
            }
            bool streamTaken = false;
            for (int i = 0; i < usedMotorPorts && !streamTaken; i++) {
                streamTaken = &bbPorts[i] != bbPort && bbPorts[i].dmaIdentifier == dmaIdentifier;
            }
            if (streamTaken) {
                continue;
            }

            bbPort->tim = tim;
            bbPort->timChannel = channel;
            bbPort->dmaRef = dmaRef;
            bbPort->dmaChannel = dmaChannel;
            bbPort->dmaIdentifier = dmaIdentifier;
            bbPort->dmaSource = timerDmaSource(channel);
            return true;
        }
    }
    return false;
}

FAST_IRQ_HANDLER void bbDMAIrqHandler(dmaChannelDescriptor_t *descriptor)
{
    bbPort_t *bbPort = (bbPort_t *)descriptor->userParam;

    // a stream stopped by software raises TC; a pending interrupt left after its flags were cleared must not cut the next frame
    if (!DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF | DMA_IT_TEIF)) {
        return;
    }

    bbDMACmd(bbPort, false);
    bbTIMDMACmd(bbPort->tim, bbPort->dmaSource, false);
    // the next frame reloads every register, so an error flag is only cleared here
    DMA_CLEAR_FLAG(descriptor, BB_DMA_ALL_FLAGS);
}

static bool bbSetupPort(bbPort_t *bbPort, IO_t io, const motorDevConfig_t *motorConfig)
{
    if (!bbFindPacer(bbPort, motorConfig->useDshotBitbangedTimer)) {
        bbStatus = DSHOT_BITBANG_STATUS_NO_PACER;
        return false;
    }
    if (!dmaAllocate(bbPort->dmaIdentifier, OWNER_DSHOT_BITBANG, RESOURCE_INDEX(bbPort->portIndex))) {
        bbStatus = DSHOT_BITBANG_STATUS_DMA_CLAIM;
        return false;
    }
    bbPacer_t *bbPacer = bbFindMotorPacer(bbPort->tim);
    if (!bbPacer) {
        bbStatus = DSHOT_BITBANG_STATUS_NO_PACER;
        return false;
    }
    bbPacer->dmaSources |= bbPort->dmaSource;

    bbPort->gpio = IO_GPIO(io);
    bbPort->portOutputCount = MOTOR_DSHOT_BUF_LENGTH;
    bbPort->portOutputBuffer = &bbOutputBuffer[(bbPort - bbPorts) * BB_PORT_BUF_WORDS];
    const uint32_t baseFrequency = getDshotBaseFrequency(motorConfig->motorPwmProtocol);
    bbPort->outputARR = timerClock(bbPort->tim) / baseFrequency - 1;
    bbPort->frameUs = (1000000ull * MOTOR_DSHOT_BUF_LENGTH) / baseFrequency + 1;

    bbTimerChannelInit(bbPort);
    dmaEnable(bbPort->dmaIdentifier);
    bbDMAPreconfigure(bbPort);
    bbDMASaveRegs(bbPort);
    dmaSetHandler(bbPort->dmaIdentifier, bbDMAIrqHandler, NVIC_BUILD_PRIORITY(2, 1), (uintptr_t)bbPort);
    return true;
}

bool dshotBitbangMotorInit(uint8_t motorIndex, uint8_t reorderedMotorIndex, ioTag_t tag)
{
    const IO_t io = IOGetByTag(tag);

    // brushed ESC autodetect leaves motor 1 owned by SYSTEM; classic DShot overwrites that too
#if defined(USE_BRUSHED_ESC_AUTODETECT) && defined(MOTOR1_PIN)
    const bool autodetectPin = tag == IO_TAG(MOTOR1_PIN);
#else
    const bool autodetectPin = false;
#endif
    if (!io || !(IOIsFreeOrPreinit(io) || (autodetectPin && IOGetOwner(io) == OWNER_SYSTEM))) {
        bbStatus = DSHOT_BITBANG_STATUS_MOTOR_PIN_CONFLICT;
        return false;
    }

    bbMotor_t *bbMotor = &bbMotors[motorIndex];
    bbMotor->io = io;
    bbMotor->pinIndex = IO_GPIOPinIdx(io);
    bbMotor->portIndex = IO_GPIOPortIdx(io);
#if defined(STM32F4)
    bbMotor->iocfg = IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_DOWN);
#else
    bbMotor->iocfg = IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_PULLDOWN);
#endif

    IOInit(io, OWNER_MOTOR, RESOURCE_INDEX(reorderedMotorIndex));
    IOConfigGPIO(io, bbMotor->iocfg);
    IOLo(io);
    return true;
}

bool dshotBitbangPostInit(const motorDevConfig_t *motorConfig, uint8_t motorCount)
{
    bbStatus = DSHOT_BITBANG_STATUS_OK;
    memset(bbOutputBuffer, 0, sizeof(bbOutputBuffer));

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++) {
        bbMotor_t *bbMotor = &bbMotors[motorIndex];
        if (!bbMotor->io) {
            bbStatus = DSHOT_BITBANG_STATUS_MOTOR_PIN_CONFLICT;
            return false;
        }

        bbPort_t *bbPort = bbFindMotorPort(bbMotor->portIndex);
        if (!bbPort) {
            if (usedMotorPorts >= MAX_SUPPORTED_MOTOR_PORTS) {
                bbStatus = DSHOT_BITBANG_STATUS_TOO_MANY_PORTS;
                return false;
            }
            bbPort = &bbPorts[usedMotorPorts++];
            bbPort->portIndex = bbMotor->portIndex;
            if (!bbSetupPort(bbPort, bbMotor->io, motorConfig)) {
                return false;
            }
        }
        bbMotor->bbPort = bbPort;
        dshotBitbangOutputDataInit(bbPort->portOutputBuffer, 1 << bbMotor->pinIndex);
        // a motor starts on the stop packet, never on an all-ones frame (full throttle)
        dshotBitbangOutputDataSet(bbPort->portOutputBuffer, bbMotor->pinIndex, 0);
        bbMotor->configured = true;
    }

    bbRunning = true;
    return true;
}

// Passthrough leaves the motor pins, pacer timers and DMA streams in other modes: set them up again.
void dshotBitbangRestore(void)
{
    if (!bbRunning) {
        return;
    }

    // a frame may still be running: the timer and DMA are only rebuilt once each stream has stopped
    bool stopped[MAX_SUPPORTED_MOTOR_PORTS];
    for (int i = 0; i < usedMotorPorts; i++) {
        stopped[i] = bbPortStop(&bbPorts[i]);
    }
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        if (bbMotors[i].configured) {
            IOConfigGPIO(bbMotors[i].io, bbMotors[i].iocfg);
            IOLo(bbMotors[i].io);
        }
    }
    for (int i = 0; i < usedMotorPorts; i++) {
        bbPort_t *bbPort = &bbPorts[i];

        if (!stopped[i]) {
            continue; // the next update sees it as busy and handles it
        }
        bbTimerChannelInit(bbPort);
        bbDMAPreconfigure(bbPort);
        bbDMASaveRegs(bbPort);
        bbDMAClearFlags(bbPort);
        bbPort->startUs = 0;
    }
}

void dshotBitbangWriteInt(uint8_t motorIndex, uint16_t value)
{
    bbMotor_t *const bbMotor = &bbMotors[motorIndex];

    if (!bbRunning || !bbMotor->configured) {
        return;
    }

    // rewriting the buffer under a running frame would corrupt it; this motor repeats its packet
    if (bbPortIsBusy(bbMotor->bbPort)) {
        return;
    }

    motorDmaOutput_t *const motor = getMotorDmaOutput(motorIndex);
    if (pwmDshotCommandIsProcessing()) {
        value = pwmGetDshotCommand(motorIndex);
        if (value) {
            motor->requestTelemetry = true;
        }
    }
    motor->value = value;
    dshotBitbangOutputDataSet(bbMotor->bbPort->portOutputBuffer, bbMotor->pinIndex, prepareDshotPacket(motor));
}

void dshotBitbangUpdateComplete(uint8_t motorCount)
{
    if (!bbRunning) {
        return;
    }

    if (pwmDshotCommandIsQueued() && !pwmDshotCommandOutputIsEnabled(motorCount)) {
        return;
    }

    for (int i = 0; i < usedMotorPorts; i++) {
        bbPort_t *bbPort = &bbPorts[i];

        if (bbPortIsBusy(bbPort)) {
            continue; // the frame in flight finishes; this update is skipped for the port
        }

        bbDMAClearFlags(bbPort);
        bbDMALoadRegs(bbPort);
        bbPort->startUs = micros();
        bbDMACmd(bbPort, true);
    }

    for (int i = 0; i < usedMotorPacers; i++) {
        bbTIMDMACmd(bbPacers[i].tim, bbPacers[i].dmaSources, true);
    }
}

#endif // USE_DSHOT_BITBANG
