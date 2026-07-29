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
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#include "build/atomic.h"
#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/bus_spi.h"
#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/time.h"

#ifdef USE_GYRO_IMUF9001
#include "sensors/gyro.h"
#include "sensors/acceleration.h"
#endif

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu3050.h"
#include "drivers/accgyro/accgyro_mpu6050.h"
#include "drivers/accgyro/accgyro_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_bmi160.h"
#include "drivers/accgyro/accgyro_spi_bmi270.h"
#include "drivers/accgyro/accgyro_spi_icm20649.h"
#include "drivers/accgyro/accgyro_spi_icm20689.h"
#include "drivers/accgyro/accgyro_spi_icm426xx.h"
#include "drivers/accgyro/accgyro_spi_mpu6000.h"
#include "drivers/accgyro/accgyro_spi_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_mpu9250.h"
#include "drivers/accgyro/accgyro_mpu.h"
#ifdef USE_GYRO_IMUF9001
#include "drivers/accgyro/accgyro_imuf9001.h"
#include "rx/rx.h"
#include "fc/fc_rc.h"
#include "fc/runtime_config.h"
#endif //USE_GYRO_IMUF9001

mpuResetFnPtr mpuResetFn;

#ifdef USE_GYRO_IMUF9001
imufData_t imufData;
#endif
#ifndef MPU_I2C_INSTANCE
#define MPU_I2C_INSTANCE I2C_DEVICE
#endif

#ifndef MPU_ADDRESS
#define MPU_ADDRESS             0x68
#endif

#define MPU_INQUIRY_MASK   0x7E

// Minimum system uptime (ms) before SPI bus access — boot-time guard, not a relative delay
#define GYRO_SPI_STARTUP_MS 100

#define GYRO_EXTI_DETECT_THRESHOLD 1000

#if defined(USE_I2C)
static void mpu6050FindRevision(gyroDev_t *gyro) {
    // There is a map of revision contained in the android source tree which is quite comprehensive and may help to understand this code
    // See https://android.googlesource.com/kernel/msm.git/+/eaf36994a3992b8f918c18e4f7411e8b2320a35f/drivers/misc/mpu6050/mldl_cfg.c
    // determine product ID and revision
    uint8_t readBuffer[6];
    bool ack = busReadRegisterBuffer(&gyro->dev, MPU_RA_XA_OFFS_H, readBuffer, 6);
    uint8_t revision = ((readBuffer[5] & 0x01) << 2) | ((readBuffer[3] & 0x01) << 1) | (readBuffer[1] & 0x01);
    if (ack && revision) {
        // Congrats, these parts are better
        if (revision == 1) {
            gyro->mpuDetectionResult.resolution = MPU_HALF_RESOLUTION;
        } else if (revision == 2) {
            gyro->mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        } else if ((revision == 3) || (revision == 7)) {
            gyro->mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        } else {
            failureMode(FAILURE_ACC_INCOMPATIBLE);
        }
    } else {
        uint8_t productId;
        ack = busReadRegisterBuffer(&gyro->dev, MPU_RA_PRODUCT_ID, &productId, 1);
        revision = productId & 0x0F;
        if (!ack || revision == 0) {
            failureMode(FAILURE_ACC_INCOMPATIBLE);
        } else if (revision == 4) {
            gyro->mpuDetectionResult.resolution = MPU_HALF_RESOLUTION;
        } else {
            gyro->mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        }
    }
}
#endif

/*
 * Gyro interrupt service routine
 */
#if defined(MPU_INT_EXTI)
// Called in ISR context after DMA transfer completes
busStatus_e mpuIntCallback(uint32_t arg)
{
    gyroDev_t *gyro = (gyroDev_t *)arg;
    int32_t gyroDmaDuration = cmpTimeCycles(getCycleCounter(), gyro->gyroLastEXTI);
    if (gyroDmaDuration > gyro->gyroDmaMaxDuration) {
        gyro->gyroDmaMaxDuration = gyroDmaDuration;
    }
    gyro->dataReady = true;
    return BUS_READY;
}

FAST_CODE static void mpuIntExtiHandler(extiCallbackRec_t *cb) {
    gyroDev_t *gyro = container_of(cb, gyroDev_t, exti);
    uint32_t nowCycles = getCycleCounter();
    int32_t gyroLastPeriod = cmpTimeCycles(nowCycles, gyro->gyroLastEXTI);
    if ((gyro->gyroShortPeriod == 0) || (gyroLastPeriod < gyro->gyroShortPeriod)) {
        gyro->gyroSyncEXTI = gyro->gyroLastEXTI + gyro->gyroDmaMaxDuration;
    }
    gyro->gyroLastEXTI = nowCycles;
#if defined(USE_GYRO_IMUF9001)
    // Keep EXTI ISR < 1 µs: SPI transfer is done by imufSpiGyroRead in GYROPID task context.
    imufTransferPending = true;
#elif defined(GYRO_USES_SPI)
    if (gyro->gyroModeSPI == GYRO_EXTI_INT_DMA) {
        spiSequence(&gyro->dev, gyro->segments);
    }
#endif
    gyro->detectedEXTI++;
}

static void mpuIntExtiInit(gyroDev_t *gyro) {
    if (gyro->mpuIntExtiTag == IO_TAG_NONE) {
        return;
    }
    const IO_t mpuIntIO = IOGetByTag(gyro->mpuIntExtiTag);
#ifdef ENSURE_MPU_DATA_READY_IS_LOW
    uint8_t status = IORead(mpuIntIO);
    if (status) {
        return;
    }
#endif
    IOInit(mpuIntIO, OWNER_MPU_EXTI, 0);
    IOConfigGPIO(mpuIntIO, IOCFG_IN_FLOATING);
    EXTIHandlerInit(&gyro->exti, mpuIntExtiHandler);
    EXTIConfig(mpuIntIO, &gyro->exti, NVIC_PRIO_MPU_INT_EXTI, IOCFG_IN_FLOATING, EXTI_TRIGGER_RISING);
    EXTIEnable(mpuIntIO, true);
}
#endif // MPU_INT_EXTI

bool mpuAccRead(accDev_t *acc) {
    uint8_t data[6];
    const bool ack = busReadRegisterBuffer(&acc->dev, MPU_RA_ACCEL_XOUT_H, data, 6);
    if (!ack) {
        return false;
    }
    acc->ADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
    acc->ADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
    acc->ADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);
    return true;
}

#ifdef USE_GYRO_IMUF9001
// NOTE: NOT in CCM (FAST_RAM_ZERO_INIT) because the polling SPI path in spiSequenceStart
// checks IS_CCM() and falls through to a slow path; keep in regular SRAM for normal access.
static uint8_t imufTxBuf[58];
static uint8_t imufRxBuf[58];
STATIC_ASSERT(sizeof(imufTxBuf) >= sizeof(imufCommand_t), imuf_tx_buffer_too_small);
STATIC_ASSERT(sizeof(imufRxBuf) >= sizeof(imufData_t), imuf_rx_buffer_too_small);
FAST_RAM_ZERO_INIT volatile uint32_t crcErrorCount = 0;
// Set by EXTI ISR; consumed (cleared then SPI executed) by imufSpiGyroRead in GYROPID task.
// Keeps EXTI ISR duration < 1 µs; moves 15–20 µs SPI polling out of interrupt context.
FAST_RAM_ZERO_INIT volatile bool imufTransferPending;

// DMA completion callback: copy data into gyroADCf and acc.dev.ADCRaw.
// CRC is tallied for diagnostics only — always use data (matches original dma_spi behavior).
FAST_CODE busStatus_e imufIntCallback(uint32_t arg) {
    gyroDev_t *gyro = (gyroDev_t *)arg;
    const uint32_t xferLen = gyro->segments[0].len;
    const uint32_t crc1 = *(uint32_t *)(imufRxBuf + xferLen - 4);
    const uint32_t crc2 = getCrcImuf9001((uint32_t *)imufRxBuf, (xferLen >> 2) - 1);
    if (crc1 != crc2) {
        if (++crcErrorCount > 100000) {
            crcErrorCount = 0;
        }
    }
    memcpy(&imufData, imufRxBuf, sizeof(imufData_t));
    acc.dev.ADCRaw[X]   = (int16_t)(imufData.accX * acc.dev.acc_1G);
    acc.dev.ADCRaw[Y]   = (int16_t)(imufData.accY * acc.dev.acc_1G);
    acc.dev.ADCRaw[Z]   = (int16_t)(imufData.accZ * acc.dev.acc_1G);
    gyro->gyroADCf[X]   = imufData.gyroX;
    gyro->gyroADCf[Y]   = imufData.gyroY;
    gyro->gyroADCf[Z]   = imufData.gyroZ;
    gyro->gyroADCRaw[X] = (int16_t)(imufData.gyroX * 16.4f);
    gyro->gyroADCRaw[Y] = (int16_t)(imufData.gyroY * 16.4f);
    gyro->gyroADCRaw[Z] = (int16_t)(imufData.gyroZ * 16.4f);
    gyro->dataReady = true;
    return BUS_READY;
}

// Prepare TX command and transfer length before each real-time DMA read.
FAST_CODE void imufPrepareDmaRead(gyroDev_t *gyro) {
    imufCommand_t *txCmd = (imufCommand_t *)imufTxBuf;
    memset(imufTxBuf, 0, sizeof(imufCommand_t));
    if (isImufCalibrating == IMUF_IS_CALIBRATING) {
        txCmd->command = IMUF_COMMAND_CALIBRATE;
        txCmd->crc     = getCrcImuf9001((uint32_t *)imufTxBuf, 11);
        isImufCalibrating = IMUF_DONE_CALIBRATING;
    } else if (isImufCalibrating == IMUF_DONE_CALIBRATING) {
        imufEndCalibration();
    } else if (isSetpointNew) {
        txCmd->command = IMUF_COMMAND_SETPOINT;
        txCmd->param1  = getSetpointRateInt(0);
        txCmd->param2  = getSetpointRateInt(1);
        txCmd->param3  = getSetpointRateInt(2);
        txCmd->crc     = getCrcImuf9001((uint32_t *)imufTxBuf, 11);
        isSetpointNew = 0;
    }
    const uint32_t xferLen = MIN((uint32_t)gyroConfig()->imuf_mode, (uint32_t)sizeof(imufTxBuf));
    memset(imufRxBuf, 0, xferLen);
    gyro->segments[0].len = xferLen;
}

// Dedicated DMA state for the SPI1/IMUF9001 link. Deliberately separate from
// busDevice_t/spiInitBusDMA()'s shared multi-device bus machinery - see
// accgyro_mpu.h comment on imufDmaStartTransfer for why. HELIOSPRING/MODE2FLUX
// are F405 (stdperiph DMA API, matches bus_spi_stdperiph.c); STRIXF10 is F722
// (LL DMA API, matches bus_spi_ll.c) - these are genuinely different vendor
// library APIs, not just naming, so the two are implemented separately below
// rather than pretending one covers both.
static dmaChannelDescriptor_t *imufDmaTx;
static dmaChannelDescriptor_t *imufDmaRx;
static gyroDev_t *imufDmaGyro;
// Defined per-MCU below (F4/F7 have different register-level APIs); declared
// here since imufDmaAllocateChannels() registers it before either definition.
static void imufDmaCompleteIsr(dmaChannelDescriptor_t *descriptor);

// Resolves and allocates SPI1's TX/RX DMA channel via the existing, already-
// correct per-target mapping in dma_reqmap_mcu.c (DMA2 Stream3/Stream0 on
// F405; F722 uses the same assignment - only F745/F746/F765 differ, and none
// of the three IMUF9001 targets are those chips). Shared by both MCU branches
// below; only the register-level DMA_Init/DMA_Cmd API differs after this.
static bool imufDmaAllocateChannels(gyroDev_t *gyro) {
    const dmaChannelSpec_t *txSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_SPI_SDO, SPIDEV_1, 0);
    const dmaChannelSpec_t *rxSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_SPI_SDI, SPIDEV_1, 0);
    // This target's SPI1 DMA entry is fixed and verified-correct in dma_reqmap_mcu.c -
    // should never be NULL. Guarded anyway (matches spiInitBusDMA()'s own defensive
    // pattern) rather than assuming the target table can't regress.
    if (!txSpec || !rxSpec) {
        failureMode(FAILURE_GYRO_INIT_FAILED);
        return false;
    }

    const dmaIdentifier_e txId = dmaGetIdentifier((DMA_Stream_TypeDef *)txSpec->ref);
    const dmaIdentifier_e rxId = dmaGetIdentifier((DMA_Stream_TypeDef *)rxSpec->ref);
    // dmaAllocate() itself guards identifier == DMA_NONE, but its return value must be
    // checked before touching the descriptor via dmaGetDescriptorByIdentifier() - matches
    // the defensive pattern spiInitBusDMA() already uses for this exact sequence.
    if (!dmaAllocate(txId, OWNER_SPI_SDO, SPIDEV_1 + 1) || !dmaAllocate(rxId, OWNER_SPI_SDI, SPIDEV_1 + 1)) {
        failureMode(FAILURE_GYRO_INIT_FAILED);
        return false;
    }
    imufDmaTx = dmaGetDescriptorByIdentifier(txId);
    imufDmaRx = dmaGetDescriptorByIdentifier(rxId);
    imufDmaTx->stream  = DMA_DEVICE_INDEX(txId);
    imufDmaTx->channel = txSpec->channel;
    imufDmaRx->stream  = DMA_DEVICE_INDEX(rxId);
    imufDmaRx->channel = rxSpec->channel;
    dmaEnable(txId);
    dmaEnable(rxId);
    dmaSetHandler(rxId, imufDmaCompleteIsr, NVIC_PRIO_SPI_DMA, 0);
    (void)gyro;
    return true;
}

#if defined(STM32F4)

static DMA_InitTypeDef imufDmaInitTx;
static DMA_InitTypeDef imufDmaInitRx;

// RX stream transfer-complete ISR: SPI operation is done once RX completes
// (matches the reasoning already used by the generic path's own comment).
// Clears only the stream-enable bit, not the whole CR - the rest of CR
// (channel/direction/inc-mode/priority) is programmed once in
// mpuImufSetupDma() and must survive across transfers so imufDmaStartTransfer
// never needs to call DMA_Init() again.
FAST_CODE static void imufDmaCompleteIsr(dmaChannelDescriptor_t *descriptor) {
    DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    ((DMA_Stream_TypeDef *)imufDmaTx->ref)->CR &= ~(uint32_t)DMA_SxCR_EN;
    ((DMA_Stream_TypeDef *)imufDmaRx->ref)->CR &= ~(uint32_t)DMA_SxCR_EN;
    SPI_I2S_DMACmd(imufDmaGyro->dev.bus->busType_u.spi.instance, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, DISABLE);
    IOHi(imufDmaGyro->dev.busType_u.spi.csnPin);
    imufIntCallback((uint32_t)imufDmaGyro);
}

// One-time setup: allocate DMA channels for SPI1, program the full stream
// configuration once (channel/direction/inc-mode/data-size/priority plus an
// initial address/length), and register our own completion handler. Called
// once from imufSpiGyroInit after mpuGyroInit (gyro->dev.bus is valid by then).
void mpuImufSetupDma(gyroDev_t *gyro) {
    imufDmaGyro = gyro;
    gyro->segments[0].len = gyroConfig()->imuf_mode;

    if (!imufDmaAllocateChannels(gyro)) {
        return;
    }

    DMA_StructInit(&imufDmaInitTx);
    imufDmaInitTx.DMA_Channel            = imufDmaTx->channel;
    imufDmaInitTx.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
    imufDmaInitTx.DMA_PeripheralBaseAddr = (uint32_t)&gyro->dev.bus->busType_u.spi.instance->DR;
    imufDmaInitTx.DMA_Memory0BaseAddr    = (uint32_t)imufTxBuf;
    imufDmaInitTx.DMA_BufferSize         = gyro->segments[0].len;
    imufDmaInitTx.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    imufDmaInitTx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    imufDmaInitTx.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    imufDmaInitTx.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    imufDmaInitTx.DMA_Priority           = DMA_Priority_Low;

    imufDmaInitRx                     = imufDmaInitTx;
    imufDmaInitRx.DMA_DIR             = DMA_DIR_PeripheralToMemory;
    imufDmaInitRx.DMA_Memory0BaseAddr = (uint32_t)imufRxBuf;
    imufDmaInitRx.DMA_Priority        = DMA_Priority_Medium;

    DMA_Init((DMA_Stream_TypeDef *)imufDmaTx->ref, &imufDmaInitTx);
    DMA_Init((DMA_Stream_TypeDef *)imufDmaRx->ref, &imufDmaInitRx);
    DMA_ITConfig((DMA_Stream_TypeDef *)imufDmaRx->ref, DMA_IT_TC, ENABLE);
}

// Kick off one DMA transfer for the current imufTxBuf/imufRxBuf contents and
// segment length (already prepared by imufPrepareDmaRead). Non-blocking -
// completion arrives asynchronously via imufDmaCompleteIsr. Stream
// configuration (channel/direction/inc-mode/priority/IT enable) was already
// programmed once in mpuImufSetupDma() and is preserved by
// imufDmaCompleteIsr's EN-only clear, so only the per-transfer address
// (M0AR) and length (NDTR) need updating here - no DMA_Init() re-run.
FAST_CODE void imufDmaStartTransfer(gyroDev_t *gyro) {
    DMA_Stream_TypeDef *streamTx = (DMA_Stream_TypeDef *)imufDmaTx->ref;
    DMA_Stream_TypeDef *streamRx = (DMA_Stream_TypeDef *)imufDmaRx->ref;
    const uint32_t len = gyro->segments[0].len;

    DMA_CLEAR_FLAG(imufDmaTx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
    DMA_CLEAR_FLAG(imufDmaRx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);

    // NDTR/M0AR may only be written while EN=0 (RM0090) - already true here
    // since imufDmaCompleteIsr cleared EN at the end of the prior transfer.
    streamTx->M0AR = (uint32_t)imufTxBuf;
    streamTx->NDTR = len;
    streamRx->M0AR = (uint32_t)imufRxBuf;
    streamRx->NDTR = len;

    DMA_Cmd(streamTx, ENABLE);
    DMA_Cmd(streamRx, ENABLE);

    IOLo(gyro->dev.busType_u.spi.csnPin);
    SPI_I2S_DMACmd(gyro->dev.bus->busType_u.spi.instance, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);
}

#elif defined(STM32F7)

// File-local in bus_spi_ll.c, not exposed via a shared header - redefined here
// for the same reason (F7 D-cache line size, needed for cache maintenance below).
#define CACHE_LINE_SIZE  32
#define CACHE_LINE_MASK  (CACHE_LINE_SIZE - 1)

static LL_DMA_InitTypeDef imufDmaInitTx;
static LL_DMA_InitTypeDef imufDmaInitRx;

FAST_CODE static void imufDmaCompleteIsr(dmaChannelDescriptor_t *descriptor) {
    DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    LL_DMA_DisableStream(imufDmaTx->dma, imufDmaTx->stream);
    LL_DMA_DisableStream(imufDmaRx->dma, imufDmaRx->stream);
    CLEAR_BIT(imufDmaGyro->dev.bus->busType_u.spi.instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
    IOHi(imufDmaGyro->dev.busType_u.spi.csnPin);
    imufIntCallback((uint32_t)imufDmaGyro);
}

// One-time setup: allocate DMA channels for SPI1, program the full stream
// configuration once (channel/direction/inc-mode/data-size/priority plus an
// initial address/length), and enable the TC interrupt. Called once from
// imufSpiGyroInit after mpuGyroInit (gyro->dev.bus is valid by then).
void mpuImufSetupDma(gyroDev_t *gyro) {
    imufDmaGyro = gyro;
    gyro->segments[0].len = gyroConfig()->imuf_mode;

    if (!imufDmaAllocateChannels(gyro)) {
        return;
    }

    LL_DMA_StructInit(&imufDmaInitTx);
    imufDmaInitTx.Channel                = imufDmaTx->channel;
    imufDmaInitTx.Mode                   = LL_DMA_MODE_NORMAL;
    imufDmaInitTx.Direction               = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    imufDmaInitTx.PeriphOrM2MSrcAddress   = (uint32_t)&gyro->dev.bus->busType_u.spi.instance->DR;
    imufDmaInitTx.MemoryOrM2MDstAddress   = (uint32_t)imufTxBuf;
    imufDmaInitTx.NbData                  = gyro->segments[0].len;
    imufDmaInitTx.Priority                = LL_DMA_PRIORITY_LOW;
    imufDmaInitTx.PeriphOrM2MSrcIncMode   = LL_DMA_PERIPH_NOINCREMENT;
    imufDmaInitTx.PeriphOrM2MSrcDataSize  = LL_DMA_PDATAALIGN_BYTE;
    imufDmaInitTx.MemoryOrM2MDstDataSize  = LL_DMA_MDATAALIGN_BYTE;
    imufDmaInitTx.MemoryOrM2MDstIncMode   = LL_DMA_MEMORY_INCREMENT;

    imufDmaInitRx                        = imufDmaInitTx;
    imufDmaInitRx.Channel                = imufDmaRx->channel;
    imufDmaInitRx.Direction               = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    imufDmaInitRx.MemoryOrM2MDstAddress   = (uint32_t)imufRxBuf;

    LL_DMA_Init(imufDmaTx->dma, imufDmaTx->stream, &imufDmaInitTx);
    LL_DMA_Init(imufDmaRx->dma, imufDmaRx->stream, &imufDmaInitRx);
    LL_EX_DMA_EnableIT_TC(imufDmaRx->ref);
}

// Kick off one DMA transfer. F7 has a D-cache that DMA bypasses - imufTxBuf/
// imufRxBuf are plain SRAM (not DTCM), so cache maintenance is mandatory here:
// clean (writeback) TX before the DMA reads it, invalidate RX after so the CPU
// doesn't read stale cached data once the DMA has written fresh bytes. F4 has
// no cache, hence no equivalent step in the STM32F4 branch above.
//
// Stream configuration (channel/direction/inc-mode/priority/IT enable) was
// already programmed once in mpuImufSetupDma() and is preserved by
// imufDmaCompleteIsr's LL_DMA_DisableStream() (EN-bit-only, per the LL
// driver), so only the per-transfer address and length need updating here -
// no LL_DMA_Init() re-run.
FAST_CODE void imufDmaStartTransfer(gyroDev_t *gyro) {
    const uint32_t len = gyro->segments[0].len;

    DMA_CLEAR_FLAG(imufDmaTx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);
    DMA_CLEAR_FLAG(imufDmaRx, DMA_IT_HTIF | DMA_IT_TEIF | DMA_IT_TCIF);

#ifdef __DCACHE_PRESENT
    SCB_CleanDCache_by_Addr((uint32_t *)((uint32_t)imufTxBuf & ~CACHE_LINE_MASK),
        (((uint32_t)imufTxBuf & CACHE_LINE_MASK) + len - 1 + CACHE_LINE_SIZE) & ~CACHE_LINE_MASK);
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)((uint32_t)imufRxBuf & ~CACHE_LINE_MASK),
        (((uint32_t)imufRxBuf & CACHE_LINE_MASK) + len - 1 + CACHE_LINE_SIZE) & ~CACHE_LINE_MASK);
#endif

    // NbData/MemoryAddress may only be written while EN=0 (RM0431) - already
    // true here since imufDmaCompleteIsr disabled the stream at the end of
    // the prior transfer.
    LL_DMA_SetMemoryAddress(imufDmaTx->dma, imufDmaTx->stream, (uint32_t)imufTxBuf);
    LL_DMA_SetDataLength(imufDmaTx->dma, imufDmaTx->stream, len);
    LL_DMA_SetMemoryAddress(imufDmaRx->dma, imufDmaRx->stream, (uint32_t)imufRxBuf);
    LL_DMA_SetDataLength(imufDmaRx->dma, imufDmaRx->stream, len);

    LL_DMA_EnableStream(imufDmaTx->dma, imufDmaTx->stream);
    LL_DMA_EnableStream(imufDmaRx->dma, imufDmaRx->stream);

    IOLo(gyro->dev.busType_u.spi.csnPin);
    SET_BIT(gyro->dev.bus->busType_u.spi.instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
}

#endif // STM32F4 / STM32F7
#endif // USE_GYRO_IMUF9001


FAST_CODE bool mpuGyroRead(gyroDev_t *gyro) {
    uint8_t data[6];
    const bool ack = busReadRegisterBuffer(&gyro->dev, MPU_RA_GYRO_XOUT_H, data, 6);
    if (!ack) {
        return false;
    }
    gyro->gyroADCRaw[X] = (int16_t)((data[0] << 8) | data[1]);
    gyro->gyroADCRaw[Y] = (int16_t)((data[2] << 8) | data[3]);
    gyro->gyroADCRaw[Z] = (int16_t)((data[4] << 8) | data[5]);
    return true;
}

FAST_CODE bool mpuGyroReadSPI(gyroDev_t *gyro)
{
#ifdef GYRO_USES_SPI
    int16_t *gyroData = (int16_t *)gyro->dev.rxBuf;
    switch (gyro->gyroModeSPI) {
    case GYRO_EXTI_INIT:
    {
        memset(gyro->dev.txBuf, 0xff, 16);
        gyro->gyroDmaMaxDuration = 5; // seed estimate in CPU cycles; updated by mpuIntCallback with actual measurements
#if defined(MPU_INT_EXTI)
        if (gyro->detectedEXTI > GYRO_EXTI_DETECT_THRESHOLD) {
            if (spiUseDMA(&gyro->dev)) {
                gyro->dev.callbackArg = (uint32_t)gyro;
                gyro->dev.txBuf[0] = gyro->accDataReg | 0x80;
                gyro->segments[0].len = gyro->gyroDataReg - gyro->accDataReg + sizeof(uint8_t) + 3 * sizeof(int16_t);
                gyro->segments[0].callback = mpuIntCallback;
                gyro->segments[0].u.buffers.txData = gyro->dev.txBuf;
                gyro->segments[0].u.buffers.rxData = &gyro->dev.rxBuf[1];
                gyro->segments[0].negateCS = true;
                gyro->segments[1].len = 0;
                gyro->segments[1].u.link.dev = NULL;
                gyro->segments[1].u.link.segments = NULL;
                gyro->gyroModeSPI = GYRO_EXTI_INT_DMA;
            } else {
                gyro->gyroModeSPI = GYRO_EXTI_INT;
            }
        } else {
            gyro->gyroModeSPI = GYRO_EXTI_NO_INT;
        }
#else
        gyro->gyroModeSPI = GYRO_EXTI_NO_INT;
#endif
        break;
    }

    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        gyro->dev.txBuf[0] = gyro->gyroDataReg | 0x80;

        busSegment_t segments[] = {
            {.u.buffers = {NULL, NULL}, 7, true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = gyro->dev.txBuf;
        segments[0].u.buffers.rxData = &gyro->dev.rxBuf[1];

        spiSequence(&gyro->dev, &segments[0]);
        spiWait(&gyro->dev);

        gyro->gyroADCRaw[X] = __builtin_bswap16(gyroData[1]);
        gyro->gyroADCRaw[Y] = __builtin_bswap16(gyroData[2]);
        gyro->gyroADCRaw[Z] = __builtin_bswap16(gyroData[3]);
        break;
    }

    case GYRO_EXTI_INT_DMA:
    {
        // Data was read by DMA from EXTI interrupt; acc and gyro may not be contiguous
        const uint8_t gyroDataIndex = ((gyro->gyroDataReg - gyro->accDataReg) >> 1) + 1;
        gyro->gyroADCRaw[X] = __builtin_bswap16(gyroData[gyroDataIndex]);
        gyro->gyroADCRaw[Y] = __builtin_bswap16(gyroData[gyroDataIndex + 1]);
        gyro->gyroADCRaw[Z] = __builtin_bswap16(gyroData[gyroDataIndex + 2]);
        break;
    }

    default:
        break;
    }

    return true;
#else
    // I2C gyro path (e.g. CRAZYFLIE2): spiSequence not available
    static const uint8_t dataToSend[7] = {MPU_RA_GYRO_XOUT_H | 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    static uint8_t data[7];
    const bool ack = spiReadWriteBufRB(&gyro->dev, (uint8_t *)dataToSend, data, 7);
    if (!ack) {
        return false;
    }
    gyro->gyroADCRaw[X] = (int16_t)((data[1] << 8) | data[2]);
    gyro->gyroADCRaw[Y] = (int16_t)((data[3] << 8) | data[4]);
    gyro->gyroADCRaw[Z] = (int16_t)((data[5] << 8) | data[6]);
    return true;
#endif
}

#ifdef USE_SPI
static bool detectSPISensorsAndUpdateDetectionResult(gyroDev_t *gyro) {
    UNUSED(gyro); // since there are FCs which have gyro on I2C but other devices on SPI
    uint8_t sensor = MPU_NONE;
    UNUSED(sensor);

    // Spin until system uptime reaches GYRO_SPI_STARTUP_MS; no-op if already past that point.
    // Do this once here rather than in each detection routine to speed boot
    while (millis() < GYRO_SPI_STARTUP_MS);

    // note, when USE_DUAL_GYRO is enabled the gyro->dev must already be initialised.
#ifdef USE_GYRO_SPI_MPU6000
#ifndef USE_DUAL_GYRO
    spiSetBusInstance(&gyro->dev, SPI_DEV_TO_CFG(MPU6000_SPI_BUS));
#endif
#ifdef MPU6000_CS_PIN
    gyro->dev.busType_u.spi.csnPin = gyro->dev.busType_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(MPU6000_CS_PIN)) : gyro->dev.busType_u.spi.csnPin;
#endif
    sensor = mpu6000SpiDetect(&gyro->dev);
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }
#endif
#ifdef USE_GYRO_SPI_MPU6500
#ifndef USE_DUAL_GYRO
    spiSetBusInstance(&gyro->dev, SPI_DEV_TO_CFG(MPU6500_SPI_BUS));
#endif
#ifdef MPU6500_CS_PIN
    gyro->dev.busType_u.spi.csnPin = gyro->dev.busType_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(MPU6500_CS_PIN)) : gyro->dev.busType_u.spi.csnPin;
#endif
    sensor = mpu6500SpiDetect(&gyro->dev);
    // some targets using MPU_9250_SPI, ICM_20608_SPI or ICM_20602_SPI state sensor is MPU_65xx_SPI
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }
#endif
#ifdef USE_GYRO_IMUF9001
#ifdef IMUF9001_SPI_BUS
    spiSetBusInstance(&gyro->dev, SPI_DEV_TO_CFG(IMUF9001_SPI_BUS));
#else
#error IMUF9001 is SPI only
#endif
#ifdef IMUF9001_CS_PIN
    gyro->dev.busType_u.spi.csnPin = gyro->dev.busType_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(IMUF9001_CS_PIN)) : gyro->dev.busType_u.spi.csnPin;
#else
#error IMUF9001 must use a CS pin (IMUF9001_CS_PIN)
#endif
#ifdef IMUF9001_RST_PIN
    gyro->dev.busType_u.spi.rstPin = IOGetByTag(IO_TAG(IMUF9001_RST_PIN));
#else
#error IMUF9001 must use a RST pin (IMUF9001_RST_PIN)
#endif
    sensor = imuf9001SpiDetect(gyro);
    // some targets using MPU_9250_SPI, ICM_20608_SPI or ICM_20602_SPI state sensor is MPU_65xx_SPI
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }
#endif
#ifdef  USE_GYRO_SPI_MPU9250
#ifndef USE_DUAL_GYRO
    spiSetBusInstance(&gyro->dev, SPI_DEV_TO_CFG(MPU9250_SPI_BUS));
#endif
#ifdef MPU9250_CS_PIN
    gyro->dev.busType_u.spi.csnPin = gyro->dev.busType_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(MPU9250_CS_PIN)) : gyro->dev.busType_u.spi.csnPin;
#endif
    sensor = mpu9250SpiDetect(&gyro->dev);
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        gyro->mpuConfiguration.resetFn = mpu9250SpiResetGyro;
        return true;
    }
#endif
#ifdef USE_GYRO_SPI_ICM20649
#ifndef USE_DUAL_GYRO
#ifdef ICM20649_SPI_BUS
    spiSetBusInstance(&gyro->dev, SPI_DEV_TO_CFG(ICM20649_SPI_BUS));
#endif
#endif
#ifdef ICM20649_CS_PIN
    gyro->dev.busType_u.spi.csnPin = gyro->dev.busType_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(ICM20649_CS_PIN)) : gyro->dev.busType_u.spi.csnPin;
#endif
    sensor = icm20649SpiDetect(&gyro->dev);
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }
#endif
#ifdef USE_GYRO_SPI_ICM20689
#ifndef USE_DUAL_GYRO
    spiSetBusInstance(&gyro->dev, SPI_DEV_TO_CFG(ICM20689_SPI_BUS));
#endif
#ifdef ICM20689_CS_PIN
    gyro->dev.busType_u.spi.csnPin = gyro->dev.busType_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(ICM20689_CS_PIN)) : gyro->dev.busType_u.spi.csnPin;
#endif
    sensor = icm20689SpiDetect(&gyro->dev);
    // icm20689SpiDetect detects ICM20602 and ICM20689
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }
#endif
#ifdef USE_GYRO_SPI_ICM42605
#ifndef USE_DUAL_GYRO
#ifdef ICM42605_SPI_BUS
    spiSetBusInstance(&gyro->dev, SPI_DEV_TO_CFG(ICM42605_SPI_BUS));
#endif
#endif
#ifdef ICM42605_CS_PIN
    gyro->dev.busType_u.spi.csnPin = gyro->dev.busType_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(ICM42605_CS_PIN)) : gyro->dev.busType_u.spi.csnPin;
#endif
    sensor = icm426xxSpiDetect(&gyro->dev);
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }
#endif
#ifdef USE_GYRO_SPI_ICM42688P
#ifndef USE_DUAL_GYRO
#ifdef ICM42688P_SPI_BUS
    spiSetBusInstance(&gyro->dev, SPI_DEV_TO_CFG(ICM42688P_SPI_BUS));
#endif
#endif
#ifdef ICM42688P_CS_PIN
    gyro->dev.busType_u.spi.csnPin = gyro->dev.busType_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(ICM42688P_CS_PIN)) : gyro->dev.busType_u.spi.csnPin;
#endif
    sensor = icm426xxSpiDetect(&gyro->dev);
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }
#endif
#ifdef USE_ACCGYRO_BMI160
#ifndef USE_DUAL_GYRO
    spiSetBusInstance(&gyro->dev, SPI_DEV_TO_CFG(BMI160_SPI_BUS));
#endif
#ifdef BMI160_CS_PIN
    gyro->dev.busType_u.spi.csnPin = gyro->dev.busType_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(BMI160_CS_PIN)) : gyro->dev.busType_u.spi.csnPin;
#endif
    sensor = bmi160Detect(&gyro->dev);
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }
#endif
#ifdef USE_ACCGYRO_BMI270
#ifndef USE_DUAL_GYRO
    spiSetBusInstance(&gyro->dev, SPI_DEV_TO_CFG(BMI270_SPI_BUS));
#endif
#ifdef BMI270_CS_PIN
    gyro->dev.busType_u.spi.csnPin = gyro->dev.busType_u.spi.csnPin == IO_NONE ? IOGetByTag(IO_TAG(BMI270_CS_PIN)) : gyro->dev.busType_u.spi.csnPin;
#endif
    sensor = bmi270Detect(&gyro->dev);
    if (sensor != MPU_NONE) {
        gyro->mpuDetectionResult.sensor = sensor;
        return true;
    }
#endif
    return false;
}
#endif

void mpuDetect(gyroDev_t *gyro) {
    // MPU datasheet specifies 30ms.
    delay(35);
#if defined(USE_I2C)
    if (gyro->dev.bus == NULL || gyro->dev.bus->busType == BUS_TYPE_I2C) {
        i2cBusSetInstance(&gyro->dev, I2C_DEV_TO_CFG(MPU_I2C_INSTANCE));
        gyro->dev.busType_u.i2c.address = MPU_ADDRESS;
        uint8_t sig = 0;
        bool ack = busReadRegisterBuffer(&gyro->dev, MPU_RA_WHO_AM_I, &sig, 1);
        if (ack) {
            // If an MPU3050 is connected sig will contain 0.
            uint8_t inquiryResult;
            ack = busReadRegisterBuffer(&gyro->dev, MPU_RA_WHO_AM_I_LEGACY, &inquiryResult, 1);
            inquiryResult &= MPU_INQUIRY_MASK;
            if (ack && inquiryResult == MPUx0x0_WHO_AM_I_CONST) {
                gyro->mpuDetectionResult.sensor = MPU_3050;
                return;
            }
            sig &= MPU_INQUIRY_MASK;
            if (sig == MPUx0x0_WHO_AM_I_CONST) {
                gyro->mpuDetectionResult.sensor = MPU_60x0;
                mpu6050FindRevision(gyro);
            } else if (sig == MPU6500_WHO_AM_I_CONST) {
                gyro->mpuDetectionResult.sensor = MPU_65xx_I2C;
            }
            return;
        }
        // I2C probe failed; reset bus state so SPI detection starts clean.
        // Without this reset, i2cBusSetInstance will have written dev->busType_u.i2c.device
        // at union offset 0, aliasing dev->busType_u.spi.csnPin after field removal.
        gyro->dev.bus = NULL;
        memset(&gyro->dev.busType_u, 0, sizeof(gyro->dev.busType_u));
    }
#endif
#ifdef USE_SPI
    detectSPISensorsAndUpdateDetectionResult(gyro);
#endif
}

void mpuGyroInit(gyroDev_t *gyro) {
    gyro->accDataReg = MPU_RA_ACCEL_XOUT_H;
    gyro->gyroDataReg = MPU_RA_GYRO_XOUT_H;
#ifdef MPU_INT_EXTI
    mpuIntExtiInit(gyro);
#endif
}

uint8_t mpuGyroDLPF(gyroDev_t *gyro) {
    uint8_t ret;
    if (gyro->gyroRateKHz > GYRO_RATE_8_kHz) {
        ret = 0;  // If gyro is in 32KHz mode then the DLPF bits aren't used - set to 0
    } else {
        switch (gyro->hardware_lpf) {
        case GYRO_HARDWARE_LPF_NORMAL:
            ret = 0;
            break;
        case GYRO_HARDWARE_LPF_EXPERIMENTAL:
            ret = 7;
            break;
        case GYRO_HARDWARE_LPF_1KHZ_SAMPLE:
            ret = 1;
            break;
        default:
            ret = 0;
            break;
        }
    }
    return ret;
}

uint8_t mpuGyroFCHOICE(gyroDev_t *gyro) {
    if (gyro->gyroRateKHz > GYRO_RATE_8_kHz) {
        if (gyro->hardware_32khz_lpf == GYRO_32KHZ_HARDWARE_LPF_EXPERIMENTAL) {
            return FCB_8800_32;
        } else {
            return FCB_3600_32;
        }
    } else {
        return FCB_DISABLED;  // Not in 32KHz mode, set FCHOICE to select 8KHz sampling
    }
}

#ifdef USE_GYRO_REGISTER_DUMP
uint8_t mpuGyroReadRegister(const extDevice_t *dev, uint8_t reg) {
    uint8_t data;
    const bool ack = busReadRegisterBuffer(dev, reg, &data, 1);
    if (ack) {
        return data;
    } else {
        return 0;
    }
}
#endif
