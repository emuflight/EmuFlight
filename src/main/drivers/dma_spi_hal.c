#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"
#include "dma_spi.h"
#include "common/time.h"
#include "sensors/gyro.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#ifdef USE_GYRO_IMUF9001
#include "drivers/accgyro/accgyro_imuf9001.h"
FAST_RAM_ZERO_INIT volatile uint32_t crcErrorCount;
#endif

FAST_RAM_ZERO_INIT SPI_HandleTypeDef dmaSpiHandle;
FAST_RAM_ZERO_INIT DMA_HandleTypeDef SpiRxDmaHandle;
FAST_RAM_ZERO_INIT DMA_HandleTypeDef SpiTxDmaHandle;
FAST_RAM_ZERO_INIT volatile dma_spi_read_status_t dmaSpiReadStatus;
FAST_RAM_ZERO_INIT volatile bool dmaSpiDeviceDataReady = false;
FAST_RAM_ZERO_INIT uint8_t dmaTxBuffer[58];
FAST_RAM_ZERO_INIT uint8_t dmaRxBuffer[58];


FAST_CODE static inline void dmaSpiCsLo(void) {
    HAL_GPIO_WritePin(DMA_SPI_NSS_PORT, DMA_SPI_NSS_PIN, 0);
}

FAST_CODE static inline void dmaSpiCsHi(void) {
    HAL_GPIO_WritePin(DMA_SPI_NSS_PORT, DMA_SPI_NSS_PIN, 1);
}

FAST_CODE_NOINLINE void DMA_SPI_TX_DMA_HANDLER(void) {
    HAL_NVIC_ClearPendingIRQ(DMA_SPI_TX_DMA_IRQn);
    HAL_DMA_IRQHandler(&SpiTxDmaHandle);
}

FAST_CODE_NOINLINE void DMA_SPI_RX_DMA_HANDLER(void) {
    HAL_NVIC_ClearPendingIRQ(DMA_SPI_RX_DMA_IRQn);
    HAL_DMA_IRQHandler(&SpiRxDmaHandle);
    if (HAL_DMA_GetState(&SpiRxDmaHandle) == HAL_DMA_STATE_READY) {
        dmaSpiCsHi();
        //spi rx dma callback
#ifdef USE_GYRO_IMUF9001
        volatile uint32_t crc1 = ( (*(uint32_t *)(dmaRxBuffer + gyroConfig()->imuf_mode - 4)) & 0xFF );
        volatile uint32_t crc2 = ( getCrcImuf9001((uint32_t *)(dmaRxBuffer), (gyroConfig()->imuf_mode >> 2) - 1) & 0xFF );
        if(crc1 == crc2) {
            if(dmaSpiReadStatus != DMA_SPI_BLOCKING_READ_IN_PROGRESS) {
                gyroDmaSpiFinishRead();
            }
            dmaSpiDeviceDataReady = true;
        } else {
            if (crcErrorCount > 100000) {
                crcErrorCount = 0;
            }
            //error handler
            crcErrorCount++; //check every so often and cause a failsafe is this number is above a certain ammount
        }
#else
        if(dmaSpiReadStatus != DMA_SPI_BLOCKING_READ_IN_PROGRESS) {
            gyroDmaSpiFinishRead();
        }
        dmaSpiDeviceDataReady = true;
#endif
    }
    dmaSpiCsHi();
    dmaSpiReadStatus = DMA_SPI_READ_DONE;
}

FAST_CODE inline bool isDmaSpiDataReady(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs) {
    (void)(currentTimeUs);
    (void)(currentDeltaTimeUs);
    return dmaSpiDeviceDataReady;
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
    GPIO_InitTypeDef GPIO_InitStruct;
    DMA_SPI_CLOCK_INIT_FUNC;
    HAL_GPIO_WritePin(DMA_SPI_NSS_PORT, DMA_SPI_NSS_PIN, 1);
    HAL_GPIO_DeInit(DMA_SPI_NSS_PORT,  DMA_SPI_NSS_PIN);
    GPIO_InitStruct.Pin   = DMA_SPI_NSS_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DMA_SPI_NSS_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(DMA_SPI_NSS_PORT, DMA_SPI_NSS_PIN, 1);
    HAL_GPIO_DeInit(DMA_SPI_SCK_PORT,  DMA_SPI_SCK_PIN);
    HAL_GPIO_DeInit(DMA_SPI_MISO_PORT, DMA_SPI_MISO_PIN);
    HAL_GPIO_DeInit(DMA_SPI_MOSI_PORT, DMA_SPI_MOSI_PIN);
    GPIO_InitStruct.Pin       = DMA_SPI_SCK_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = DMA_SPI_SCK_AF;
    HAL_GPIO_Init(DMA_SPI_SCK_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin       = DMA_SPI_MISO_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = DMA_SPI_MISO_AF;
    HAL_GPIO_Init(DMA_SPI_MISO_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin       = DMA_SPI_MOSI_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = DMA_SPI_MOSI_AF;
    HAL_GPIO_Init(DMA_SPI_MOSI_PORT, &GPIO_InitStruct);
    SpiRxDmaHandle.Instance                 = DMA_SPI_RX_DMA_STREAM;
    SpiRxDmaHandle.Init.Channel             = DMA_SPI_RX_DMA_CHANNEL;
    SpiRxDmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    SpiRxDmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
    SpiRxDmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
    SpiRxDmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    SpiRxDmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    SpiRxDmaHandle.Init.Mode                = DMA_NORMAL;
    SpiRxDmaHandle.Init.Priority            = DMA_PRIORITY_HIGH;
    SpiRxDmaHandle.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    HAL_DMA_UnRegisterCallback(&SpiRxDmaHandle, HAL_DMA_XFER_ALL_CB_ID);
    HAL_DMA_Init(&SpiRxDmaHandle);
    __HAL_LINKDMA(hspi, hdmarx, SpiRxDmaHandle);
    HAL_NVIC_SetPriority(DMA_SPI_RX_DMA_IRQn, DMA_SPI_DMA_RX_PRE_PRI, DMA_SPI_DMA_RX_SUB_PRI);
    HAL_NVIC_EnableIRQ(DMA_SPI_RX_DMA_IRQn);
    SpiTxDmaHandle.Instance                 = DMA_SPI_TX_DMA_STREAM;
    SpiTxDmaHandle.Init.Channel             = DMA_SPI_TX_DMA_CHANNEL;
    SpiTxDmaHandle.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    SpiTxDmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
    SpiTxDmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
    SpiTxDmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    SpiTxDmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    SpiTxDmaHandle.Init.Mode                = DMA_NORMAL;
    SpiTxDmaHandle.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    SpiTxDmaHandle.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    HAL_DMA_UnRegisterCallback(&SpiTxDmaHandle, HAL_DMA_XFER_ALL_CB_ID);
    HAL_DMA_Init(&SpiTxDmaHandle);
    __HAL_LINKDMA(hspi, hdmatx, SpiTxDmaHandle);
    HAL_NVIC_SetPriority(DMA_SPI_TX_DMA_IRQn, DMA_SPI_DMA_TX_PRE_PRI, DMA_SPI_DMA_TX_SUB_PRI);
    HAL_NVIC_EnableIRQ(DMA_SPI_TX_DMA_IRQn);
}

void dmaSpiInit(void) {
    dmaSpiHandle.Instance               = DMA_SPI_SPI;
    HAL_SPI_DeInit(&dmaSpiHandle);
    dmaSpiHandle.Init.Mode              = SPI_MODE_MASTER;
    dmaSpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
    dmaSpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
    dmaSpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
    dmaSpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
    dmaSpiHandle.Init.NSS               = SPI_NSS_SOFT;
    dmaSpiHandle.Init.BaudRatePrescaler = DMA_SPI_BAUDRATE;
    dmaSpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    dmaSpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
    dmaSpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    dmaSpiHandle.Init.CRCPolynomial     = 7;
    HAL_SPI_Init(&dmaSpiHandle);
}

FAST_CODE_NOINLINE void dmaSpiTransmitReceive(uint8_t* txBuffer, uint8_t* rxBuffer, uint32_t size, uint32_t blockingRead) {
    if (HAL_SPI_GetState(&dmaSpiHandle) == HAL_SPI_STATE_READY) {
        dmaSpiCsLo();
        if(!blockingRead) {
            dmaSpiReadStatus = DMA_SPI_READ_IN_PROGRESS;
            HAL_SPI_TransmitReceive_DMA(&dmaSpiHandle, txBuffer, rxBuffer, size);
        } else {
            dmaSpiReadStatus = DMA_SPI_BLOCKING_READ_IN_PROGRESS;
            HAL_SPI_TransmitReceive(&dmaSpiHandle, txBuffer, rxBuffer, size, 40);
            dmaSpiCsHi();
            dmaSpiReadStatus = DMA_SPI_READ_DONE;
        }
    }
}
