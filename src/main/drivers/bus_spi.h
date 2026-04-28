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

#pragma once

#include "drivers/bus.h"
#include "drivers/io_types.h"
#include "drivers/rcc_types.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#if defined(STM32F4) || defined(STM32F3)
#define SPI_IO_AF_CFG           IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#define SPI_IO_AF_SCK_CFG       IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_DOWN)
#define SPI_IO_AF_MISO_CFG      IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP)
#define SPI_IO_CS_CFG           IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL)
#elif defined(STM32F7)
#define SPI_IO_AF_CFG           IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)
#define SPI_IO_AF_SCK_CFG_HIGH  IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)
#define SPI_IO_AF_SCK_CFG_LOW   IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLDOWN)
#define SPI_IO_AF_MISO_CFG      IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLUP)
#define SPI_IO_CS_CFG           IO_CONFIG(GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL)
#elif defined(STM32F1)
#define SPI_IO_AF_SCK_CFG       IO_CONFIG(GPIO_Mode_AF_PP,       GPIO_Speed_50MHz)
#define SPI_IO_AF_MOSI_CFG      IO_CONFIG(GPIO_Mode_AF_PP,       GPIO_Speed_50MHz)
#define SPI_IO_AF_MISO_CFG      IO_CONFIG(GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz)
#define SPI_IO_CS_CFG           IO_CONFIG(GPIO_Mode_Out_PP,      GPIO_Speed_50MHz)
#endif

/*
  Flash M25p16 tolerates 20mhz, SPI_CLOCK_FAST should sit around 20 or less.
*/
typedef enum {
    SPI_CLOCK_INITIALIZATION = 256,
#if defined(STM32F4)
    SPI_CLOCK_SLOW          = 128, //00.65625 MHz
    SPI_CLOCK_STANDARD      = 8,   //10.50000 MHz
    SPI_CLOCK_FAST          = 4,   //21.00000 MHz
    SPI_CLOCK_ULTRAFAST     = 2    //42.00000 MHz
#elif defined(STM32F7)
    SPI_CLOCK_SLOW          = 256, //00.42188 MHz
    SPI_CLOCK_STANDARD      = 16,  //06.57500 MHz
    SPI_CLOCK_FAST          = 8,   //13.50000 MHz
    SPI_CLOCK_ULTRAFAST     = 2    //54.00000 MHz
#else
    SPI_CLOCK_SLOW          = 128, //00.56250 MHz
    SPI_CLOCK_STANDARD      = 4,   //09.00000 MHz
    SPI_CLOCK_FAST          = 2,   //18.00000 MHz
    SPI_CLOCK_ULTRAFAST     = 2    //18.00000 MHz
#endif
} SPIClockDivider_e;

typedef enum SPIDevice {
    SPIINVALID = -1,
    SPIDEV_1   = 0,
    SPIDEV_2,
    SPIDEV_3,
    SPIDEV_4
} SPIDevice;

#if defined(STM32F1)
#define SPIDEV_COUNT 2
#elif defined(STM32F3) || defined(STM32F4)
#define SPIDEV_COUNT 3
#elif defined(STM32F7)
#define SPIDEV_COUNT 4
#else
#define SPIDEV_COUNT 4

#endif

// Macros to convert between CLI bus number and SPIDevice.
#define SPI_CFG_TO_DEV(x)   ((x) - 1)
#define SPI_DEV_TO_CFG(x)   ((x) + 1)

// Size of SPI CS pre-initialization tag arrays
#define SPI_PREINIT_IPU_COUNT 11
#define SPI_PREINIT_OPU_COUNT 2

void spiPreinitCsByTag(ioTag_t iotag);
void spiPreinitCsByIO(IO_t io);
void spiPreInit(void);

bool spiInit(SPIDevice device);
void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor);
uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t data);
bool spiIsBusBusy(SPI_TypeDef *instance);

bool spiTransfer(SPI_TypeDef *instance, const uint8_t *txData, uint8_t *rxData, int len);

uint16_t spiGetErrorCounter(SPI_TypeDef *instance);
void spiResetErrorCounter(SPI_TypeDef *instance);
SPIDevice spiDeviceByInstance(SPI_TypeDef *instance);
SPI_TypeDef *spiInstanceByDevice(SPIDevice device);

// Bus-abstraction accessor. Returns a pointer to the shared busDevice_t that
// represents the given SPI peripheral (populated by spiInit()). Returns NULL
// for invalid devices.
busDevice_t *spiBusByDevice(SPIDevice device);

// Mark an extDevice_t as belonging to an SPI bus, using the 1-based CLI device id.
// Returns false if device is 0 (disabled), out of range, or the peripheral is absent.
bool spiSetBusInstance(extDevice_t *dev, uint32_t device);

// Called after all devices are initialised to enable SPI DMA where channels are available.
// Stage M.1: stub — DMA channel allocation requires dma_reqmap (Stage M.3).
void spiInitBusDMA(void);

// Determine the divisor / clock for a given frequency
uint16_t spiCalculateDivider(uint32_t freq);
uint32_t spiCalculateClock(uint16_t spiClkDivisor);

// Per-device clock and phase settings; hardware applied at spiSequenceStart time.
void spiSetClkDivisor(const extDevice_t *dev, uint16_t divider);
void spiSetClkPhasePolarity(const extDevice_t *dev, bool leadingEdge);

// Enable/disable DMA on a specific device. Enabled by default; a no-op in Stage M.1
// (DMA transfers are not yet active; bus->useDMA stays false until spiInitBusDMA).
void spiDmaEnable(const extDevice_t *dev, bool enable);

// Segment-based SPI API (matches BF 4.5-maintenance).
// Dispatches via spiSequenceStart: polled when bus->useDMA is false,
// DMA when enabled by spiInitBusDMA.
void spiSequence(const extDevice_t *dev, busSegment_t *segments);
void spiWait(const extDevice_t *dev);
void spiRelease(const extDevice_t *dev);
bool spiIsBusy(const extDevice_t *dev);
void spiLinkSegments(const extDevice_t *dev, busSegment_t *firstSegment, busSegment_t *secondSegment);

/*
 * Routine naming convention:
 *  spi[Read][Write][Reg][Msk][Buf][RB]
 *
 *  Read:      Perform a read, returning the value read unless 'Buf'
 *  Write:     Perform a write
 *  ReadWrite: Perform both, returning the value read unless 'Buf'
 *  Reg:       Register number 'reg' written first
 *  Msk:       Register OR'd with 0x80 (device signals read via bit 7)
 *  Buf:       Pass data of given length by reference
 *  RB:        Return false immediately if bus busy, else complete and return true
 */
uint8_t spiReadReg(const extDevice_t *dev, uint8_t reg);
uint8_t spiReadRegMsk(const extDevice_t *dev, uint8_t reg);
void spiReadRegBuf(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);
bool spiReadRegBufRB(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);
bool spiReadRegMskBufRB(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint8_t length);

void spiWrite(const extDevice_t *dev, uint8_t data);
void spiWriteReg(const extDevice_t *dev, uint8_t reg, uint8_t data);
bool spiWriteRegRB(const extDevice_t *dev, uint8_t reg, uint8_t data);
void spiWriteRegBuf(const extDevice_t *dev, uint8_t reg, uint8_t *data, uint32_t length);

uint8_t spiReadWrite(const extDevice_t *dev, uint8_t data);
uint8_t spiReadWriteReg(const extDevice_t *dev, uint8_t reg, uint8_t data);
void spiReadWriteBuf(const extDevice_t *dev, uint8_t *txData, uint8_t *rxData, int len);
bool spiReadWriteBufRB(const extDevice_t *dev, uint8_t *txData, uint8_t *rxData, int length);

bool spiUseDMA(const extDevice_t *dev);
bool spiUseSDO_DMA(const extDevice_t *dev);
void spiBusDeviceRegister(const extDevice_t *dev);
uint8_t spiGetRegisteredDeviceCount(void);
uint8_t spiGetExtDeviceCount(const extDevice_t *dev);

struct spiPinConfig_s;
void spiPinConfigure(const struct spiPinConfig_s *pConfig);
