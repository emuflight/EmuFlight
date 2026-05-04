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

#include "platform.h"

#include "common/utils.h"

#include "drivers/bus_spi.h"
#include "drivers/dma_reqmap.h"

// SPI peripheral → DMA stream/channel mapping for F4/F7.
// Guarded by USE_SPI so the table is always non-empty when compiled.
// Other MCU families or targets without USE_SPI fall through to the NULL stub.

#if (defined(STM32F4) || defined(STM32F7)) && defined(USE_SPI)

typedef struct dmaPeripheralMapping_s {
    dmaPeripheral_e  device;
    uint8_t          index;
    dmaChannelSpec_t channelSpec[MAX_PERIPHERAL_DMA_OPTIONS];
} dmaPeripheralMapping_t;

#if defined(STM32F4)
#define DMA(d, s, c) { DMA_CODE(d, s, c), (dmaResource_t *)DMA ## d ## _Stream ## s, DMA_Channel_ ## c }
#elif defined(STM32F7)
#define DMA(d, s, c) { DMA_CODE(d, s, c), (dmaResource_t *)DMA ## d ## _Stream ## s, DMA_CHANNEL_ ## c }
#endif

static const dmaPeripheralMapping_t dmaPeripheralMapping[] = {
    // SPI1: opt 0 preference differs between F745/F746/F765 and other F4/F7 variants.
#if defined(STM32F745xx) || defined(STM32F746xx) || defined(STM32F765xx)
    { DMA_PERIPH_SPI_SDO, SPIDEV_1, { DMA(2, 5, 3), DMA(2, 3, 3) } },
    { DMA_PERIPH_SPI_SDI, SPIDEV_1, { DMA(2, 2, 3), DMA(2, 0, 3) } },
#else
    { DMA_PERIPH_SPI_SDO, SPIDEV_1, { DMA(2, 3, 3), DMA(2, 5, 3) } },
    { DMA_PERIPH_SPI_SDI, SPIDEV_1, { DMA(2, 0, 3), DMA(2, 2, 3) } },
#endif
    { DMA_PERIPH_SPI_SDO, SPIDEV_2, { DMA(1, 4, 0) } },
    { DMA_PERIPH_SPI_SDI, SPIDEV_2, { DMA(1, 3, 0) } },
    { DMA_PERIPH_SPI_SDO, SPIDEV_3, { DMA(1, 5, 0), DMA(1, 7, 0) } },
    { DMA_PERIPH_SPI_SDI, SPIDEV_3, { DMA(1, 0, 0), DMA(1, 2, 0) } },

#if defined(STM32F411xE) || defined(STM32F745xx) || defined(STM32F746xx) || defined(STM32F765xx) || defined(STM32F722xx)
    { DMA_PERIPH_SPI_SDO, SPIDEV_4, { DMA(2, 1, 4), DMA(2, 4, 5) } },
    { DMA_PERIPH_SPI_SDI, SPIDEV_4, { DMA(2, 0, 4), DMA(2, 3, 5) } },
#endif
};

#undef DMA

const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e device, uint8_t index, int8_t opt)
{
    if (opt < 0 || opt >= MAX_PERIPHERAL_DMA_OPTIONS) {
        return NULL;
    }

    for (unsigned i = 0; i < ARRAYLEN(dmaPeripheralMapping); i++) {
        const dmaPeripheralMapping_t *periph = &dmaPeripheralMapping[i];
        if (periph->device == device && periph->index == index && periph->channelSpec[opt].ref) {
            return &periph->channelSpec[opt];
        }
    }

    return NULL;
}

#else  // No SPI, F1/F3, or other unsupported MCU family → no DMA for SPI.

const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e device, uint8_t index, int8_t opt)
{
    UNUSED(device);
    UNUSED(index);
    UNUSED(opt);
    return NULL;
}

#endif  // (STM32F4 || STM32F7) && USE_SPI


// Timer-DMA spec functions are not used in EF's SPI refactor.
// Stubs satisfy the declared interface; full implementation deferred.

dmaoptValue_t dmaoptByTag(ioTag_t ioTag)
{
    UNUSED(ioTag);
    return DMA_OPT_UNUSED;
}

const dmaChannelSpec_t *dmaGetChannelSpecByTimerValue(TIM_TypeDef *tim, uint8_t channel, dmaoptValue_t dmaopt)
{
    UNUSED(tim);
    UNUSED(channel);
    UNUSED(dmaopt);
    return NULL;
}

const dmaChannelSpec_t *dmaGetChannelSpecByTimer(const struct timerHardware_s *timer)
{
    UNUSED(timer);
    return NULL;
}

dmaoptValue_t dmaGetOptionByTimer(const struct timerHardware_s *timer)
{
    UNUSED(timer);
    return DMA_OPT_UNUSED;
}
