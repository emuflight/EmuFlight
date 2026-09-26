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

// This file is copied with modifications from project Deviation,
// see http://deviationtx.com

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef USE_RX_SPI

#include "build/build_config.h"

#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/rcc.h"
#include "drivers/system.h"

#include "pg/rx_spi.h"

#include "rx_spi.h"

static extDevice_t rxSpiDevice;
static extDevice_t *dev = &rxSpiDevice;

bool rxSpiDeviceInit(const rxSpiConfig_t *rxSpiConfig) {
    if (!spiSetBusInstance(dev, rxSpiConfig->spibus)) {
        return false;
    }
    const IO_t rxCsPin = IOGetByTag(rxSpiConfig->csnTag);
    IOInit(rxCsPin, OWNER_RX_SPI_CS, 0);
    IOConfigGPIO(rxCsPin, SPI_IO_CS_CFG);
    dev->busType_u.spi.csnPin = rxCsPin;
    IOHi(rxCsPin);
    spiSetClkPhasePolarity(dev, true);
    spiSetClkDivisor(dev, SPI_CLOCK_STANDARD);
    return true;
}

uint8_t rxSpiTransferByte(uint8_t data) {
    return spiReadWrite(dev, data);
}

void rxSpiWriteByte(uint8_t data) {
    spiWrite(dev, data);
}

void rxSpiWriteCommand(uint8_t command, uint8_t data) {
    spiWriteReg(dev, command, data);
}

void rxSpiWriteCommandMulti(uint8_t command, const uint8_t *data, uint8_t length) {
    spiWriteRegBuf(dev, command, (uint8_t *)data, length);
}

uint8_t rxSpiReadCommand(uint8_t command, uint8_t data) {
    UNUSED(data);
    return spiReadReg(dev, command);
}

void rxSpiReadCommandMulti(uint8_t command, uint8_t commandData, uint8_t *retData, uint8_t length) {
    UNUSED(commandData);
    spiReadRegBuf(dev, command, retData, length);
}
#endif
