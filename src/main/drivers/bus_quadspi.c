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
 *
 * Author: Dominic Clifton
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_QUADSPI

#include "drivers/bus_quadspi.h"
#include "drivers/bus_quadspi_impl.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "rcc.h"

#include "pg/bus_quadspi.h"

quadSpiDevice_t quadSpiDevice[QUADSPIDEV_COUNT] = { 0 };

quadSpiDevice_e quadSpiDeviceByInstance(quadSpiResource_t *instance)
{
    if (!instance) {
        return QUADSPIINVALID;
    }
    for (size_t i = 0; i < QUADSPIDEV_COUNT; i++) {
        if (quadSpiHardware[i].reg == instance) {
            return quadSpiHardware[i].device;
        }
    }
    return QUADSPIINVALID;
}

quadSpiResource_t *quadSpiInstanceByDevice(quadSpiDevice_e device)
{
    if (device == QUADSPIINVALID || device >= QUADSPIDEV_COUNT) {
        return NULL;
    }

    return quadSpiDevice[device].dev;
}

bool quadSpiInit(quadSpiDevice_e device)
{
    switch (device) {
    case QUADSPIINVALID:
        return false;
    case QUADSPIDEV_1:
#ifdef USE_QUADSPI_DEVICE_1
        quadSpiInitDevice(device);
        return true;
#else
        break;
#endif
    }
    return false;
}

uint32_t quadSpiTimeoutUserCallback(quadSpiResource_t *instance)
{
    quadSpiDevice_e device = quadSpiDeviceByInstance(instance);
    if (device == QUADSPIINVALID) {
        return UINT32_MAX;
    }
    quadSpiDevice[device].errorCount++;
    return quadSpiDevice[device].errorCount;
}

uint16_t quadSpiGetErrorCounter(quadSpiResource_t *instance)
{
    quadSpiDevice_e device = quadSpiDeviceByInstance(instance);
    if (device == QUADSPIINVALID) {
        return 0;
    }
    return quadSpiDevice[device].errorCount;
}

void quadSpiResetErrorCounter(quadSpiResource_t *instance)
{
    quadSpiDevice_e device = quadSpiDeviceByInstance(instance);
    if (device != QUADSPIINVALID) {
        quadSpiDevice[device].errorCount = 0;
    }
}

// QSPI bus device array - consistent with SPI bus device pattern
busDevice_t quadSpiBusDevice[QUADSPIDEV_COUNT];

// Mark this bus as being QSPI and record the first owner to use it
bool quadSpiSetBusInstance(extDevice_t *dev, uint32_t device)
{
    if ((device == 0) || (device > QUADSPIDEV_COUNT)) {
        return false;
    }

    busDevice_t *bus = &quadSpiBusDevice[QUADSPI_CFG_TO_DEV(device)];

    // By default each device should use QSPI DMA if the bus supports it
    dev->useDMA = true;

    if (bus->busType == BUS_TYPE_QSPI) {
        // This bus has already been initialised — safe to assign now
        dev->bus = bus;
        bus->deviceCount++;
        return true;
    }

    bus->busType_u.qspi.instance = quadSpiDevice[QUADSPI_CFG_TO_DEV(device)].dev;

    if (bus->busType_u.qspi.instance == NULL) {
        return false;
    }

    dev->bus = bus;
    bus->busType = BUS_TYPE_QSPI;
    bus->deviceCount = 1;
    bus->curSegment = (volatile busSegment_t *)BUS_QSPI_FREE;
    quadSpiInitBusDMA(bus);

    return true;
}
#endif
