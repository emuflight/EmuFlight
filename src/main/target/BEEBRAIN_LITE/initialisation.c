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

#include "config/feature.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"

#include "fc/config.h"

void targetPreInit(void) {
    IO_t mcoPin = IOGetByTag(IO_TAG(PA8));
    IOInit(mcoPin, OWNER_SYSTEM, 1);
    IOConfigGPIOAF(mcoPin, IO_CONFIG(GPIO_Mode_AF,  GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_NOPULL), GPIO_AF_MCO);
    RCC_MCO1Config(RCC_MCO1Source_HSE, RCC_MCO1Div_5);
    spiPreinitCsByTag(IO_TAG(MPU6500_CS_PIN));
    // spiPreinitCsByTag(IO_TAG(RTC6705_CS_PIN));
    spiPreinitCsByTag(IO_TAG(BMP280_CS_PIN));
}
