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

#include <stdint.h>

#include "platform.h"

#ifdef USE_TARGET_CONFIG

#include "flight/mixer.h"
#include "io/osd.h"
#include "pg/pinio.h"
#include "pg/piniobox.h"
#include "target.h"

#include "config_helper.h"


void targetConfiguration(void)
{
    pinioConfigMutable()->config[0] = PINIO_CONFIG_OUT_INVERTED | PINIO_CONFIG_MODE_OUT_PP;
    pinioBoxConfigMutable()->permanentId[0] = BOXARM;

    osdConfigMutable()->item_pos[OSD_MAIN_BATT_VOLTAGE] = OSD_POS(1, 12) | VISIBLE_FLAG;
    osdConfigMutable()->item_pos[OSD_ALTITUDE] = OSD_POS(1, 11) | VISIBLE_FLAG;

    serialPortConfig_t *uart4Port = serialFindPortConfiguration(SERIAL_PORT_UART4);
    if (uart4Port) {
        uart4Port->functionMask = FUNCTION_ESC_SENSOR;
    }

    motorConfigMutable()->dev.motorPwmProtocol = PWM_TYPE_DSHOT600;
}

#endif
