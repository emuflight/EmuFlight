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

#ifdef USE_TARGET_CONFIG

#include "pg/pinio.h"
#include "pg/piniobox.h"
/*
#include "blackbox/blackbox.h"
#include "fc/rc_modes.h"
#include "common/axis.h"
#include "common/filter.h"
#include "config/feature.h"
#include "drivers/pwm_esc_detect.h"
#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "io/beeper.h"
#include "io/serial.h"
#include "pg/rx.h"
#include "rx/rx.h"
*/
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"


void targetConfiguration(void) {
    pinioConfigMutable()->config[1] = PINIO_CONFIG_OUT_INVERTED | PINIO_CONFIG_MODE_OUT_PP;
    pinioBoxConfigMutable()->permanentId[0] = 40;
#if defined (TRANSTECF411HD)
    boardAlignmentMutable()->rollDegrees = 180;
#endif
}
#endif
