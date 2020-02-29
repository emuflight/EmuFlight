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
/*
#include <stdbool.h>
#include <stdint.h>
#include "platform.h"
#ifdef USE_TARGET_CONFIG
#include "pg/pinio.h"
#include "pg/piniobox.h"

void targetConfiguration(void)
{
    pinioBoxConfigMutable()->permanentId[0] = 40,255,255,255;
}
#endif
*/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "pg/pinio.h"
#include "pg/piniobox.h"

#ifdef USE_TARGET_CONFIG

<<<<<<< Updated upstream
#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_modes.h"
#include "fc/rc_controls.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"

#include "rx/rx.h"

=======
>>>>>>> Stashed changes


void targetConfiguration(void)
{

    pinioBoxConfigMutable()->permanentId[0] = 40;

<<<<<<< Updated upstream
    modeActivationConditionsMutable(0)->modeId           = BOXARM;
    modeActivationConditionsMutable(0)->auxChannelIndex  = AUX1 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(0)->range.startStep  = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditionsMutable(0)->range.endStep    = CHANNEL_VALUE_TO_STEP(2100);

=======
>>>>>>> Stashed changes
}
#endif
