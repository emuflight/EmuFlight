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

#include "platform.h"

#ifdef USE_PINIOBOX

#include "pg/pg_ids.h"
#include "piniobox.h"
#include "drivers/io.h"
#include "interface/msp_box.h"

PG_REGISTER_WITH_RESET_TEMPLATE(pinioBoxConfig_t, pinioBoxConfig, PG_PINIOBOX_CONFIG, 1);

// Default: all slots unassigned (PERMANENT_ID_NONE) — VTX/load-switch always ON.
// BF parity: BF's pgResetFn_pinioBoxConfig also always defaults to NONE regardless
// of PINIOx_BOX target defines. PINIOx_BOX in target.h is informational only —
// it names the recommended box for switch control (e.g. USER1=40 for pit mode)
// but is not applied automatically. Pilot configures it in the Modes tab if needed.
PG_RESET_TEMPLATE(pinioBoxConfig_t, pinioBoxConfig,
{ PERMANENT_ID_NONE, PERMANENT_ID_NONE, PERMANENT_ID_NONE, PERMANENT_ID_NONE }
                 );
#endif
