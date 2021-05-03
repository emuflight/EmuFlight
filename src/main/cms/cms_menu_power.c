/*
 * This file is part of Cleanflight and Betaflight and EmuFlight.
 *
 * Cleanflight and Betaflight and EmuFlight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight and EmuFlight are distributed in the hope that they
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
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef USE_CMS

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_power.h"

#include "config/feature.h"

#include "sensors/battery.h"
#include "sensors/current.h"
#include "sensors/voltage.h"

#include "config/config.h"

uint16_t batteryConfig_vbatmincellvoltage;
uint16_t batteryConfig_vbatmaxcellvoltage;
uint16_t batteryConfig_vbatwarningcellvoltage;

static const void *cmsx_Power_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    batteryConfig_vbatmincellvoltage = batteryConfig()->vbatmincellvoltage;
    batteryConfig_vbatmaxcellvoltage = batteryConfig()->vbatmaxcellvoltage;
    batteryConfig_vbatwarningcellvoltage = batteryConfig()->vbatwarningcellvoltage;

    return NULL;
}

static const void *cmsx_Power_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    batteryConfigMutable()->vbatmincellvoltage = batteryConfig_vbatmincellvoltage;
    batteryConfigMutable()->vbatmaxcellvoltage = batteryConfig_vbatmaxcellvoltage;
    batteryConfigMutable()->vbatwarningcellvoltage = batteryConfig_vbatwarningcellvoltage;

    return NULL;
}

static const OSD_Entry cmsx_menuPowerEntries[] =
{
    { "-- POWER --", OME_Label, NULL, NULL, 0},

    { "VBAT CLMIN", OME_UINT16, NULL, &(OSD_UINT16_t) { &batteryConfig_vbatmincellvoltage, VBAT_CELL_VOTAGE_RANGE_MIN, VBAT_CELL_VOTAGE_RANGE_MAX, 1 }, 0 },
    { "VBAT CLMAX", OME_UINT16, NULL, &(OSD_UINT16_t) { &batteryConfig_vbatmaxcellvoltage, VBAT_CELL_VOTAGE_RANGE_MIN, VBAT_CELL_VOTAGE_RANGE_MAX, 1 }, 0 },
    { "VBAT CLWARN", OME_UINT16, NULL, &(OSD_UINT16_t) { &batteryConfig_vbatwarningcellvoltage, VBAT_CELL_VOTAGE_RANGE_MIN, VBAT_CELL_VOTAGE_RANGE_MAX, 1 }, 0 },

    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

CMS_Menu cmsx_menuPower = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUPWR",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_Power_onEnter,
    .onExit = cmsx_Power_onExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuPowerEntries
};

#endif
