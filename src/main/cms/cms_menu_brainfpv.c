/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

// Menu contents for PID, RATES, RC preview, misc
// Should be part of the relevant .c file.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#ifdef USE_CMS

#include "build/version.h"

#include "drivers/system.h"

//#include "common/typeconversion.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_imu.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"
#include "config/feature.h"

#include "brainfpv/video.h"
#include "brainfpv/osd_utils.h"
#include "brainfpv/ir_transponder.h"
#include "brainfpv/brainfpv_osd.h"

bfOsdConfig_t bfOsdConfigCms;

static long menuBrainFPVOnEnter(void)
{
    memcpy(&bfOsdConfigCms, bfOsdConfig(), sizeof(bfOsdConfig_t));
    return 0;
}

static long menuBrainFPVOnExit(const OSD_Entry *self)
{
    UNUSED(self);

    memcpy(bfOsdConfigMutable(), &bfOsdConfigCms, sizeof(bfOsdConfig_t));
    return 0;
}

OSD_UINT8_t entryAhiSteps =  {&bfOsdConfigCms.ahi_steps, 0, 9, 1};
const char *STICKS_DISPLAY_NAMES[] = {"OFF", "MODE2", "MODE1"};
OSD_TAB_t entrySticksDisplay = {&bfOsdConfigCms.sticks_display, 2, &STICKS_DISPLAY_NAMES[0]};
const char *FONT_NAMES[] = {"DEFAULT", "LARGE", "BOLD"};
OSD_TAB_t entryOSDFont = {&bfOsdConfigCms.font, 2, &FONT_NAMES[0]};
OSD_UINT8_t entryWhiteLevel =  {&bfOsdConfigCms.white_level, 100, 120, 1};
OSD_UINT8_t entryBlackLevel =  {&bfOsdConfigCms.black_level, 15, 40, 1};
OSD_UINT8_t entrySyncTh =  {&bfOsdConfigCms.sync_threshold, BRAINFPV_OSD_SYNC_TH_MIN, BRAINFPV_OSD_SYNC_TH_MAX, 1};
OSD_INT8_t entryXoff =  {&bfOsdConfigCms.x_offset, -8, 7, 1};
OSD_UINT8_t entryXScale =  {&bfOsdConfigCms.x_scale, 0, 15, 1};
OSD_UINT8_t entry3DShift =  {&bfOsdConfigCms.sbs_3d_right_eye_offset, 10, 40, 1};
OSD_UINT16_t entryMapMaxDist =  {&bfOsdConfigCms.map_max_dist_m, 10, 32767, 10};

OSD_Entry cmsx_menuBrainFPVOsdEntries[] =
{
    {"-- BRAIN OSD --------", OME_Label, NULL, NULL, 0},
    {"AHI STEPS", OME_UINT8, NULL, &entryAhiSteps, 0},
    {"ALTITUDE SCALE", OME_Bool, NULL, &bfOsdConfigCms.altitude_scale, 0},
    {"SPEED SCALE", OME_Bool, NULL, &bfOsdConfigCms.speed_scale, 0},
    {"MAP", OME_Bool, NULL, &bfOsdConfigCms.map, 0},
    {"MAP MAX DIST M", OME_UINT16, NULL, &entryMapMaxDist, 0},
    {"SHOW STICKS", OME_TAB, NULL, &entrySticksDisplay, 0},
    {"FONT", OME_TAB, NULL, &entryOSDFont, 0},
    {"OSD WHITE", OME_UINT8, NULL, &entryWhiteLevel, 0},
    {"OSD BLACK", OME_UINT8, NULL, &entryBlackLevel, 0},
    {"INVERT", OME_Bool, NULL, &bfOsdConfigCms.invert, 0},
    {"OSD SYNC TH", OME_UINT8, NULL, &entrySyncTh, 0},
    {"OSD X OFF", OME_INT8, NULL, &entryXoff, 0},
    {"OSD X SC", OME_UINT8, NULL, &entryXScale, 0},
    {"3D MODE", OME_Bool, NULL, &bfOsdConfigCms.sbs_3d_enabled, 0},
    {"3D R SHIFT", OME_UINT8, NULL, &entry3DShift, 0},

    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuBrainFPVOsd = {
    .onEnter = NULL,
    .onExit = NULL,
    .entries = cmsx_menuBrainFPVOsdEntries,
};

const char * HD_FRAME_NAMES[] = {"OFF", "FULL", "CORNER"};
OSD_TAB_t entryHdFrameMode = {&bfOsdConfigCms.hd_frame, 2, &HD_FRAME_NAMES[0]};
OSD_UINT8_t entryHdFrameWidth = {&bfOsdConfigCms.hd_frame_width, 20, 255, 1};
OSD_UINT8_t entryHdFrameHeight = {&bfOsdConfigCms.hd_frame_height, 20, 255, 1};
OSD_INT8_t entryHdFrameHOffset = {&bfOsdConfigCms.hd_frame_h_offset, -100, 100, 1};
OSD_INT8_t entryHdFrameVOffset = {&bfOsdConfigCms.hd_frame_v_offset, -100, 100, 1};

OSD_Entry cmsx_menuBrainFPVHdFrameEntries[] =
{
    {"-- HD CAM FRAME --", OME_Label, NULL, NULL, 0},
    {"MODE", OME_TAB, NULL, &entryHdFrameMode, 0},
    {"WIDTH", OME_UINT8, NULL, &entryHdFrameWidth, 0},
    {"HEIGHT", OME_UINT8, NULL, &entryHdFrameHeight, 0},
    {"H OFFSET", OME_INT8, NULL, &entryHdFrameHOffset, 0},
    {"V OFFSET", OME_INT8, NULL, &entryHdFrameVOffset, 0},

    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuBrainFPVHdFrame = {
    .onEnter = NULL,
    .onExit = NULL,
    .entries = cmsx_menuBrainFPVHdFrameEntries,
};


const char * IR_NAMES[] = {"OFF", "I-LAP", "TRACKMATE"};
OSD_TAB_t entryIRSys = {&bfOsdConfigCms.ir_system, 2, &IR_NAMES[0]};
OSD_UINT32_t entryIRIlap =  {&bfOsdConfigCms.ir_ilap_id, 0, 9999999, 1};
OSD_UINT16_t entryIRTrackmate =  {&bfOsdConfigCms.ir_trackmate_id, 0, 4095, 1};

OSD_Entry cmsx_menuBrainFPVIrEntries[] =
{
    {"-- IR TRANSPONDER --", OME_Label, NULL, NULL, 0},
    {"IR SYS", OME_TAB, NULL, &entryIRSys, 0},
    {"I LAP ID", OME_UINT32, NULL, &entryIRIlap, 0},
    {"TRACKMATE ID", OME_UINT16, NULL, &entryIRTrackmate, 0},

    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuBrainFPVIr = {
    .onEnter = NULL,
    .onExit = NULL,
    .entries = cmsx_menuBrainFPVIrEntries,
};

const char * CRSF_OPT_NAMES[] = {"NO", "WHEN LQ LOW", "WHEN SNR LOW", "YES"};
OSD_TAB_t entryCrsfRssiMode = {&bfOsdConfigCms.crsf_link_stats_rssi, 3, &CRSF_OPT_NAMES[0]};
OSD_TAB_t entryCrsfSnrMode = {&bfOsdConfigCms.crsf_link_stats_snr, 3, &CRSF_OPT_NAMES[0]};
OSD_INT8_t entryCrsfSnrThreshold = {&bfOsdConfigCms.crsf_link_stats_snr_threshold, -10, 10, 1};

OSD_Entry cmsx_menuBrainFPVCrsfLinkEntries[] =
{
    {"-- CRSF LINK QUALITY --", OME_Label, NULL, NULL, 0},

    {"ENABLED", OME_Bool, NULL, &bfOsdConfigCms.crsf_link_stats, 0},
    {"TX POWER", OME_Bool, NULL, &bfOsdConfigCms.crsf_link_stats_power, 0},
    {"RSSI", OME_TAB, NULL, &entryCrsfRssiMode, 0},
    {"SNR", OME_TAB, NULL, &entryCrsfSnrMode, 0},
    {"SNR THRESHOLD", OME_INT8, NULL, &entryCrsfSnrThreshold, 0},

    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuBrainFPVCrsfLink = {
    .onEnter = NULL,
    .onExit = NULL,
    .entries = cmsx_menuBrainFPVCrsfLinkEntries,
};

OSD_Entry cmsx_menuBrainFPVEntires[] =
{
    {"--- BRAINFPV ---", OME_Label, NULL, NULL, 0},
    {"BRAIN OSD", OME_Submenu, cmsMenuChange, &cmsx_menuBrainFPVOsd, 0},
    {"HD FRAME", OME_Submenu, cmsMenuChange, &cmsx_menuBrainFPVHdFrame, 0},
    {"CRSF LINK QUALITY", OME_Submenu, cmsMenuChange, &cmsx_menuBrainFPVCrsfLink, 0},

    {"IR TRANSPONDER", OME_Submenu, cmsMenuChange, &cmsx_menuBrainFPVIr, 0},
#if defined(USE_BRAINFPV_SPECTROGRAPH)
    {"SPECTROGRAPH", OME_Bool, NULL, &bfOsdConfigCms.spec_enabled, 0},
#endif /* defined(USE_BRAINFPV_SPECTROGRAPH) */
    {"SHOW LOGO ON ARM", OME_Bool, NULL, &bfOsdConfigCms.show_logo_on_arm, 0},
    {"SHOW PILOT LOGO", OME_Bool, NULL, &bfOsdConfigCms.show_pilot_logo, 0},
    {"BACK", OME_Back, NULL, NULL, 0},
    {NULL, OME_END, NULL, NULL, 0}
};

CMS_Menu cmsx_menuBrainFPV = {
    .onEnter = menuBrainFPVOnEnter,
    .onExit = menuBrainFPVOnExit,
    .entries = cmsx_menuBrainFPVEntires,
};
#endif // CMS
