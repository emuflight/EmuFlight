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

 // Ability to use mutlilanguage OSD made by QuickFlash
 // French translation by ...
 // other languages by ...

#pragma once

#define ENGLISH_OSD
//#define FRENCH_OSD
//#define GERMAN_OSD
//#define TAGALOG_OSD
//#define JAPANESE_OSD

#ifdef ENGLISH_OSD
// text from osd_elements.c
#define AG "ag" // shortened version of antigravity
#define CRAFTNAME "CRAFT_NAME"
#define DBG "DBG %5d %5d %5d %5d" // keep the same except for the DBG which is short for debug
#define DISARMED "DISARMED"
#define DISPLAYNAME "DISPLAY_NAME"
#define RATE "RATE_%u" // only change the RATE portion
#define FS "!FS!" // only change the FS part which is short for failsafe, keep 4 characters long which inlcude the ! !
#define RESC "RESC" // RESC is short for rescue, keep 4 characters long
#define HEAD "HEAD" // HEAD is short for headfree mode, keep 4 characters long
#define ANGL "ANGL" // short for angle mode, keep 4 characters long
#define HOR "HOR " // short for horizon mode, keep 4 characters long
#define NFE "NFE " // stands for NFE mode, keep 4 characters long
#define AIR "AIR " // short for airmode, keep 4 characters long
#define ACRO "ACRO" // short for acro mode, keep 4 characters long
#define PIT "PIT" // short for pitch, keep 3 characters long
#define ROL "ROL" // short for roll, keep 3 characters long
#define YA "YAW" // keep 3 characters long
#define BEACON_ON " BEACON ON" // keep the space and try to keep the same length
#define ARM_IN "ARM IN %d.%d" // only change the ARM IN keep the rest the same.
#define FAIL_SAFE "FAIL SAFE"
#define CRASH_FLIP "CRASH FLIP" // also known as turtle mode, keep naming consistent with crash flip
#define LAUNCH_ANGLE "LAUNCH %d" // only change the LAUNCH portion
#define LAUNCH "LAUNCH"
#define RSSI_LOW "RSSI LOW"
#define RSSI_DBM "RSSI DBM"
#define LINK_QUALITY "LINK QUALITY"
#define LAND_NOW " LAND NOW"
#define RESCUE_NA "RESCUE N/A"
#define RESCUE_OFF "RESCUE OFF"
#define HEADFREE "HEADFREE"
#define ACC_RECOVERY "ACC RECOVERY"
#define CORE "CORE %c: %3d%c" // only change the CORE portion
#define LOW_BATTERY "LOW BATTERY"
#define RCSMOOTHING "RCSMOOTHING"
#define OVER_CAP "OVER CAP" // short for over capacity, as in your battery capacity
#define BATT_LESS_FULL "BATT < FULL" // short for battery < full, keep the <
// text from osd.c
#define ON_TIME "ON TIME  " // keep the length the same
#define TOTAL_ARM "TOTAL ARM" // keep the length the same
#define LAST_ARM "LAST ARM " // keep the length the same
#define ON_ARM "ON/ARM   " // keep the length the same
#define FAULT "FAULT"
#define NO_RTC "NO RTC" // stands for no real time clock
#define MAX_ALTITUDE "MAX ALTITUDE"
#define MAX_SPEED "MAX SPEED"
#define MAX_DISTANCE "MAX DISTANCE"
#define FLIGHT_DISTANCE "FLIGHT DISTANCE"
#define MIN_BATTERY "MIN BATTERY"
#define END_BATTERY "END BATTERY"
#define BATTERY "BATTERY"
#define MIN_RSSI "MIN RSSI"
#define MAX_CURRENT "MAX CURRENT"
#define USED_MAH "USED MAH"
#define BLACKBOX "BLACKBOX"
#define BB_LOG_NUM "BB LOG NUM" // stands for blackbox log number
#define MAX_G_FORCE "MAX G-FORCE"
#define MAX_ESC_TEMP "MAX ESC TEMP"
#define MAX_ESC_RPM "MAX ESC RPM"
#define MIN_LINK "MIN LINK"
#define PEAK_FFT "PEAK FFT"
#define THRT_LESS_20 "THRT<20%" // short for throttle less than 20
#define MIN_RSSI_DBM "MIN RSSI DBM"
#define TOTAL_FLIGHTS "TOTAL FLIGHTS"
#define TOTAL_FLIGHT_TIME "TOTAL FLIGHT TIME"
#define TOTAL_DISTANCE "TOTAL DISTANCE"
#define STATS "  --- STATS ---" // keep the length the same
#define OSD_ARMED "ARMED"
// text from cms_menu_blackbox.c
// text from cms_menu_failsafe.c
// text from cms_menu_firmware.c
// text from cms_menu_gps_rescue.c
// text from cms_menu_imu.c
// text from cms_menu_ledstrip.c
// text from cms_menu_main.c
// text from cms_menu_misc.c
// text from cms_menu_osd.c
// text from cms_menu_persistent_stats.c
// text from cms_menu_power.c
// text from cms_menu_saveexit.c
// text from cms_menu_vtx_common.c
// text from cms_menu_vtx_rtc6705.c
// text from cms_menu_vtx_smartaudio.c
// text from cms_menu_vtx_tramp.c
// text from cms.c


#elif defined(FRENCH_OSD)
// text from osd_elements.c
#define AG "ag" // shortened version of antigravity
#define CRAFTNAME "CRAFT_NAME"
#define DBG "DBG %5d %5d %5d %5d" // keep the same except for the DBG which is short for debug
#define DISARMED "DISARMED"
#define DISPLAYNAME "DISPLAY_NAME"
#define RATE "RATE_%u" // only change the RATE portion
#define FS "!FS!" // only change the FS part which is short for failsafe, keep 4 characters long which inlcude the ! !
#define RESC "RESC" // RESC is short for rescue, keep 4 characters long
#define HEAD "HEAD" // HEAD is short for headfree mode, keep 4 characters long
#define ANGL "ANGL" // short for angle mode, keep 4 characters long
#define HOR "HOR " // short for horizon mode, keep 4 characters long
#define NFE "NFE " // stands for NFE mode, keep 4 characters long
#define AIR "AIR " // short for airmode, keep 4 characters long
#define ACRO "ACRO" // short for acro mode, keep 4 characters long
#define PIT "PIT" // short for pitch, keep 3 characters long
#define ROL "ROL" // short for roll, keep 3 characters long
#define YA "YAW" // keep 3 characters long
#define BEACON_ON " BEACON ON" // keep the space and try to keep the same length
#define ARM_IN "ARM IN %d.%d" // only change the ARM IN keep the rest the same.
#define FAIL_SAFE "FAIL SAFE"
#define CRASH_FLIP "CRASH FLIP" // also known as turtle mode, keep naming consistent with crash flip
#define LAUNCH_ANGLE "LAUNCH %d" // only change the LAUNCH portion
#define LAUNCH "LAUNCH"
#define RSSI_LOW "RSSI LOW"
#define RSSI_DBM "RSSI DBM"
#define LINK_QUALITY "LINK QUALITY"
#define LAND_NOW " LAND NOW"
#define RESCUE_NA "RESCUE N/A"
#define RESCUE_OFF "RESCUE OFF"
#define HEADFREE "HEADFREE"
#define ACC_RECOVERY "ACC RECOVERY"
#define CORE "CORE %c: %3d%c" // only change the CORE portion
#define LOW_BATTERY "LOW BATTERY"
#define RCSMOOTHING "RCSMOOTHING"
#define OVER_CAP "OVER CAP" // short for over capacity, as in your battery capacity
#define BATT_LESS_FULL "BATT < FULL" // short for battery < full, keep the <
// text from osd.c
#define ON_TIME "ON TIME  " // keep the length the same
#define TOTAL_ARM "TOTAL ARM" // keep the length the same
#define LAST_ARM "LAST ARM " // keep the length the same
#define ON_ARM "ON/ARM   " // keep the length the same
#define FAULT "FAULT"
#define NO_RTC "NO RTC" // stands for no real time clock
#define MAX_ALTITUDE "MAX ALTITUDE"
#define MAX_SPEED "MAX SPEED"
#define MAX_DISTANCE "MAX DISTANCE"
#define FLIGHT_DISTANCE "FLIGHT DISTANCE"
#define MIN_BATTERY "MIN BATTERY"
#define END_BATTERY "END BATTERY"
#define BATTERY "BATTERY"
#define MIN_RSSI "MIN RSSI"
#define MAX_CURRENT "MAX CURRENT"
#define USED_MAH "USED MAH"
#define BLACKBOX "BLACKBOX"
#define BB_LOG_NUM "BB LOG NUM" // stands for blackbox log number
#define MAX_G_FORCE "MAX G-FORCE"
#define MAX_ESC_TEMP "MAX ESC TEMP"
#define MAX_ESC_RPM "MAX ESC RPM"
#define MIN_LINK "MIN LINK"
#define PEAK_FFT "PEAK FFT"
#define THRT_LESS_20 "THRT<20%" // short for throttle less than 20
#define MIN_RSSI_DBM "MIN RSSI DBM"
#define TOTAL_FLIGHTS "TOTAL FLIGHTS"
#define TOTAL_FLIGHT_TIME "TOTAL FLIGHT TIME"
#define TOTAL_DISTANCE "TOTAL DISTANCE"
#define STATS "  --- STATS ---" // keep the length the same
#define OSD_ARMED "ARMED"
// text from cms_menu_blackbox.c
// text from cms_menu_failsafe.c
// text from cms_menu_firmware.c
// text from cms_menu_gps_rescue.c
// text from cms_menu_imu.c
// text from cms_menu_ledstrip.c
// text from cms_menu_main.c
// text from cms_menu_misc.c
// text from cms_menu_osd.c
// text from cms_menu_persistent_stats.c
// text from cms_menu_power.c
// text from cms_menu_saveexit.c
// text from cms_menu_vtx_common.c
// text from cms_menu_vtx_rtc6705.c
// text from cms_menu_vtx_smartaudio.c
// text from cms_menu_vtx_tramp.c
// text from cms.c

#elif defined(GERMAN_OSD)
// text from osd_elements.c
#define AG "ag" // shortened version of antigravity
#define CRAFTNAME "CRAFT_NAME"
#define DBG "DBG %5d %5d %5d %5d" // keep the same except for the DBG which is short for debug
#define DISARMED "DISARMED"
#define DISPLAYNAME "DISPLAY_NAME"
#define RATE "RATE_%u" // only change the RATE portion
#define FS "!FS!" // only change the FS part which is short for failsafe, keep 4 characters long which inlcude the ! !
#define RESC "RESC" // RESC is short for rescue, keep 4 characters long
#define HEAD "HEAD" // HEAD is short for headfree mode, keep 4 characters long
#define ANGL "ANGL" // short for angle mode, keep 4 characters long
#define HOR "HOR " // short for horizon mode, keep 4 characters long
#define NFE "NFE " // stands for NFE mode, keep 4 characters long
#define AIR "AIR " // short for airmode, keep 4 characters long
#define ACRO "ACRO" // short for acro mode, keep 4 characters long
#define PIT "PIT" // short for pitch, keep 3 characters long
#define ROL "ROL" // short for roll, keep 3 characters long
#define YA "YAW" // keep 3 characters long
#define BEACON_ON " BEACON ON" // keep the space and try to keep the same length
#define ARM_IN "ARM IN %d.%d" // only change the ARM IN keep the rest the same.
#define FAIL_SAFE "FAIL SAFE"
#define CRASH_FLIP "CRASH FLIP" // also known as turtle mode, keep naming consistent with crash flip
#define LAUNCH_ANGLE "LAUNCH %d" // only change the LAUNCH portion
#define LAUNCH "LAUNCH"
#define RSSI_LOW "RSSI LOW"
#define RSSI_DBM "RSSI DBM"
#define LINK_QUALITY "LINK QUALITY"
#define LAND_NOW " LAND NOW"
#define RESCUE_NA "RESCUE N/A"
#define RESCUE_OFF "RESCUE OFF"
#define HEADFREE "HEADFREE"
#define ACC_RECOVERY "ACC RECOVERY"
#define CORE "CORE %c: %3d%c" // only change the CORE portion
#define LOW_BATTERY "LOW BATTERY"
#define RCSMOOTHING "RCSMOOTHING"
#define OVER_CAP "OVER CAP" // short for over capacity, as in your battery capacity
#define BATT_LESS_FULL "BATT < FULL" // short for battery < full, keep the <
// text from osd.c
#define ON_TIME "ON TIME  " // keep the length the same
#define TOTAL_ARM "TOTAL ARM" // keep the length the same
#define LAST_ARM "LAST ARM " // keep the length the same
#define ON_ARM "ON/ARM   " // keep the length the same
#define FAULT "FAULT"
#define NO_RTC "NO RTC" // stands for no real time clock
#define MAX_ALTITUDE "MAX ALTITUDE"
#define MAX_SPEED "MAX SPEED"
#define MAX_DISTANCE "MAX DISTANCE"
#define FLIGHT_DISTANCE "FLIGHT DISTANCE"
#define MIN_BATTERY "MIN BATTERY"
#define END_BATTERY "END BATTERY"
#define BATTERY "BATTERY"
#define MIN_RSSI "MIN RSSI"
#define MAX_CURRENT "MAX CURRENT"
#define USED_MAH "USED MAH"
#define BLACKBOX "BLACKBOX"
#define BB_LOG_NUM "BB LOG NUM" // stands for blackbox log number
#define MAX_G_FORCE "MAX G-FORCE"
#define MAX_ESC_TEMP "MAX ESC TEMP"
#define MAX_ESC_RPM "MAX ESC RPM"
#define MIN_LINK "MIN LINK"
#define PEAK_FFT "PEAK FFT"
#define THRT_LESS_20 "THRT<20%" // short for throttle less than 20
#define MIN_RSSI_DBM "MIN RSSI DBM"
#define TOTAL_FLIGHTS "TOTAL FLIGHTS"
#define TOTAL_FLIGHT_TIME "TOTAL FLIGHT TIME"
#define TOTAL_DISTANCE "TOTAL DISTANCE"
#define STATS "  --- STATS ---" // keep the length the same
#define OSD_ARMED "ARMED"
// text from cms_menu_blackbox.c
// text from cms_menu_failsafe.c
// text from cms_menu_firmware.c
// text from cms_menu_gps_rescue.c
// text from cms_menu_imu.c
// text from cms_menu_ledstrip.c
// text from cms_menu_main.c
// text from cms_menu_misc.c
// text from cms_menu_osd.c
// text from cms_menu_persistent_stats.c
// text from cms_menu_power.c
// text from cms_menu_saveexit.c
// text from cms_menu_vtx_common.c
// text from cms_menu_vtx_rtc6705.c
// text from cms_menu_vtx_smartaudio.c
// text from cms_menu_vtx_tramp.c
// text from cms.c
#endif

#elif defined(TAGALOG_OSD)
// text from osd_elements.c
#define AG "ag" // shortened version of antigravity
#define CRAFTNAME "PANGALAN_NG_DRONE"
#define DBG "DBG %5d %5d %5d %5d" // keep the same except for the DBG which is short for debug
#define DISARMED "DISARMADO"
#define DISPLAYNAME "IPAKITA_ANG_PANGALAN"
#define RATE "RATE_%u" // only change the RATE portion
#define FS "!FS!" // only change the FS part which is short for failsafe, keep 4 characters long which inlcude the ! !
#define RESC "RESC" // RESC is short for rescue, keep 4 characters long
#define HEAD "HEAD" // HEAD is short for headfree mode, keep 4 characters long
#define ANGL "ANGL" // short for angle mode, keep 4 characters long
#define HOR "HOR " // short for horizon mode, keep 4 characters long
#define NFE "NFE " // stands for NFE mode, keep 4 characters long
#define AIR "AIR " // short for airmode, keep 4 characters long
#define ACRO "ACRO" // short for acro mode, keep 4 characters long
#define PIT "PIT" // short for pitch, keep 3 characters long
#define ROL "ROL" // short for roll, keep 3 characters long
#define YA "YAW" // keep 3 characters long
#define BEACON_ON " BEACON ON" // keep the space and try to keep the same length
#define ARM_IN "ARM IN %d.%d" // only change the ARM IN keep the rest the same.
#define FAIL_SAFE "FAIL SAFE"
#define CRASH_FLIP "CRASH FLIP" // also known as turtle mode, keep naming consistent with crash flip
#define LAUNCH_ANGLE "LAUNCH %d" // only change the LAUNCH portion
#define LAUNCH "LAUNCH"
#define RSSI_LOW "RSSI LOW"
#define RSSI_DBM "RSSI DBM"
#define LINK_QUALITY "LINK QUALITY"
#define LAND_NOW " LAPAG NA"
#define RESCUE_NA "RESCUE N/A"
#define RESCUE_OFF "RESCUE OFF"
#define HEADFREE "HEADFREE"
#define ACC_RECOVERY "ACC RECOVERY"
#define CORE "CORE %c: %3d%c" // only change the CORE portion
#define LOW_BATTERY "PAUBOS NA BATIRYA"
#define RCSMOOTHING "RCSMOOTHING"
#define OVER_CAP "OVER CAP" // short for over capacity, as in your battery capacity
#define BATT_LESS_FULL "HINDI PUNONG BATIRYA" // short for battery < full, keep the <
// text from osd.c
#define ON_TIME "ON TIME  " // keep the length the same
#define TOTAL_ARM "TOTAL ARM" // keep the length the same
#define LAST_ARM "HULING ARM " // keep the length the same
#define ON_ARM "ON/ARM   " // keep the length the same
#define FAULT "FAULT"
#define NO_RTC "NO RTC" // stands for no real time clock
#define MAX_ALTITUDE "SAGAD NA TAAS"
#define MAX_SPEED "SAGAD NA BILIS"
#define MAX_DISTANCE "SAGAD NA DISTANSYA"
#define FLIGHT_DISTANCE "DISTANSYA NG LIPAD"
#define MIN_BATTERY "PINAKAMABABANG BOLTAHE"
#define END_BATTERY "HULING BOLTAHE"
#define BATTERY "BATIRYA"
#define MIN_RSSI "MIN RSSI"
#define MAX_CURRENT "SAGAD NA KURYENTE"
#define USED_MAH "NAGAMIT NA MAH"
#define BLACKBOX "BLACKBOX"
#define BB_LOG_NUM "BB LOG NUM" // stands for blackbox log number
#define MAX_G_FORCE "MAX G-FORCE"
#define MAX_ESC_TEMP "SAGAD NA ESC INIT"
#define MAX_ESC_RPM "SAGAD NA ESC RPM"
#define MIN_LINK "MIN LINK"
#define PEAK_FFT "PEAK FFT"
#define THRT_LESS_20 "THRT<20%" // short for throttle less than 20
#define MIN_RSSI_DBM "MIN RSSI DBM"
#define TOTAL_FLIGHTS "DAMI NG LIPAD"
#define TOTAL_FLIGHT_TIME "KABUOANG LIPAD"
#define TOTAL_DISTANCE "KABUOANG DISTANSYA"
#define STATS "  --- STATS ---" // keep the length the same
#define OSD_ARMED "ARMADO"
// text from cms_menu_blackbox.c
// text from cms_menu_failsafe.c
// text from cms_menu_firmware.c
// text from cms_menu_gps_rescue.c
// text from cms_menu_imu.c
// text from cms_menu_ledstrip.c
// text from cms_menu_main.c
// text from cms_menu_misc.c
// text from cms_menu_osd.c
// text from cms_menu_persistent_stats.c
// text from cms_menu_power.c
// text from cms_menu_saveexit.c
// text from cms_menu_vtx_common.c
// text from cms_menu_vtx_rtc6705.c
// text from cms_menu_vtx_smartaudio.c
// text from cms_menu_vtx_tramp.c
// text from cms.c

#ifdef JAPANESE_OSD
// text from osd_elements.c
#define AG "ag" // shortened version of antigravity
#define CRAFTNAME "KETAI-MEI"
#define DBG "DBG %5d %5d %5d %5d" // keep the same except for the DBG which is short for debug
#define DISARMED "KAIJO"
#define DISPLAYNAME "DISPLAY_NAME"
#define RATE "RATE_%u" // only change the RATE portion
#define FS "!FS!" // only change the FS part which is short for failsafe, keep 4 characters long which inlcude the ! !
#define RESC "RESC" // RESC is short for rescue, keep 4 characters long
#define HEAD "HEAD" // HEAD is short for headfree mode, keep 4 characters long
#define ANGL "ANGL" // short for angle mode, keep 4 characters long
#define HOR "HOR " // short for horizon mode, keep 4 characters long
#define NFE "NFE " // stands for NFE mode, keep 4 characters long
#define AIR "AIR " // short for airmode, keep 4 characters long
#define ACRO "ACRO" // short for acro mode, keep 4 characters long
#define PIT "PIT" // short for pitch, keep 3 characters long
#define ROL "ROL" // short for roll, keep 3 characters long
#define YA "YAW" // keep 3 characters long
#define BEACON_ON " BEACON ON" // keep the space and try to keep the same length
#define ARM_IN "ARM IN %d.%d" // only change the ARM IN keep the rest the same.
#define FAIL_SAFE "FAIL SAFE"
#define CRASH_FLIP "CRASH FLIP" // also known as turtle mode, keep naming consistent with crash flip
#define LAUNCH_ANGLE "LAUNCH %d" // only change the LAUNCH portion
#define LAUNCH "LAUNCH"
#define RSSI_LOW "RSSI LOW"
#define RSSI_DBM "RSSI DBM"
#define LINK_QUALITY "LINK QUALITY"
#define LAND_NOW " CHAKURIKU SEYO"
#define RESCUE_NA "RESCUE N/A"
#define RESCUE_OFF "RESCUE OFF"
#define HEADFREE "HEADFREE"
#define ACC_RECOVERY "ACC RECOVERY"
#define CORE "CORE %c: %3d%c" // only change the CORE portion
#define LOW_BATTERY "TEI DEN ATSU"
#define RCSMOOTHING "RCSMOOTHING"
#define OVER_CAP "OVER CAP" // short for over capacity, as in your battery capacity
#define BATT_LESS_FULL "BATT < FULL" // short for battery < full, keep the <
// text from osd.c
#define ON_TIME "ON TIME  " // keep the length the same
#define TOTAL_ARM "TOTAL ARM" // keep the length the same
#define LAST_ARM "LAST ARM " // keep the length the same
#define ON_ARM "ON/ARM   " // keep the length the same
#define FAULT "FAULT"
#define NO_RTC "NO RTC" // stands for no real time clock
#define MAX_ALTITUDE "SAIDAI KOUDO"
#define MAX_SPEED "SAIDAI SOKUDO"
#define MAX_DISTANCE "SAIDAI KYORI"
#define FLIGHT_DISTANCE "SOU HIKOU KYORI"
#define MIN_BATTERY "SAISHOU DEN ATSU"
#define END_BATTERY "GENKAI NO DEN ATSU"
#define BATTERY "BATTERY"
#define MIN_RSSI "MIN RSSI"
#define MAX_CURRENT "SAIDAI DEN RYU"
#define USED_MAH "SHOUHI MAH"
#define BLACKBOX "BLACKBOX"
#define BB_LOG_NUM "BB LOG NUM" // stands for blackbox log number
#define MAX_G_FORCE "MAX G-FORCE"
#define MAX_ESC_TEMP "MAX ESC TEMP"
#define MAX_ESC_RPM "MAX ESC RPM"
#define MIN_LINK "MIN LINK"
#define PEAK_FFT "PEAK FFT"
#define THRT_LESS_20 "THRT<20%" // short for throttle less than 20
#define MIN_RSSI_DBM "MIN RSSI DBM"
#define TOTAL_FLIGHTS "SOU HIKOU"
#define TOTAL_FLIGHT_TIME "SOU HIKOU JIKAN"
#define TOTAL_DISTANCE "SOU KYORI"
#define STATS "  --- STATS ---" // keep the length the same
#define OSD_ARMED "READY"
// text from cms_menu_blackbox.c
// text from cms_menu_failsafe.c
// text from cms_menu_firmware.c
// text from cms_menu_gps_rescue.c
// text from cms_menu_imu.c
// text from cms_menu_ledstrip.c
// text from cms_menu_main.c
// text from cms_menu_misc.c
// text from cms_menu_osd.c
// text from cms_menu_persistent_stats.c
// text from cms_menu_power.c
// text from cms_menu_saveexit.c
// text from cms_menu_vtx_common.c
// text from cms_menu_vtx_rtc6705.c
// text from cms_menu_vtx_smartaudio.c
// text from cms_menu_vtx_tramp.c
// text from cms.c
#endif
