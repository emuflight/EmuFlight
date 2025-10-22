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

#pragma once

#include "drivers/display.h"

#include "common/time.h"

extern bool cmsInMenu;

// Device management
bool cmsDisplayPortRegister(displayPort_t *pDisplay);
extern displayPort_t *pCurrentDisplay;

// For main.c and scheduler
void cmsInit(void);
void cmsHandler(timeUs_t currentTimeUs);

bool cmsDisplayPortSelect(displayPort_t *instance);
void cmsMenuOpen(void);
long cmsMenuChange(displayPort_t *pPort, const void *ptr);
long cmsMenuExit(displayPort_t *pPort, const void *ptr);
void cmsUpdate(uint32_t currentTimeUs);

#define CMS_STARTUP_HELP_TEXT1 "MENU:THR MID"
#define CMS_STARTUP_HELP_TEXT2     "+ YAW LEFT"
#define CMS_STARTUP_HELP_TEXT3     "+ PITCH UP"

// cmsMenuExit special ptr values
#define CMS_EXIT             (0)
#define CMS_EXIT_SAVE        (1)
#define CMS_EXIT_SAVEREBOOT  (2)
