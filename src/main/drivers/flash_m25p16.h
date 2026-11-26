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

#include "flash_impl.h"

bool m25p16_identify(flashDevice_t *fdevice, uint32_t jedecID);

// Number of RDID bytes needed to resolve chips whose layout depends on the extended ID
#define M25P16_EXT_ID_LENGTH 6

// True when the 3-byte ID does not fix the layout and the extended RDID bytes must be checked
bool m25p16_idNeedsExtendedRead(uint32_t jedecID);
// True when the chip behind jedecID has a layout this driver supports; rdid holds RDID bytes 00h..len-1
bool m25p16_extendedIdSupported(uint32_t jedecID, const uint8_t *rdid, uint8_t len);
