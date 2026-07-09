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
#include <string.h>

#include "platform.h"

#if defined(USE_BOARD_INFO)
#include "build/version.h"

#include "common/utils.h"

#include "fc/board_info.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "board.h"

PG_REGISTER_WITH_RESET_FN(boardConfig_t, boardConfig, PG_BOARD_CONFIG, 0);

void pgResetFn_boardConfig(boardConfig_t *boardConfig) {
    if (boardInformationIsSet()) {
        strncpy(boardConfig->manufacturerId, getManufacturerId(), MAX_MANUFACTURER_ID_LENGTH);
        boardConfig->manufacturerId[MAX_MANUFACTURER_ID_LENGTH] = '\0';
        strncpy(boardConfig->boardName, getBoardName(), MAX_BOARD_NAME_LENGTH);
        boardConfig->boardName[MAX_BOARD_NAME_LENGTH] = '\0';
        boardConfig->boardInformationSet = true;
    } else {
#if !defined(GENERIC_TARGET)
        // BOARD_NAME (bare-token, BF-style) is intentionally NOT consumed here:
        // EF's build system passes -D$(TARGET) (bare, unquoted) for every target
        // (Makefile, plus -D$(BASE_TARGET) for shared-target.h sub-variants), so
        // for any target whose BOARD_NAME value matches its own target name (the
        // expected/standard convention), STR(BOARD_NAME) fully macro-expands
        // through that collision down to "1" instead of the intended name.
        // MANUFACTURER_ID doesn't have this problem (its value never matches a
        // target name), so it's safe to prefer as-is. Fall back to EF-style
        // USBD_PRODUCT_STRING/TARGET_MANUFACTURER_IDENTIFIER otherwise.
        //
        // strncpy() truncates at MAX_*_LENGTH and the trailing NUL is set
        // explicitly: some values (e.g. HGLRCF405V2's MANUFACTURER_ID, exactly
        // MAX_MANUFACTURER_ID_LENGTH chars; CRAZYFLIE2's USBD_PRODUCT_STRING
        // "Crazyflie 2.0 (BigQuad Deck)", far longer than MAX_BOARD_NAME_LENGTH)
        // are compile-time string literals GCC can prove are >= n chars, so it
        // correctly flags plain strncpy() here as -Wstringop-truncation -- the
        // explicit assignment resolves that unconditionally, for any macro
        // value, without depending on the PG reset buffer's memset(0) (which
        // fc/board_info.c's copies of runtime-length values rely on instead).
#if defined(USBD_PRODUCT_STRING)
        strncpy(boardConfig->boardName, USBD_PRODUCT_STRING, MAX_BOARD_NAME_LENGTH);
#else
        strncpy(boardConfig->boardName, targetName, MAX_BOARD_NAME_LENGTH);
#endif
        boardConfig->boardName[MAX_BOARD_NAME_LENGTH] = '\0';
#if defined(MANUFACTURER_ID)
        strncpy(boardConfig->manufacturerId, STR(MANUFACTURER_ID), MAX_MANUFACTURER_ID_LENGTH);
#elif defined(TARGET_MANUFACTURER_IDENTIFIER)
        strncpy(boardConfig->manufacturerId, TARGET_MANUFACTURER_IDENTIFIER, MAX_MANUFACTURER_ID_LENGTH);
#endif
        boardConfig->manufacturerId[MAX_MANUFACTURER_ID_LENGTH] = '\0';
        boardConfig->boardInformationSet = true;
#else
        boardConfig->boardInformationSet = false;
#endif // GENERIC_TARGET
    }
#if defined(USE_SIGNATURE)
    if (signatureIsSet()) {
        memcpy(boardConfig->signature, getSignature(), SIGNATURE_LENGTH);
        boardConfig->signatureSet = true;
    } else {
        boardConfig->signatureSet = false;
    }
#endif
}
#endif // USE_BOARD_INFO:
