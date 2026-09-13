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
#include "common/utils.h"
#include "drivers/io.h"

void debugInit(void)
{
    // A board with a genuine hardware fact excluding PA13/PA14 from TARGET_IO_PORTA would hit
    // a hard compile error from IO_TAG() on the excluded pin without this guard.
#if (TARGET_IO_PORTA) & BIT(13)
    IO_t io = IOGetByTag(IO_TAG(PA13)); // SWDIO
    if (IOGetOwner(io) == OWNER_FREE) {
        IOInit(io, OWNER_SWD, 0);
    }
#endif
#if (TARGET_IO_PORTA) & BIT(14)
    IO_t io2 = IOGetByTag(IO_TAG(PA14)); // SWCLK
    if (IOGetOwner(io2) == OWNER_FREE) {
        IOInit(io2, OWNER_SWD, 0);
    }
#endif
}
