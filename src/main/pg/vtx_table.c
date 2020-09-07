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

#include <string.h>
#include "common/printf.h"

#include "platform.h"

#ifdef USE_VTX_TABLE

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/vtx_table.h"
#include "drivers/vtx_common.h"
#include "drivers/vtx_table.h"
#include "config/config_reset.h"

PG_REGISTER_WITH_RESET_FN(vtxTableConfig_t, vtxTableConfig, PG_VTX_TABLE_CONFIG, 0);

void pgResetFn_vtxTableConfig(vtxTableConfig_t *vtxTableConfig)
{
      vtxTableConfig->bands = 5;
      vtxTableConfig->channels = 8;
      for (int band = 0; band < VTX_TABLE_MAX_BANDS; band++) {
          vtxTableConfigClearBand(vtxTableConfig, band);
      }
      // clear the vtx tables so that no non defaults slip through
      vtxTableConfig->frequency[0][0] = 5865;
      vtxTableConfig->frequency[0][1] = 5845;
      vtxTableConfig->frequency[0][2] = 5825;
      vtxTableConfig->frequency[0][3] = 5805;
      vtxTableConfig->frequency[0][4] = 5785;
      vtxTableConfig->frequency[0][5] = 5765;
      vtxTableConfig->frequency[0][6] = 5745;
      vtxTableConfig->frequency[0][7] = 5725;
      vtxTableConfig->frequency[1][0] = 5733;
      vtxTableConfig->frequency[1][1] = 5752;
      vtxTableConfig->frequency[1][2] = 5771;
      vtxTableConfig->frequency[1][3] = 5790;
      vtxTableConfig->frequency[1][4] = 5809;
      vtxTableConfig->frequency[1][5] = 5828;
      vtxTableConfig->frequency[1][6] = 5847;
      vtxTableConfig->frequency[1][7] = 5866;
      vtxTableConfig->frequency[2][0] = 5705;
      vtxTableConfig->frequency[2][1] = 5685;
      vtxTableConfig->frequency[2][2] = 5665;
      vtxTableConfig->frequency[2][3] = 5645;
      vtxTableConfig->frequency[2][4] = 5885;
      vtxTableConfig->frequency[2][5] = 5905;
      vtxTableConfig->frequency[2][6] = 5925;
      vtxTableConfig->frequency[2][7] = 5945;
      vtxTableConfig->frequency[3][0] = 5740;
      vtxTableConfig->frequency[3][1] = 5760;
      vtxTableConfig->frequency[3][2] = 5780;
      vtxTableConfig->frequency[3][3] = 5800;
      vtxTableConfig->frequency[3][4] = 5820;
      vtxTableConfig->frequency[3][5] = 5840;
      vtxTableConfig->frequency[3][6] = 5860;
      vtxTableConfig->frequency[3][7] = 5880;
      vtxTableConfig->frequency[4][0] = 5658;
      vtxTableConfig->frequency[4][1] = 5695;
      vtxTableConfig->frequency[4][2] = 5732;
      vtxTableConfig->frequency[4][3] = 5769;
      vtxTableConfig->frequency[4][4] = 5806;
      vtxTableConfig->frequency[4][5] = 5843;
      vtxTableConfig->frequency[4][6] = 5880;
      vtxTableConfig->frequency[4][7] = 5917;
            // 0 = { 0 = 5865, 1 = 5845, 2 = 5825, 3 = 5805, 4 = 5785, 5 = 5765, 6 = 5745, 7 = 5725 }, // Boscam A
            // 1 = { 5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866 }, // Boscam B
            // 2 = { 5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945 }, // Boscam E
            // 3 = { 5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880 }, // FatShark
            // 4 = { 5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917 }, // RaceBand
        strcpy(vtxTableConfig->bandNames[0], "--------");
        strcpy(vtxTableConfig->bandNames[1], "BOSCAM A");
        strcpy(vtxTableConfig->bandNames[2], "BOSCAM B");
        strcpy(vtxTableConfig->bandNames[3], "BOSCAM E");
        strcpy(vtxTableConfig->bandNames[4], "FATSHARK");
        strcpy(vtxTableConfig->bandNames[5], "RACEBAND");

        vtxTableConfig->bandLetters[0] = '-';
        vtxTableConfig->bandLetters[1] = 'A';
        vtxTableConfig->bandLetters[2] = 'B';
        vtxTableConfig->bandLetters[3] = 'E';
        vtxTableConfig->bandLetters[4] = 'F';
        vtxTableConfig->bandLetters[5] = 'R';
        //   0 = "-",
        //   1 = "A",
        //   2 = "B",
        //   3 = "E",
        //   4 = "F",
        //   5 = "R",

        strcpy(vtxTableConfig->channelNames[0], "-");
        strcpy(vtxTableConfig->channelNames[1], "1");
        strcpy(vtxTableConfig->channelNames[2], "2");
        strcpy(vtxTableConfig->channelNames[3], "3");
        strcpy(vtxTableConfig->channelNames[4], "4");
        strcpy(vtxTableConfig->channelNames[5], "5");
        strcpy(vtxTableConfig->channelNames[6], "6");
        strcpy(vtxTableConfig->channelNames[7], "7");
        strcpy(vtxTableConfig->channelNames[8], "8");
        //         0 = "-",
        //         1 = "1",
        //         2 = "2",
        //         3 = "3",
        //         4 = "4",
        //         5 = "5",
        //         6 = "6",
        //         7 = "7",
        //         8 = "8",

        vtxTableConfig->powerLevels = 4;
        vtxTableConfigClearPowerValues(vtxTableConfig, 0);
        vtxTableConfigClearPowerLabels(vtxTableConfig, 0);
        // clear the power and labels so that no non defaults slip through
        vtxTableConfig->powerValues[0] = 0;
        vtxTableConfig->powerValues[1] = 1;
        vtxTableConfig->powerValues[2] = 2;
        vtxTableConfig->powerValues[3] = 3;
        //   0 = 0,
        //   1 = 1,
        //   2 = 2,
        //   3 = 3,

        strcpy(vtxTableConfig->powerLabels[0], "25");
        strcpy(vtxTableConfig->powerLabels[1], "200");
        strcpy(vtxTableConfig->powerLabels[2], "500");
        strcpy(vtxTableConfig->powerLabels[3], "800");
        //   0 = "25",
        //   1 = "200",
        //   2 = "500",
        //   3 = "800",
}
#endif
