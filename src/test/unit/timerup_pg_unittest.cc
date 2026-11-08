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

// The Makefile defines TIMUP1/3/8/15/20_DMA_OPT; slot n holds TIM(n+1)'s TIM_UP stream option.

extern "C" {

#include "platform.h"
#include "pg/timerup.h"

void pgResetFn_timerUpConfig(timerUpConfig_t *config);

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(TimerUpPgUnittest, DefinedMacrosLandInTheirTimerSlot)
{
    timerUpConfig_t config[HARDWARE_TIMER_DEFINITION_COUNT + 1] = {};
    pgResetFn_timerUpConfig(config);

    EXPECT_EQ(config[0].dmaopt, 0);  // TIM1: an explicit 0 must stay distinguishable from unset
    EXPECT_EQ(config[2].dmaopt, 2);  // TIM3
    EXPECT_EQ(config[7].dmaopt, 11); // TIM8
}

TEST(TimerUpPgUnittest, TimersWithoutAMacroStayUnused)
{
    timerUpConfig_t config[HARDWARE_TIMER_DEFINITION_COUNT + 1] = {};
    pgResetFn_timerUpConfig(config);

    EXPECT_EQ(config[1].dmaopt, DMA_OPT_UNUSED); // TIM2
    EXPECT_EQ(config[3].dmaopt, DMA_OPT_UNUSED); // TIM4
    EXPECT_EQ(config[11].dmaopt, DMA_OPT_UNUSED); // TIM12
}

TEST(TimerUpPgUnittest, MacrosPastTheSlotCountAreIgnored)
{
    // adversarial: TIMUP15/TIMUP20 are defined but the array has only 14 slots; no write may land
    // in the slot just past the end (sentinel) or wrap into a valid one.
    timerUpConfig_t config[HARDWARE_TIMER_DEFINITION_COUNT + 1] = {};
    config[HARDWARE_TIMER_DEFINITION_COUNT].dmaopt = 99;
    pgResetFn_timerUpConfig(config);

    EXPECT_EQ(config[HARDWARE_TIMER_DEFINITION_COUNT].dmaopt, 99);
    EXPECT_EQ(config[13].dmaopt, DMA_OPT_UNUSED);
}
