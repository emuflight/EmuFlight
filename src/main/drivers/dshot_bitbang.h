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

#include <stdbool.h>
#include <stdint.h>

#include "drivers/pwm_output.h"

typedef enum {
    DSHOT_BITBANG_OFF,
    DSHOT_BITBANG_ON,
    DSHOT_BITBANG_AUTO,
} dshotBitbangMode_e;

typedef enum {
    DSHOT_BITBANGED_TIMER_AUTO,
    DSHOT_BITBANGED_TIMER_TIM1,
    DSHOT_BITBANGED_TIMER_TIM8,
} dshotBitbangedTimer_e;

typedef enum {
    DSHOT_BITBANG_STATUS_OK,
    DSHOT_BITBANG_STATUS_MOTOR_PIN_CONFLICT,
    DSHOT_BITBANG_STATUS_NO_PACER,
    DSHOT_BITBANG_STATUS_TOO_MANY_PORTS,
    DSHOT_BITBANG_STATUS_DMA_CLAIM,
    DSHOT_BITBANG_STATUS_SPI_DMA2,
} dshotBitbangStatus_e;

#define MOTOR_DSHOT_FRAME_BITS       16
#define MOTOR_DSHOT_STATE_PER_SYMBOL 3  // initial high, bit value, low
#define MOTOR_DSHOT_BIT_HOLD_STATES  3  // idle states so the ESC samples the last bit
#define MOTOR_DSHOT_BUF_LENGTH       ((MOTOR_DSHOT_FRAME_BITS * MOTOR_DSHOT_STATE_PER_SYMBOL) + MOTOR_DSHOT_BIT_HOLD_STATES)

#ifdef USE_DSHOT
// Bitbang makes DSHOT150/300/600 only. AUTO stays inert on EF.
static inline bool dshotBitbangProtocolSupported(uint8_t protocol)
{
    return protocol == PWM_TYPE_DSHOT150 || protocol == PWM_TYPE_DSHOT300 || protocol == PWM_TYPE_DSHOT600;
}

static inline bool dshotBitbangIsActiveFor(uint8_t mode, uint8_t protocol)
{
    return mode == DSHOT_BITBANG_ON && dshotBitbangProtocolSupported(protocol);
}
#endif

// One BSRR word per state: states 0 and 2 of every symbol set and reset all pins of the port,
// state 1 resets the pins whose bit is 0. The hold states keep the line idle.
// The buffer starts zeroed; Init only adds a pin to the set and reset words, so pins already
// set up on the same port keep their packets.
static inline void dshotBitbangOutputDataInit(uint32_t *buffer, uint16_t portMask)
{
    const uint32_t resetMask = (uint32_t)portMask << 16;
    const uint32_t setMask = portMask;

    for (int symbol = 0; symbol < MOTOR_DSHOT_FRAME_BITS; symbol++) {
        buffer[symbol * MOTOR_DSHOT_STATE_PER_SYMBOL + 0] |= setMask;
        buffer[symbol * MOTOR_DSHOT_STATE_PER_SYMBOL + 2] |= resetMask;
    }

    const int holdIndex = MOTOR_DSHOT_FRAME_BITS * MOTOR_DSHOT_STATE_PER_SYMBOL;
    buffer[holdIndex + 0] |= resetMask;
}

static inline void dshotBitbangOutputDataSet(uint32_t *buffer, int pinNumber, uint16_t packet)
{
    const uint32_t middleBit = 1u << (pinNumber + 16);

    // only this pin's bit is rewritten, so a motor not written this frame repeats its last packet
    for (int pos = 0; pos < MOTOR_DSHOT_FRAME_BITS; pos++) {
        uint32_t *middle = &buffer[pos * MOTOR_DSHOT_STATE_PER_SYMBOL + 1];
        *middle &= ~middleBit;
        if (!(packet & 0x8000)) {
            *middle |= middleBit;
        }
        packet <<= 1;
    }
}

struct motorDevConfig_s;

// false on hardware that cannot run bitbang safely; classic DShot runs instead
bool dshotBitbangSupported(void);
// Pin-level setup before DMA is claimed. false means the motor pin cannot be used.
bool dshotBitbangMotorInit(uint8_t motorIndex, uint8_t reorderedMotorIndex, ioTag_t tag);
// DMA and pacer claim, after SPI took its DMA. false stops all DShot output.
bool dshotBitbangPostInit(const struct motorDevConfig_s *motorConfig, uint8_t motorCount);
void dshotBitbangWriteInt(uint8_t motorIndex, uint16_t value);
void dshotBitbangUpdateComplete(uint8_t motorCount);
dshotBitbangStatus_e dshotBitbangGetStatus(void);
bool dshotBitbangIsRunning(void);
void dshotBitbangRestore(void);
