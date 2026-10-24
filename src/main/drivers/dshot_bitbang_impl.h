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

#include "platform.h"

#include "drivers/dma.h"
#include "drivers/dshot_bitbang.h"
#include "drivers/io_types.h"
#include "drivers/timer.h"

#define MAX_SUPPORTED_MOTOR_PORTS 4 // one pacer channel per GPIO port group
#define MAX_MOTOR_PACERS          4 // TIM1 and TIM8 at most

typedef struct bbDmaRegs_s {
    uint32_t CR;
    uint32_t FCR;
    uint32_t NDTR;
    uint32_t PAR;
    uint32_t M0AR;
} bbDmaRegs_t;

// One pacer timer: its channel DMA requests clock the BSRR writes of every port on it
typedef struct bbPacer_s {
    TIM_TypeDef *tim;
    uint16_t dmaSources;
} bbPacer_t;

// One GPIO port group, driven by one DMA stream paced by one timer channel
typedef struct bbPort_s {
    int portIndex;
    GPIO_TypeDef *gpio;
    TIM_TypeDef *tim;
    uint8_t timChannel;
    uint16_t dmaSource;
    DMA_Stream_TypeDef *dmaRef;
    uint32_t dmaChannel;
    dmaIdentifier_e dmaIdentifier;
    bbDmaRegs_t dmaRegs;
    uint16_t outputARR;
    uint32_t *portOutputBuffer;
    uint32_t portOutputCount;
} bbPort_t;

typedef struct bbMotor_s {
    int pinIndex;
    int portIndex;
    IO_t io;
    uint32_t iocfg;
    bbPort_t *bbPort;
    bool configured;
} bbMotor_t;

// MCU specific, dshot_bitbang_ll.c (F7/H7) or dshot_bitbang_stdperiph.c (F4)
void bbTimerChannelInit(const bbPort_t *bbPort);
void bbDMAPreconfigure(bbPort_t *bbPort);
void bbDMALoadRegs(const bbPort_t *bbPort);
void bbDMACmd(const bbPort_t *bbPort, bool enable);
bool bbDMAIsEnabled(const bbPort_t *bbPort);
void bbTIMDMACmd(TIM_TypeDef *tim, uint16_t dmaSources, bool enable);
