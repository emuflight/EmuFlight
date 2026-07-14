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

#define NOINLINE __attribute__((noinline))

#if !defined(UNIT_TEST) && !defined(SIMULATOR_BUILD) && !(USBD_DEBUG_LEVEL > 0)
#pragma GCC poison sprintf snprintf
#endif

#if defined(STM32F745xx) || defined(STM32F746xx) || defined(STM32F722xx)
#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#include "system_stm32f7xx.h"

#include "stm32f7xx_ll_spi.h"
#include "stm32f7xx_ll_gpio.h"
#include "stm32f7xx_ll_dma.h"
#include "stm32f7xx_ll_rcc.h"
#include "stm32f7xx_ll_bus.h"
#include "stm32f7xx_ll_tim.h"
#include "stm32f7xx_ll_system.h"
#include "drivers/stm32f7xx_ll_ex.h"

// Chip Unique ID on F7
#if defined(STM32F722xx)
#define U_ID_0 (*(uint32_t*)0x1ff07a10)
#define U_ID_1 (*(uint32_t*)0x1ff07a14)
#define U_ID_2 (*(uint32_t*)0x1ff07a18)
#else
#define U_ID_0 (*(uint32_t*)0x1ff0f420)
#define U_ID_1 (*(uint32_t*)0x1ff0f424)
#define U_ID_2 (*(uint32_t*)0x1ff0f428)
#endif

#ifndef STM32F7
#define STM32F7
#endif

#elif defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H723xx) || defined(STM32H725xx) || defined(STM32H730xx) || defined(STM32H735xx) || defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
#ifdef FLASH_SIZE
#undef FLASH_SIZE
#endif
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "system_stm32h7xx.h"

#include "stm32h7xx_ll_spi.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_system.h"
#include "drivers/stm32h7xx_ll_ex.h"

// Chip Unique ID — same register base (0x1FF1E800) on all H7 variants per RM0433/RM0468
#define U_ID_0 (*(uint32_t*)UID_BASE)
#define U_ID_1 (*(uint32_t*)(UID_BASE + 4))
#define U_ID_2 (*(uint32_t*)(UID_BASE + 8))

#ifndef STM32H7
#define STM32H7
#endif

// H7 splits APB1 into APB1LENR/APB1HENR and adds APB4ENR.
// RCC_ClockCmd is a no-op for HAL builds, so only compilation compat is needed.
#define RCC_APB1ENR_I2C1EN   RCC_APB1LENR_I2C1EN
#define RCC_APB1ENR_I2C2EN   RCC_APB1LENR_I2C2EN
#define RCC_APB1ENR_I2C3EN   RCC_APB1LENR_I2C3EN
#define RCC_APB1ENR_I2C4EN   RCC_APB4ENR_I2C4EN
#define RCC_APB1ENR_SPI2EN   RCC_APB1LENR_SPI2EN
#define RCC_APB1ENR_SPI3EN   RCC_APB1LENR_SPI3EN
#define RCC_APB1ENR_USART2EN RCC_APB1LENR_USART2EN
#define RCC_APB1ENR_USART3EN RCC_APB1LENR_USART3EN
#define RCC_APB1ENR_UART4EN  RCC_APB1LENR_UART4EN
#define RCC_APB1ENR_UART5EN  RCC_APB1LENR_UART5EN
#define RCC_APB1ENR_UART7EN  RCC_APB1LENR_UART7EN
#define RCC_APB1ENR_UART8EN  RCC_APB1LENR_UART8EN
#define RCC_APB1ENR_TIM2EN   RCC_APB1LENR_TIM2EN
#define RCC_APB1ENR_TIM3EN   RCC_APB1LENR_TIM3EN
#define RCC_APB1ENR_TIM4EN   RCC_APB1LENR_TIM4EN
#define RCC_APB1ENR_TIM5EN   RCC_APB1LENR_TIM5EN
#define RCC_APB1ENR_TIM6EN   RCC_APB1LENR_TIM6EN
#define RCC_APB1ENR_TIM7EN   RCC_APB1LENR_TIM7EN
#define RCC_APB1ENR_TIM12EN  RCC_APB1LENR_TIM12EN
#define RCC_APB1ENR_TIM13EN  RCC_APB1LENR_TIM13EN
#define RCC_APB1ENR_TIM14EN  RCC_APB1LENR_TIM14EN

// H7 memory section attributes for DMA-capable AXI SRAM placement
#define DMA_RAM     __attribute__((section(".DMA_RAM"), aligned(32)))
#define DMA_RW_AXI  __attribute__((section(".DMA_RW_AXI"), aligned(32)))

// H7 driver enablement flags (BF-style; always on for H7)
#define USE_DMA
#define USE_TIMER
#define USE_UART

// H7 has no TIM9/10/11 — IRQ handler names differ from F7. timer_hal.c uses F7-style.
#define TIM1_UP_TIM10_IRQHandler  TIM1_UP_IRQHandler

// QuadSPI platform traits for bus_quadspi driver
#define MAX_QUADSPI_PIN_SEL 3

#elif defined(STM32F40_41xxx) || defined (STM32F411xE) || defined (STM32F446xx)

#include "stm32f4xx.h"

// Chip Unique ID on F405
#define U_ID_0 (*(uint32_t*)0x1fff7a10)
#define U_ID_1 (*(uint32_t*)0x1fff7a14)
#define U_ID_2 (*(uint32_t*)0x1fff7a18)

#ifndef STM32F4
#define STM32F4
#endif

#elif defined(STM32F303xC)
#include "stm32f30x_conf.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_gpio.h"
#include "core_cm4.h"

// Chip Unique ID on F303
#define U_ID_0 (*(uint32_t*)0x1FFFF7AC)
#define U_ID_1 (*(uint32_t*)0x1FFFF7B0)
#define U_ID_2 (*(uint32_t*)0x1FFFF7B4)

#ifndef STM32F3
#define STM32F3
#endif

#elif defined(STM32F10X)

#include "stm32f10x_conf.h"
#include "stm32f10x_gpio.h"
#include "core_cm3.h"

// Chip Unique ID on F103
#define U_ID_0 (*(uint32_t*)0x1FFFF7E8)
#define U_ID_1 (*(uint32_t*)0x1FFFF7EC)
#define U_ID_2 (*(uint32_t*)0x1FFFF7F0)

#ifndef STM32F1
#define STM32F1
#endif

#elif defined(SIMULATOR_BUILD)

// Nop

#else // STM32F10X
#error "Invalid chipset specified. Update platform.h"
#endif

#include "target/common_fc_pre.h"
#include "target.h"

// USB product name: prefer BOARD_NAME from target.h; otherwise fall back to
// the target-specific USBD_PRODUCT_STRING defined in target.h.
#if defined(BOARD_NAME) && !defined(USBD_PRODUCT_STRING)
#define USBD_PRODUCT_STRINGIFY_(x) #x
#define USBD_PRODUCT_STRINGIFY(x) USBD_PRODUCT_STRINGIFY_(x)
#define USBD_PRODUCT_STRING "EmuFlight - " USBD_PRODUCT_STRINGIFY(BOARD_NAME)
#endif

#include "target/common_fc_post.h"
#include "target/common_defaults_post.h"

#define USE_ARM_MATH // try to use FPU functions

#if defined(UNIT_TEST) || defined(SIMULATOR_BUILD)
// This feature uses 'arm_math.h', which does not exist for x86.
#undef USE_ARM_MATH
#endif