/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/exti.h"
#include "drivers/nvic.h"
#include "drivers/persistent.h"
#include "drivers/system.h"


void SystemClock_Config(void);

void enableGPIOPowerUsageAndNoiseReductions(void)
{
    // AHB4 — GPIO ports (H7: AHB4, not AHB1 as on F4/F7)
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOJ_CLK_ENABLE();
    __HAL_RCC_GPIOK_CLK_ENABLE();
    // AHB1 — DMA
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    // APB1L — timers (H7 APB1 is split into APB1L/APB1H; TIM2-7/12-14 are APB1L)
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_TIM6_CLK_ENABLE();
    __HAL_RCC_TIM7_CLK_ENABLE();   // USB CDC TX poll timer — must be enabled before CDC_Itf_Init
    __HAL_RCC_TIM12_CLK_ENABLE();
    __HAL_RCC_TIM13_CLK_ENABLE();
    __HAL_RCC_TIM14_CLK_ENABLE();
    __HAL_RCC_LPTIM1_CLK_ENABLE();
    // APB1L — SPI, USART, I2C
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_UART5_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();
    __HAL_RCC_I2C3_CLK_ENABLE();
    __HAL_RCC_UART7_CLK_ENABLE();
    __HAL_RCC_UART8_CLK_ENABLE();
    // APB2 — timers (H7: TIM15/16/17 replace F7's TIM9/10/11)
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    __HAL_RCC_TIM15_CLK_ENABLE();
    __HAL_RCC_TIM16_CLK_ENABLE();
    __HAL_RCC_TIM17_CLK_ENABLE();
    // APB2 — SPI, USART
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_SPI4_CLK_ENABLE();
    __HAL_RCC_SPI5_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_USART6_CLK_ENABLE();
}

void configureMasterClockOutputs(void)
{
    // Initialize pins for MCO1 and MCO2 for clock testing/verification

    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_MCO;

    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

bool isMPUSoftReset(void)
{
    if (cachedRccCsrValue & RCC_RSR_SFTRSTF)
        return true;
    else
        return false;
}

#if defined(USE_FLASH_MEMORY_MAPPED)

/*
 * Memory mapped targets use a bootloader which enables memory mapped mode before running the firmware directly from external flash.
 * Code running from external flash, i.e. most of the firmware, must not disable peripherals or reconfigure pins used by the CPU to access the flash chip.
 * Refer to reference manuals and linker scripts for addresses of memory mapped regions.
 * STM32H830 - RM0468 "Table 6. Memory map and default device memory area attributes"
 *
 * If the config is also stored on the same flash chip that code is running from then VERY special care must be taken when detecting the flash chip
 * and when writing an updated config back to the flash.
 */

static bool memoryMappedModeEnabledOnBoot = false;

bool isMemoryMappedModeEnabledOnBoot(void)
{
    return memoryMappedModeEnabledOnBoot;
}

void memoryMappedModeInit(void)
{
#if defined(STM32H730xx) || defined(STM32H723xx)
    // Smaller MCU packages have ONE OCTOSPI interface which supports memory mapped mode.
    memoryMappedModeEnabledOnBoot = READ_BIT(OCTOSPI1->CR, OCTOSPI_CR_FMODE) == OCTOSPI_CR_FMODE;
#else
#error No Memory Mapped implementation on current MCU.
#endif
}
#else
bool isMemoryMappedModeEnabledOnBoot(void)
{
    return false;
}
#endif

void systemInit(void)
{
    // Configure NVIC preempt/priority groups
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITY_GROUPING);

    // cache RCC->RSR value to use it in isMPUSoftReset() and others
    cachedRccCsrValue = RCC->RSR;

    /* Accounts for OP Bootloader, set the Vector Table base address as specified in .ld file */
    //extern void *isr_vector_table_base;
    //NVIC_SetVectorTable((uint32_t)&isr_vector_table_base, 0x0);
    //__HAL_RCC_USB_OTG_FS_CLK_DISABLE;

    //RCC_ClearFlag();

#if defined(STM32H743xx) || defined(STM32H750xx)
    __HAL_RCC_D2SRAM1_CLK_ENABLE();
    __HAL_RCC_D2SRAM2_CLK_ENABLE();
    __HAL_RCC_D2SRAM3_CLK_ENABLE();
#elif defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
    __HAL_RCC_AHBSRAM1_CLK_ENABLE();
    __HAL_RCC_AHBSRAM2_CLK_ENABLE();
#elif defined(STM32H723xx) || defined(STM32H725xx) || defined(STM32H730xx)
    __HAL_RCC_D2SRAM1_CLK_ENABLE();
    __HAL_RCC_D2SRAM2_CLK_ENABLE();
#else
#error Unknown MCU
#endif

#ifdef USE_MCO_OUTPUTS
    configureMasterClockOutputs();
#endif

    // Init cycle counter
    cycleCounterInit();

    // SysTick is updated whenever HAL_RCC_ClockConfig is called.

    enableGPIOPowerUsageAndNoiseReductions();
}

void systemReset(void)
{
    SCB_DisableDCache();
    SCB_DisableICache();

    __disable_irq();
    NVIC_SystemReset();
}

void systemResetWithoutDisablingCaches(void)
{
    __disable_irq();
    NVIC_SystemReset();
}

void systemResetToBootloader(void)
{
    persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_REQUEST_ROM);
    __disable_irq();
    NVIC_SystemReset();
}


#if defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H723xx) || defined(STM32H725xx) || defined(STM32H730xx)
#define SYSMEMBOOT_VECTOR_TABLE ((uint32_t *)0x1ff09800)
#elif defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
#define SYSMEMBOOT_VECTOR_TABLE ((uint32_t *)0x1ff0a000)
#else
#error Unknown MCU
#endif

typedef void *(*bootJumpPtr)(void);

void systemJumpToBootloader(void)
{
    __SYSCFG_CLK_ENABLE();

    uint32_t bootStack =  SYSMEMBOOT_VECTOR_TABLE[0];

    bootJumpPtr SysMemBootJump = (bootJumpPtr)SYSMEMBOOT_VECTOR_TABLE[1];

    __set_MSP(bootStack); //Set the main stack pointer to its default values

    SysMemBootJump();

    while (1);
}


void systemProcessResetReason(void)
{
    uint32_t bootloaderRequest = persistentObjectRead(PERSISTENT_OBJECT_RESET_REASON);

    switch (bootloaderRequest) {
#if defined(USE_FLASH_BOOT_LOADER)
    case RESET_BOOTLOADER_REQUEST_FLASH:
#endif
    case RESET_BOOTLOADER_REQUEST_ROM:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_POST);
        systemJumpToBootloader();

        break;

    case RESET_FORCED:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE);
        break;

    case RESET_BOOTLOADER_POST:
        // Boot loader activity magically prevents SysTick from interrupting.
        // Issue a soft reset to prevent the condition.
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_FORCED);
        systemResetWithoutDisablingCaches(); // observed that disabling dcache after cold boot with BOOT pin high causes segfault.

        break;

    case RESET_MSC_REQUEST:
    case RESET_NONE:
    default:
        break;

    }
}
