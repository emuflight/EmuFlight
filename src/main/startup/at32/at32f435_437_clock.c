/**
  **************************************************************************
  * @file     at32f435_437_clock.c
  * @brief    system clock config program
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to 
  * download from Artery official website is the copyrighted work of Artery. 
  * Artery authorizes customers to use, copy, and distribute the BSP 
  * software and its related documentation for the purpose of design and 
  * development in conjunction with Artery microcontrollers. Use of the 
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

/* includes ------------------------------------------------------------------*/
#include "at32f435_437_clock.h"
#include "platform.h"

/* 576 MHz VCO with the /2 post-divider gives 288 MHz; the first pre-divider that keeps the PLL input in 2..16 MHz and the multiplier integral in 31..500 is used. HEXT is 4..25 MHz (DS V2.30). */
#define PLL_VCO_HZ 576000000ULL
#define PLL_MS_OK(ms) ((HEXT_VALUE >= 4000000U) && (HEXT_VALUE <= 25000000U) && (HEXT_VALUE >= 2000000U * (ms)) && (HEXT_VALUE <= 16000000U * (ms)) && \
                       (((PLL_VCO_HZ * (ms)) % HEXT_VALUE) == 0) && \
                       ((PLL_VCO_HZ * (ms) / HEXT_VALUE) >= 31) && ((PLL_VCO_HZ * (ms) / HEXT_VALUE) <= 500))
#if PLL_MS_OK(1)
#define PLL_MS 1
#elif PLL_MS_OK(2)
#define PLL_MS 2
#elif PLL_MS_OK(3)
#define PLL_MS 3
#elif PLL_MS_OK(4)
#define PLL_MS 4
#elif PLL_MS_OK(5)
#define PLL_MS 5
#elif PLL_MS_OK(6)
#define PLL_MS 6
#elif PLL_MS_OK(7)
#define PLL_MS 7
#elif PLL_MS_OK(8)
#define PLL_MS 8
#elif PLL_MS_OK(9)
#define PLL_MS 9
#elif PLL_MS_OK(10)
#define PLL_MS 10
#elif PLL_MS_OK(11)
#define PLL_MS 11
#elif PLL_MS_OK(12)
#define PLL_MS 12
#elif PLL_MS_OK(13)
#define PLL_MS 13
#elif PLL_MS_OK(14)
#define PLL_MS 14
#elif PLL_MS_OK(15)
#define PLL_MS 15
#else
#error "SYSTEM_HSE_MHZ cannot produce a 576 MHz PLL VCO frequency within the PLL limits"
#endif
#define PLL_NS ((uint32_t)(PLL_VCO_HZ * PLL_MS / HEXT_VALUE))

/**
  * @brief  system clock config program
  * @note   the system clock is configured as follow:
  *         - system clock        = (hext * pll_ns)/(pll_ms * pll_fr)
  *         - system clock source = pll (hext)
  *         - hext                = 8000000
  *         - sclk                = 288000000
  *         - ahbdiv              = 1
  *         - ahbclk              = 288000000
  *         - apb1div             = 2
  *         - apb1clk             = 144000000
  *         - apb2div             = 2
  *         - apb2clk             = 144000000
  *         - pll_ns              = 72
  *         - pll_ms              = 1
  *         - pll_fr              = 2
  * @param  none
  * @retval none
  */
void system_clock_config(void)
{
  /* enable pwc periph clock */
  crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);

  /* config ldo voltage */
  pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V3);
 
  /* set the flash clock divider */
  flash_clock_divider_set(FLASH_CLOCK_DIV_3);

  /* reset crm */
  crm_reset();

  /* enable hext */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);

   /* wait till hext is ready */
  while(crm_hext_stable_wait() == ERROR)
  {
  }

  /* enable hick */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HICK, TRUE);

   /* wait till hick is ready */
  while(crm_flag_get(CRM_HICK_STABLE_FLAG) != SET)
  {
  }

  /* config pll clock resource */
  crm_pll_config(CRM_PLL_SOURCE_HEXT, PLL_NS, PLL_MS, CRM_PLL_FR_2);

  /* enable pll */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);

  /* wait till pll is ready */
  while(crm_flag_get(CRM_PLL_STABLE_FLAG) != SET)
  {
  }

  /* config ahbclk */
  crm_ahb_div_set(CRM_AHB_DIV_1);

  /* config apb2clk */
  crm_apb2_div_set(CRM_APB2_DIV_2);

  /* config apb1clk */
  crm_apb1_div_set(CRM_APB1_DIV_2);

  /* enable auto step mode */
  crm_auto_step_mode_enable(TRUE);

  /* select pll as system clock source */
  crm_sysclk_switch(CRM_SCLK_PLL);

  /* wait till pll is used as system clock source */
  while(crm_sysclk_switch_status_get() != CRM_SCLK_PLL)
  {
  }

  /* disable auto step mode */
  crm_auto_step_mode_enable(FALSE);

  /* config usbclk from pll */
  crm_usb_clock_div_set(CRM_USB_DIV_6);
  crm_usb_clock_source_select(CRM_USB_CLOCK_SOURCE_PLL);

  /* update system_core_clock global variable */
  system_core_clock_update();
}
