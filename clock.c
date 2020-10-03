/*
 * Copyright 2020 Sebastian Pauka
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef STM32F1
#define STM32F1
#endif

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rtc.h>

void clock_setup(void)
{
    /* Use an 8MHz external clock to generate a 48MHz SYSCLK */
    /* Enable external high-speed oscillator 8MHz. */
    rcc_osc_on(RCC_HSE);
    rcc_wait_for_osc_ready(RCC_HSE);
    rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSECLK);
    rcc_osc_off(RCC_HSI);

    /*
     * Set prescalers for AHB, ADC, APB1, APB2.
     */
    rcc_set_hpre(RCC_CFGR_HPRE_SYSCLK_NODIV);    /* Set. 48MHz Max. 72MHz */
    rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV4);  /* Set. 12MHz Max. 14MHz */
    rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_DIV2);     /* Set. 24MHz Max. 36MHz */
    rcc_set_ppre2(RCC_CFGR_PPRE2_HCLK_NODIV);    /* Set. 48MHz Max. 72MHz */

    /*
     * Sysclk runs with 24MHz -> 0 waitstates.
     * 0WS from 0-24MHz
     * 1WS from 24-48MHz
     * 2WS from 48-72MHz
     */
    flash_set_ws(FLASH_ACR_LATENCY_1WS);

    /*
     * Set the PLL multiplication factor to 3.
     * 8MHz (external) * 6 (multiplier) = 48MHz
     */
    rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_PLL_CLK_MUL6);

    /* Select HSE as PLL source. */
    rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_CLK);

    /*
     * External frequency undivided before entering PLL
     * (only valid/needed for HSE).
     */
    rcc_set_pllxtpre(RCC_CFGR_PLLXTPRE_HSE_CLK);

    /* Enable PLL oscillator and wait for it to stabilize. */
    rcc_osc_on(RCC_PLL);
    rcc_wait_for_osc_ready(RCC_PLL);

    /* Select PLL as SYSCLK source. */
    rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);

    /* Set the peripheral clock frequencies used */
    rcc_ahb_frequency = 48000000;
    rcc_apb1_frequency = 24000000;
    rcc_apb2_frequency = 48000000;

    /* Set USB divider */
    rcc_set_usbpre(RCC_CFGR_USBPRE_PLL_CLK_NODIV);

    /* Set up the RTC using a 32.768KHz Crystal */
    rtc_auto_awake(RCC_LSE, 0x7FFF);
}

uint8_t rtc_h(void) {
    uint32_t c = (RTC_CNTH << 16) | RTC_CNTL;
    return (uint8_t) ((c/3600) % 24);
}
uint8_t rtc_m(void) {
    uint32_t c = (RTC_CNTH << 16) | RTC_CNTL;
    return (uint8_t) ((c/60) % 60);
}
uint8_t rtc_s(void) {
    uint32_t c = (RTC_CNTH << 16) | RTC_CNTL;
    return (uint8_t) (c % 60);
}

void rtc_set_time(uint8_t h, uint8_t m, uint8_t s) {
    // Calculate time
    uint32_t t = (h * 3600) + (m * 60) + s;
    // Set time
    rtc_set_counter_val(t);
}