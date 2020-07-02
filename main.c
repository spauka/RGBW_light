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

#include <stddef.h>
#include <stdlib.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#include "usb.h"
#include "light.h"

static void clock_setup(void)
{
    /* Use an 8MHz external clock to generate a 48MHz SYSCLK */
    /* Enable external high-speed oscillator 8MHz. */
    rcc_osc_on(RCC_HSE);
    rcc_wait_for_osc_ready(RCC_HSE);
    rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSECLK);

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

static void gpio_setup(void)
{
    /* Enable GPIOA clock. */
    rcc_periph_clock_enable(RCC_GPIOA);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO8 | GPIO9 | GPIO10);
    gpio_clear(GPIOA, GPIO8 | GPIO9 | GPIO10);
}

static void nvic_setup(void)
{
    /* Without this the RTC interrupt routine will never be called. */
    nvic_enable_irq(NVIC_RTC_IRQ);
    nvic_set_priority(NVIC_RTC_IRQ, 1);
}

void rtc_isr(void)
{
    /* The interrupt flag isn't cleared by hardware, we have to do it. */
    rtc_clear_flag(RTC_SEC);

    uint32_t c = rtc_get_counter_val();
    uint16_t d = c & 0x0007;
    d <<= 8;
    /* Display the bottom 3 bits of the counter with the LED's */
    GPIOA_ODR &= 0xF8FF;
    GPIOA_ODR |= d;
}



int main(void)
{
    usbd_device *usbd_dev;

    clock_setup();
    gpio_setup();
    nvic_setup();

    rtc_isr();
    /* Start the interrupt */
    rtc_interrupt_enable(RTC_SEC);

    // Enable the light
    struct light_config light_config;
    uint32_t led_states[300];
    light_config.led_state = led_states;
    light_init(&light_config, 300);

    uint16_t j = 0;
    while (true) {
        for (size_t i = 0; i < 300; i += 1) {
            //light_set(&light_config, i, i, 0, i, 0);
            light_set_hls(&light_config, i, (i+j)%255, 16, 255);
        }
        light_update(&light_config);
        j += 1;
    }

    usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
    usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);
    /* Poll Forever */
    while(1) {
        usbd_poll(usbd_dev);
    }

    return 0;
}
