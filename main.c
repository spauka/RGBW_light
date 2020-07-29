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
#include <stdio.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/adc.h>

#include "usb.h"
#include "light.h"
#include "printf.h"

static const uint8_t USB_VOLTAGE_CHANNEL = ADC_CHANNEL9;

static usbd_device *usbd_dev;
char output_buffer[64];
size_t output_buffer_size = 0;
bool welcome_printed = false;
bool serial_connected = false;

static void clock_setup(void)
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

static void gpio_setup(void)
{
    /* Enable clocks. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_ADC1);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO8 | GPIO9 | GPIO10);
    gpio_clear(GPIOA, GPIO8 | GPIO9 | GPIO10);

    gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_PULL_UPDOWN, GPIO14 | GPIO15);
    gpio_set(GPIOB, GPIO14 | GPIO15);

    /* Set Analog In Mode */
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_ANALOG, GPIO1);

    /* Enable and configure ADC */
    adc_set_single_conversion_mode(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);
    uint8_t channels[1] = {ADC_CHANNEL9};
    adc_set_regular_sequence(ADC1, 1, channels);
    adc_power_on(ADC1);
    adc_reset_calibration(ADC1);
    adc_calibrate(ADC1);
}

uint16_t read_adc(uint8_t channel) {
    adc_set_regular_sequence(ADC1, 1, &channel);
    adc_start_conversion_direct(ADC1);
    while (!adc_eoc(ADC1));
    return adc_read_regular(ADC1);
}

uint32_t read_usb_voltage(void) {
    uint16_t adc_val = read_adc(USB_VOLTAGE_CHANNEL);
    uint32_t voltage = (3300 * adc_val) / 0xFFF;
    voltage = (voltage * 1000000)/545454;
    return voltage;
}

static void nvic_setup(void)
{
    /* Without this the RTC interrupt routine will never be called. */
    nvic_enable_irq(NVIC_RTC_IRQ);
    nvic_set_priority(NVIC_RTC_IRQ, 1);

    /* Enable the USB interrupt too */
    nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
}

void rtc_isr(void)
{
    /* The interrupt flag isn't cleared by hardware, we have to do it. */
    RTC_CRL &= ~RTC_CRL_SECF;

    // Read the counter value
    uint32_t c = (RTC_CNTH << 16) | RTC_CNTL;
    uint16_t d = c & 0x0007;
    d <<= 8;
    /* Display the bottom 3 bits of the counter with the LED's */
    GPIOA_ODR &= 0xF8FF;
    GPIOA_ODR |= d;

    output_buffer_size = snprintf(output_buffer, 64, "Clock Tick: %d\r\n", c);
}

void usb_lp_can_rx0_isr(void)
{
    /* Poll the state of the USB */
    usbd_poll(usbd_dev);
}

void _putchar(char c)
{
    if (serial_connected) {
        int ret = 0;
        while (ret == 0)
            ret = usbd_ep_write_packet(usbd_dev, 0x82, &c, 1);
    }
}

int main(void)
{
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

    uint8_t brightness = 8;
    uint16_t max_leds = 300;
    uint32_t usb_voltage = 0;

    usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
    usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);
    usbd_register_suspend_callback(usbd_dev, cdcacm_suspend);
    usbd_register_resume_callback(usbd_dev, cdcacm_wkup);

    /* Poll Forever */
    uint32_t j = 0;
    while(1) {
        if (serial_connected && !welcome_printed) {
            printf("SK6812 Light Controller\r\n");
            printf("Connected...\r\n");
            welcome_printed = true;
        } else if (serial_connected) {
            if (output_buffer_size > 0) {
                usbd_ep_write_packet(usbd_dev, 0x82, output_buffer, output_buffer_size+1);
                output_buffer_size = 0;
            }
        } else if (!serial_connected) {
            welcome_printed = false;
        }

        uint16_t value = ~gpio_port_read(GPIOB);
        if (value & 0x8000) {
            if (brightness < 255)
                brightness += 1;
        }
        if (value & 0x4000) {
            if (brightness > 0)
                brightness -= 1;
        }

        for (size_t i = 0; i < max_leds; i += 1) {
            //light_set_hsv(&light_config, i, ((i+j)<<10)%maxHue, 0xFFFF, 8);
            light_set(&light_config, i, 0, 0, 0, brightness);
        }
        for (size_t i = max_leds; i < 300; i += 1) {
            light_set(&light_config, i, 0, 0, 0, 0);
        }
        light_update(&light_config);
        j += 1;

        usb_voltage = read_usb_voltage();
        if (usb_voltage < 4700 && max_leds > 0) {
            max_leds -= 1;
        } else if (usb_voltage > 4800 && max_leds < 300) {
            max_leds += 1;
        }

        if (serial_connected && j%5 == 0) {
            printf("USB Voltage: %dmV\r\n", usb_voltage);
        }
    }

    return 0;
}
