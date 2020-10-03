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

#include "clock.h"
#include "io.h"
#include "usb.h"
#include "light.h"
#include "printf.h"

#define N_LED 72

usbd_device *usbd_dev;
uint32_t clock_tick = 0;
bool welcome_printed = false;
bool serial_connected = false;

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
    // uint16_t d = c & 0x0007;
    // d <<= 8;
    // /* Display the bottom 3 bits of the counter with the LED's */
    // GPIOA_ODR &= 0xF8FF;
    // GPIOA_ODR |= d;

    /* Print out the clock tick in the main thread */
    clock_tick = c;
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
    uint32_t led_states[N_LED];
    light_config.led_state = led_states;
    light_init(&light_config, N_LED);

    uint8_t brightness = 2;
    uint8_t max_brightness = 0;
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
        } else if (!serial_connected) {
            welcome_printed = false;
        }

        if (serial_connected && clock_tick) {
            printf("Clock Tick: %d\r\n", clock_tick);
            printf("Clock Time: %.2d:%.2d:%.2d\r\n", rtc_h(), rtc_m(), rtc_s());
            clock_tick = 0;
        }

        /* Try write out any available packets */
        output_serial(usbd_dev);

        /* Read button states */
        uint16_t value = ~gpio_port_read(GPIOB);
        if (value & 0x8000 && j%25 == 1) {
            rtc_set_time(rtc_h(), rtc_m()+1, 0);
        }
        if (value & 0x4000 && j%25 == 1) {
            if (rtc_m() == 0)
                rtc_set_time(rtc_h()-1, 59, 0);
            else
                rtc_set_time(rtc_h(), rtc_m()-1, 0);
        }

        /* Set light states */
        for (size_t i = 0; i < N_LED; i += 1) {
            uint8_t allowed_brightness = ((max_brightness < brightness) ? max_brightness : brightness);
            light_set_hsv(&light_config, i, ((i+j)<<10)%maxHue, 0xAFFF, allowed_brightness);
            //light_set(&light_config, i, 0, 0, 0, allowed_brightness);
        }
        light_update(&light_config);
        j += 1;

        usb_voltage = read_usb_voltage();
        if (usb_voltage < 4700 && max_brightness > 0) {
            max_brightness -= 1;
        } else if (usb_voltage > 4800 && max_brightness < 255) {
            max_brightness += 1;
        }

        if (serial_connected && j%1000 == 0) {
            printf("USB Voltage: %dmV\r\n", usb_voltage);
            int32_t temperature = read_temp();
            printf("Temperature: %d.%.3dC\r\n", temperature/1000, temperature%1000);
        }
    }

    return 0;
}
