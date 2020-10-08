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

union rgbw deep_night = {
    .r = 0,
    .g = 0,
    .b = 1,
    .w = 0
};

union rgbw night = {
    .r = 0,
    .g = 0,
    .b = 1,
    .w = 0
};

union rgbw dawn = {
    .r = 8,
    .g = 0,
    .b = 8,
    .w = 4
};

union rgbw day = {
    .r = 0,
    .g = 8,
    .b = 64,
    .w = 32
};

union rgbw dusk = {
    .r = 6,
    .g = 0,
    .b = 16,
    .w = 1
};

static const uint8_t dawn_brightness = 75;
static const uint8_t day_brightness = 100;
static const uint8_t dusk_brightness = 80;
static const uint8_t night_brightness = 50;
static const uint8_t deep_night_brightness = 10;

static const uint8_t dawn_start = 6;
static const uint8_t day_start = 8;
static const uint8_t dusk_start = 17;
static const uint8_t night_start = 20;
static const uint8_t deep_night_start = 22;
static const uint8_t transition_time = 30; // Time in mins

int16_t interpolate(int16_t start, int16_t stop, int16_t pos, int16_t length);
union rgbw interpolate_col(union rgbw start, union rgbw stop, int16_t pos, int16_t length);

int16_t interpolate(int16_t start, int16_t stop, int16_t pos, int16_t length) {
    if (pos < 0)
        return start;
    if (pos > length)
        return stop;
    return start + ((pos*(stop - start))/(length-1));
}

union rgbw interpolate_col(union rgbw start, union rgbw stop, int16_t pos, int16_t length) {
    union rgbw result;
    result.r = interpolate(start.r, stop.r, pos, length);
    result.g = interpolate(start.g, stop.g, pos, length);
    result.b = interpolate(start.b, stop.b, pos, length);
    result.w = interpolate(start.w, stop.w, pos, length);
    return result;
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

        if (clock_tick) {
            if (serial_connected) {
                printf("Clock Time: %.4d:%.2d:%.2d:%.2d\r\n", rtc_d(), rtc_h(), rtc_m(), rtc_s());
            }

            /* Figure out which time we are in */
            union rgbw result;
            uint16_t brightness;
            uint8_t h = rtc_h();
            if (h < dawn_start) {
                result = deep_night;
                brightness = deep_night_brightness;
            } else if (h == dawn_start) {
                result = interpolate_col(deep_night, dawn, rtc_m(), transition_time);
                brightness = interpolate(deep_night_brightness, dawn_brightness, rtc_m(), transition_time);
            } else if (h < day_start) {
                result = dawn;
                brightness = dawn_brightness;
            } else if (h == day_start) {
                result = interpolate_col(dawn, day, rtc_m(), transition_time);
                brightness = interpolate(dawn_brightness, day_brightness, rtc_m(), transition_time);
            } else if (h < dusk_start) {
                result = day;
                brightness = day_brightness;
            } else if (h == dusk_start) {
                result = interpolate_col(day, dusk, rtc_m(), transition_time);
                brightness = interpolate(day_brightness, dusk_brightness, rtc_m(), transition_time);
            } else if (h < night_start) {
                result = dusk;
                brightness = dusk_brightness;
            } else if (h == night_start){
                result = interpolate_col(dusk, night, rtc_m(), transition_time);
                brightness = interpolate(dusk_brightness, night_brightness, rtc_m(), transition_time);
            } else if (h < deep_night_start) {
                result = night;
                brightness = night_brightness;
            } else if (h == deep_night_start) {
                result = interpolate_col(night, deep_night, rtc_m(), transition_time);
                brightness = interpolate(night_brightness, deep_night_brightness, rtc_m(), transition_time);
            } else {
                result = deep_night;
                brightness = deep_night_brightness;
            }
            brightness = (brightness*N_LED)/100;
            uint16_t num_should_light = 0, num_lit = 0;
            for (size_t i = 0; i < N_LED; i += 1) {
                num_should_light = (brightness*(i+1))/100;
                if (num_lit < num_should_light) {
                    led_states[i] = result.rgbw;
                    num_lit++;
                }
                else {
                    led_states[i] = 0;
                }
            }
            light_update(&light_config);
            clock_tick = 0;
        }

        /* Try write out any available packets */
        output_serial(usbd_dev);

        /* Read button states */
        uint16_t value = ~gpio_port_read(GPIOB);
        if (value & 0x8000 && j%25000 == 1) {
            rtc_set_time(rtc_h(), rtc_m()+1, 0);
        }
        if (value & 0x4000 && j%25000 == 1) {
            if (rtc_m() == 0)
                rtc_set_time(rtc_h()-1, 59, 0);
            else
                rtc_set_time(rtc_h(), rtc_m()-1, 0);
        }
        j += 1;

        usb_voltage = read_usb_voltage();
        if (usb_voltage < 4700 && light_config.max_brightness > 0) {
            light_config.max_brightness -= 1;
        } else if (usb_voltage > 4800 && light_config.max_brightness < 255) {
            light_config.max_brightness += 1;
        }

        if (serial_connected && j%1000000 == 0) {
            printf("USB Voltage: %dmV\r\n", usb_voltage);
            int32_t temperature = read_temp();
            printf("Temperature: %d.%.3dC\r\n", temperature/1000, temperature%1000);
        }
    }

    return 0;
}
