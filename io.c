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

#include "io.h"

void gpio_setup(void)
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
    adc_set_sample_time(ADC1, ADC_CHANNEL16, ADC_SMPR_SMP_239DOT5CYC);
    uint8_t channels[1] = {ADC_CHANNEL9};
    adc_set_regular_sequence(ADC1, 1, channels);
    adc_power_on(ADC1);
    adc_enable_temperature_sensor();
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
    uint32_t voltage = (3300 * (uint32_t)adc_val) / 0xFFF;
    voltage = (voltage * 1000000)/545454;
    return voltage;
}

int32_t read_temp(void) {
    uint16_t adc_val = read_adc(TEMP_CHANNEL);
    int32_t voltage = (3300 * (int32_t)adc_val) / 0xFFF;
    uint32_t temperature = ((1430 - voltage)*1000000 / 4300) + 25000;
    return temperature;
}
