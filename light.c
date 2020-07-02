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

#include "light.h"
#include "colorsys.h"

#ifndef STM32F1
#define STM32F1
#endif

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

void light_init(struct light_config* config, size_t n_leds) {
    // First, configure the SPI port
    rcc_periph_clock_enable(RCC_SPI1);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO7);
    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_16, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_LSBFIRST);
    spi_enable_software_slave_management(SPI1);
    spi_set_nss_high(SPI1);
    spi_enable(SPI1);

    // Set up the config structure
    config->spi_base = SPI1;
    config->n_leds = n_leds;
    config->max_brightness = 255;
    light_clear(config);

    // Write out the blank state
    light_update(config);
}

void light_set(struct light_config* config, size_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
    if (n > config->n_leds)
        return;

    union rgbw val;
    val.r = r;
    val.g = g;
    val.b = b;
    val.w = w;
    config->led_state[n] = val.rgbw;
}

void light_set_hls(struct light_config* config, size_t n, uint8_t h, uint8_t l, uint8_t s) {
    if (n > config->n_leds)
        return;

    union rgbw val = hls_to_rgb(h, l, s);
    config->led_state[n] = val.rgbw;
}

uint8_t light_code(uint8_t b) {
    if (b)
        return 0x03;
    return 0x01;
}

void light_update(struct light_config* config) {
    spi_send(config->spi_base, 0x00);
    for (size_t n = 0; n < config->n_leds; n++) {
        for (int8_t i = 30; i >= 0; i -= 2) {
            uint8_t code = (config->led_state[n] >> i);
            code = light_code(code & 0x02) | (light_code(code & 0x01) << 4);
            spi_send(config->spi_base, code);
        }
    }
    spi_send(config->spi_base, 0x00);
}

void light_clear(struct light_config* config) {
    for (size_t i = 0; i < config->n_leds; i++)
        config->led_state[i] = 0x00000000;
}
