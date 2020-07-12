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

#ifndef __LIGHT_H__
#define __LIGHT_H__

#include <stddef.h>
#include <stdint.h>

#include "colorsys.h"

struct light_config {
    uint32_t spi_base;
    size_t n_leds;
    uint8_t max_brightness;
    uint32_t* led_state;
};

void light_init(struct light_config* config, size_t n_leds);
void light_set(struct light_config* config, size_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w);
void light_set_hsv(struct light_config* config, size_t n, uint32_t h, uint16_t s, uint8_t v);
void light_update(struct light_config* config);
void light_clear(struct light_config* config);

#endif