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

#include "colorsys.h"

static uint8_t _v(int16_t m1, int16_t m2, int16_t hue) {
    if (hue < 0)
        hue += 255;
    hue = hue % 255;
    int16_t val;
    if (hue <= 42) {
        val = m1 + (((m2-m1)*hue*6)/255);
        return val;
    } else if (hue < 128) {
        return m2;
    } else if (hue <= 170) {
        val = m1 + ((m2-m1)*(170-hue)*6)/255;
        return val;
    }
    return m1;
}

union rgbw hls_to_rgb(uint8_t h, uint8_t l, uint8_t s) {
    union rgbw res;
    int16_t m1 = 0, m2 = 0, hue = h;
    res.w = 0;
    if (s == 0) {
        res.r = l;
        res.g = l;
        res.b = l;
    }
    if (l <= 127) {
        m2 = l + ((l*s)>>8);
    } else {
        m2 = l + s - ((l*s)>>8);
    }
    if (m2 > 255)
        m2 = 255;
    m1 = 2*l - m2;
    res.r = _v(m1, m2, hue + 85);
    res.g = _v(m1, m2, hue);
    res.b = _v(m1, m2, hue - 85);
    return res;
}
