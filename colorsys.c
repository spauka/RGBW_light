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
 *
 * This conversion is based on: Integer-based accurate conversion between RGB and HSV color spaces
 * (https://doi.org/10.1016/j.compeleceng.2015.08.005)
 */

#include "colorsys.h"

union rgbw hsv_to_rgb(uint32_t h, uint16_t s, uint8_t v) {
    union rgbw res;
    res.w = 0;

    if (s == 0 || v == 0) {
        res.r = v;
        res.g = v;
        res.b = v;
        return res;
    }

    uint32_t delta = ((s * v) >> 16) + 1;
    uint8_t min = v - delta;
    uint8_t* mid;

    if (h >= hueEdgeLen * 4) {
        h -= hueEdgeLen * 4;
        if (h < hueEdgeLen) {
            res.b = v;
            res.g = min;
            mid = &res.r;
        } else {
            h -= hueEdgeLen;
            h = hueEdgeLen - h;
            res.r = v;
            res.g = min;
            mid = &res.b;
        }
    } else if (h >= hueEdgeLen * 2) {
        h -= hueEdgeLen * 2;
        if (h < hueEdgeLen) {
            res.g = v;
            res.r = min;
            mid = &res.b;
        } else {
            h -= hueEdgeLen;
            h = hueEdgeLen - h;
            res.b = v;
            res.r = min;
            mid = &res.g;
        }
    } else {
        if (h < hueEdgeLen) {
            res.r = v;
            res.b = min;
            mid = &res.g;
        } else {
            h -= hueEdgeLen;
            h = hueEdgeLen - h;
            res.g = v;
            res.b = min;
            mid = &res.r;
        }
    }

    *mid = ((h * delta) >> 16) + min;
    return res;
}
