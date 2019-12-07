/**
 * Interface to a color TFT display.
 *
 * Copyright (C) 2019 Serge Vakulenko
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *   3. The name of the author may not be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdint.h>
#include "tft.h"

//
// Draw one pixel
//
void tft_pixel(int color, int x, int y)
{
    uint32_t y_bit = (y & 7);

    /*
     * We only need to modify the Y coordinate since the pitch
     * of the screen is the same as the width.
     * Dividing y by 8 gives us which row the pixel is in but not
     * the bit position.
     */
    y >>= 3;

    uint8_t *ptr = tft.image + (y * tft.width) + x;
    if (color)
        *ptr |= 1 << y_bit;
    else
        *ptr &= ~(1 << y_bit);
}

//
// Fill rectangle.
//
void tft_fill(int color, int x0, int y0, int x1, int y1)
{
    if (x0 < 0) x0 = 0;
    if (y0 < 0) x0 = 0;
    if (x1 < 0) x1 = 0;
    if (y1 < 0) x1 = 0;
    if (x0 >= tft.width) x0 = tft.width-1;
    if (x1 >= tft.width) x1 = tft.width-1;
    if (y0 >= tft.height) y0 = tft.height-1;
    if (y1 >= tft.height) y1 = tft.height-1;

    if (x1 < x0) {
        int t = x0;
        x0 = x1;
        x1 = t;
    }
    if (y1 < y0) {
        int t = y0;
        y0 = y1;
        y1 = t;
    }

    int x, y;
    for (y = y0; y <= y1; y++) {
        for (x = x0; x <= x1; x++) {
            tft_pixel(color, x, y);
        }
    }
}

//
// Draw a line.
//
void tft_line(int color, int x0, int y0, int x1, int y1)
{
    int dx, dy, stepx, stepy, fraction;

    if (x0 == x1 || y0 == y1) {
        tft_fill(color, x0, y0, x1, y1);
        return;
    }

    // Use Bresenham's line algorithm.
    dy = y1 - y0;
    if (dy < 0) {
        dy = -dy;
        stepy = -1;
    } else {
        stepy = 1;
    }
    dx = x1 - x0;
    if (dx < 0) {
        dx = -dx;
        stepx = -1;
    } else {
        stepx = 1;
    }
    dy <<= 1;                           // dy is now 2*dy
    dx <<= 1;                           // dx is now 2*dx
    tft_pixel(color, x0, y0);
    if (dx > dy) {
        fraction = dy - (dx >> 1);      // same as 2*dy - dx
        while (x0 != x1) {
            if (fraction >= 0) {
                y0 += stepy;
                fraction -= dx;         // same as fraction -= 2*dx
            }
            x0 += stepx;
            fraction += dy;             // same as fraction -= 2*dy
            tft_pixel(color, x0, y0);
        }
    } else {
        fraction = dx - (dy >> 1);
        while (y0 != y1) {
            if (fraction >= 0) {
                x0 += stepx;
                fraction -= dy;
            }
            y0 += stepy;
            fraction += dx;
            tft_pixel(color, x0, y0);
        }
    }
}

//
// Draw a rectangular frame.
//
void tft_rect(int color, int x0, int y0, int x1, int y1)
{
    tft_fill(color, x0, y0, x1, y0);
    tft_fill(color, x0, y1, x1, y1);
    tft_fill(color, x0, y0, x0, y1);
    tft_fill(color, x1, y0, x1, y1);
}

//
// Swap values of two integer variables.
//
#define swapi(x,y) { int _t = x; x = y; y = _t; }

//
// Fill a triangle.
// Code ported from AdaFruit TFT LCD library.
//
void tft_fill_triangle(int color, int x0, int y0, int x1, int y1, int x2, int y2)
{
    int a, b, y, last;

    // Sort coordinates by Y order (y2 >= y1 >= y0)
    if (y0 > y1) {
        swapi(y0, y1);
        swapi(x0, x1);
    }
    if (y1 > y2) {
        swapi(y2, y1);
        swapi(x2, x1);
    }
    if (y0 > y1) {
        swapi(y0, y1);
        swapi(x0, x1);
    }

    if (y0 == y2) {
        // Handle awkward all-on-same-line case as its own thing
        a = b = x0;

        if (x1 < a)
            a = x1;
        else if (x1 > b)
            b = x1;

        if (x2 < a)
            a = x2;
        else if (x2 > b)
            b = x2;

        tft_fill(color, a, y0, b, y0);
        return;
    }

    int dx01 = x1 - x0;
    int dy01 = y1 - y0;
    int dx02 = x2 - x0;
    int dy02 = y2 - y0;
    int dx12 = x2 - x1;
    int dy12 = y2 - y1;
    int sa = 0;
    int sb = 0;

    // For upper part of triangle, find scanline crossings for segments
    // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
    // is included here (and second loop will be skipped, avoiding a /0
    // error there), otherwise scanline y1 is skipped here and handled
    // in the second loop...which also avoids a /0 error here if y0=y1
    // (flat-topped triangle).
    if (y1 == y2)
        last = y1;      // Include y1 scanline
    else
        last = y1-1;    // Skip it

    for (y=y0; y<=last; y++) {
        a   = x0 + sa / dy01;
        b   = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;

        /* longhand:
         * a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
         * b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
         */
        if (a > b)
            swapi(a, b);

        tft_fill(color, a, y, b, y);
    }

    // For lower part of triangle, find scanline crossings for segments
    // 0-2 and 1-2.  This loop is skipped if y1=y2.
    sa = dx12 * (y - y1);
    sb = dx02 * (y - y0);
    for (; y<=y2; y++) {
        a   = x1 + sa / dy12;
        b   = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;

        /* longhand:
         * a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
         * b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
         */
        if (a > b)
            swapi(a, b);

        tft_fill(color, a, y, b, y);
    }
}

//
// Draw a circle.
//
void tft_circle(int color, int x0, int y0, int radius)
{
    int f = 1 - radius;
    int ddF_x = 0;
    int ddF_y = -2 * radius;
    int x = 0;
    int y = radius;

    tft_pixel(color, x0, y0 + radius);
    tft_pixel(color, x0, y0 - radius);
    tft_pixel(color, x0 + radius, y0);
    tft_pixel(color, x0 - radius, y0);
    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x + 1;
        tft_pixel(color, x0 + x, y0 + y);
        tft_pixel(color, x0 - x, y0 + y);
        tft_pixel(color, x0 + x, y0 - y);
        tft_pixel(color, x0 - x, y0 - y);
        tft_pixel(color, x0 + y, y0 + x);
        tft_pixel(color, x0 - y, y0 + x);
        tft_pixel(color, x0 + y, y0 - x);
        tft_pixel(color, x0 - y, y0 - x);
    }
}

//
// Draw a glyph of one symbol.
//
static void tft_glyph(const tft_font_t *font,
    int color, int background, int x, int y, int width,
    const unsigned short *bits)
{
    int h, w;

    if (x + width > tft.width || y + font->height > tft.height)
        return;

    if (background >= 0) {
        // Update background.
        tft_fill(background, x, y, x + width - 1, y + font->height - 1);
    }

    // Loop on each glyph row.
    for (h=0; h<font->height; h++) {
        unsigned bitmask = 0;

        // Loop on every pixel in the row (left to right).
        for (w=0; w<width; w++) {
            if ((w & 15) == 0)
                bitmask = *bits++;
            else
                bitmask <<= 1;

            if (bitmask & 0x8000)
                tft_pixel(color, x + w, y + h);
        }
    }
}

//
// Draw a character from a specified font.
//
void tft_char(const tft_font_t *font,
    int color, int background, int x, int y, int sym)
{
    unsigned cindex, width;
    const unsigned short *bits;

    if (x >= 0)
        tft.col = x;
    if (y >= 0)
        tft.row = y;
    switch (sym) {
    case '\n':      // goto next line
        tft.row += font->height;
        tft.col = 0;
        if (tft.row > tft.height - font->height)
            tft.row = 0;
        return;
    case '\r':      // carriage return - go to begin of line
        tft.col = 0;
        return;
    case '\t':      // tab replaced by space
        sym = ' ';
        break;
    }

    if (sym < font->firstchar || sym >= font->firstchar + font->size)
        sym = font->defaultchar;
    cindex = sym - font->firstchar;

    // Get font bitmap depending on fixed pitch or not.
    if (font->width) {
        // Proportional font.
        width = font->width[cindex];
    } else {
        // Fixed width font.
        width = font->maxwidth;
    }
    if (font->offset) {
        bits = font->bits + font->offset[cindex];
    } else {
        bits = font->bits + cindex * font->height;
    }

    // Draw a character.
    tft_glyph(font, color, background, tft.col, tft.row, width, bits);
    tft.col += width;
}

//
// Draw a string of characters.
// TODO: Decode UTF-8.
//
void tft_text(const tft_font_t *font,
    int color, int background, int x, int y, const char *text)
{
    int sym;

    if (x >= 0)
        tft.col = x;
    if (y >= 0)
        tft.row = y;
    for (;;) {
        sym = *text++;
        if (! sym)
            break;

        tft_char(font, color, background, -1, -1, sym);
    }
}

//
// Return a width of the given symbol.
//
static int tft_char_width(const tft_font_t *font, int sym)
{
    switch (sym) {
    case '\n':      // goto next line
    case '\r':      // carriage return - go to begin of line
        return 0;
    case '\t':      // tab replaced by space
        sym = ' ';
        break;
    }

    if (!font->width) {
        // Fixed width font.
        return font->maxwidth;
    }

    // Proportional font.
    if (sym < font->firstchar || sym >= font->firstchar + font->size)
        sym = font->defaultchar;

    return font->width[sym - font->firstchar];
}

//
// Compute a width in pixels of a text string.
//
int tft_text_width(const tft_font_t *font, const char *text, int nchars)
{
    int width = 0;

    while (nchars-- > 0) {
        int sym = *text++;
        if (! sym)
            break;

        width += tft_char_width(font, sym);
    }
    return width;
}
