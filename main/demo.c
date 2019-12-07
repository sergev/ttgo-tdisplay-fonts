/*
 * Demo of fonts on TFT display.
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
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "tft.h"

//
// TTGO T-Display board
//
#define PIN_BACKLIGHT   4   // io4 - backlight
#define PIN_BUTTON1     0   // io0 - button S1
#define PIN_BUTTON2     35  // io35 - button S2

//
// Font collection.
//
extern tft_font_t font_5x7;
extern tft_font_t font_6x9;
extern tft_font_t font_digits20;
extern tft_font_t font_digits32;

extern tft_font_t font_verdana8;
extern tft_font_t font_verdana10;
extern tft_font_t font_verdana12;
extern tft_font_t font_verdana14;
extern tft_font_t font_verdana18;

extern tft_font_t font_tahoma8;
extern tft_font_t font_tahoma10;
extern tft_font_t font_tahoma12;
extern tft_font_t font_tahoma14;
extern tft_font_t font_tahoma18;

extern tft_font_t font_georgia8;
extern tft_font_t font_georgia10;
extern tft_font_t font_georgia12;
extern tft_font_t font_georgia14;
extern tft_font_t font_georgia18;

extern tft_font_t font_lucidasans8;
extern tft_font_t font_lucidasans10;
extern tft_font_t font_lucidasans12;
extern tft_font_t font_lucidasans14;
extern tft_font_t font_lucidasans18;
extern tft_font_t font_lucidasans19;
extern tft_font_t font_lucidasans24;
extern tft_font_t font_lucidasans28;

extern tft_font_t font_lucidabright8;
extern tft_font_t font_lucidabright10;
extern tft_font_t font_lucidabright12;
extern tft_font_t font_lucidabright14;
extern tft_font_t font_lucidabright18;
extern tft_font_t font_lucidabright19;
extern tft_font_t font_lucidabright24;

extern tft_font_t font_courier8;
extern tft_font_t font_courier10;
extern tft_font_t font_courier12;
extern tft_font_t font_courier14;
extern tft_font_t font_courier18;
extern tft_font_t font_courier24;

extern tft_font_t font_helvetica8;
extern tft_font_t font_helvetica10;
extern tft_font_t font_helvetica12;
extern tft_font_t font_helvetica14;
extern tft_font_t font_helvetica18;
extern tft_font_t font_helvetica24;

extern tft_font_t font_times8;
extern tft_font_t font_times10;
extern tft_font_t font_times12;
extern tft_font_t font_times14;
extern tft_font_t font_times18;
extern tft_font_t font_times24;

//
// Screen size.
//
int xsize, ysize;

void setup_pins()
{
    // Backlight pin: output.
    gpio_pad_select_gpio(PIN_BACKLIGHT);
    gpio_set_direction(PIN_BACKLIGHT, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_BACKLIGHT, 1);

    // User button pins: input.
    gpio_pad_select_gpio(PIN_BUTTON1);
    gpio_set_direction(PIN_BUTTON1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_BUTTON1, GPIO_PULLUP_ONLY);

    gpio_pad_select_gpio(PIN_BUTTON2);
    gpio_set_direction(PIN_BUTTON2, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_BUTTON2, GPIO_PULLUP_ONLY);
}

void show(const tft_font_t *font, const char *title, int digits_only)
{
    int x = 0, y = 0, i;
    const char *phrase = digits_only ? "0123456789" :
                         "The quick brown fox jumps over the lazy dog.";

    tft_clear(0);
    tft_text(&font_tahoma10, 1, 0, x, y, title);
    y += font_tahoma10.height;

    for (i=0; y<ysize; i++) {
        tft_text(font, 1, 0, x, y, phrase);
        y += font->height;
    }
    tft_update();

    // Wait for user button pressed (active low).
    while (gpio_get_level(PIN_BUTTON1))
        vTaskDelay(1);
    gpio_set_level(PIN_BACKLIGHT, 0);

    // Wait until user button released.
    while (!gpio_get_level(PIN_BUTTON1))
        vTaskDelay(1);
    gpio_set_level(PIN_BACKLIGHT, 1);
}

void app_main()
{
    printf("Draw fonts.\n");
    setup_pins();

    if (tft_init(0, 0, &xsize, &ysize) < 0)
        return;

    for (;;) {
        printf("Screen size %u x %u.\n", xsize, ysize);

        show(&font_lucidasans8, "Lucida Sans 8", 0);
        show(&font_lucidasans10, "Lucida Sans 10", 0);
        show(&font_lucidasans12, "Lucida Sans 12", 0);
        show(&font_lucidasans14, "Lucida Sans 14", 0);
        show(&font_lucidasans18, "Lucida Sans 18", 0);
        show(&font_lucidasans19, "Lucida Sans 19", 0);
        show(&font_lucidasans24, "Lucida Sans 24", 0);
        show(&font_lucidasans28, "Lucida Sans 28", 0);

        show(&font_lucidabright8, "Lucida Bright 8", 0);
        show(&font_lucidabright10, "Lucida Bright 10", 0);
        show(&font_lucidabright12, "Lucida Bright 12", 0);
        show(&font_lucidabright14, "Lucida Bright 14", 0);
        show(&font_lucidabright18, "Lucida Bright 18", 0);
        show(&font_lucidabright19, "Lucida Bright 19", 0);
        show(&font_lucidabright24, "Lucida Bright 24", 0);

        show(&font_tahoma8, "Tahoma 8", 0);
        show(&font_tahoma10, "Tahoma 10", 0);
        show(&font_tahoma12, "Tahoma 12", 0);
        show(&font_tahoma14, "Tahoma 14", 0);
        show(&font_tahoma18, "Tahoma 18", 0);

        show(&font_verdana8, "Verdana 8", 0);
        show(&font_verdana10, "Verdana 10", 0);
        show(&font_verdana12, "Verdana 12", 0);
        show(&font_verdana14, "Verdana 14", 0);
        show(&font_verdana18, "Verdana 18", 0);

        show(&font_georgia8, "Georgia 8", 0);
        show(&font_georgia10, "Georgia 10", 0);
        show(&font_georgia12, "Georgia 12", 0);
        show(&font_georgia14, "Georgia 14", 0);
        show(&font_georgia18, "Georgia 18", 0);

        show(&font_courier8, "Courier 8", 0);
        show(&font_courier10, "Courier 10", 0);
        show(&font_courier12, "Courier 12", 0);
        show(&font_courier14, "Courier 14", 0);
        show(&font_courier18, "Courier 18", 0);
        show(&font_courier24, "Courier 24", 0);

        show(&font_helvetica8, "Helvetica 8", 0);
        show(&font_helvetica10, "Helvetica 10", 0);
        show(&font_helvetica12, "Helvetica 12", 0);
        show(&font_helvetica14, "Helvetica 14", 0);
        show(&font_helvetica18, "Helvetica 18", 0);
        show(&font_helvetica24, "Helvetica 24", 0);

        show(&font_times8, "Times 8", 0);
        show(&font_times10, "Times 10", 0);
        show(&font_times12, "Times 12", 0);
        show(&font_times14, "Times 14", 0);
        show(&font_times18, "Times 18", 0);
        show(&font_times24, "Times 24", 0);

        show(&font_5x7, "5x7", 0);
        show(&font_6x9, "6x9", 0);

        show(&font_digits20, "Digits 20", 1);
        show(&font_digits32, "Digits 32", 1);
    }
}
