#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_vfs.h>
#include <esp_spiffs.h>
#include "tft.h"

int xsize, ysize;

void LineTest()
{
    uint16_t color;
    tft_clear(COLOR_BLACK);
    color = COLOR_RED;
    for (int ypos = 0; ypos < ysize; ypos = ypos+10) {
        tft_line(color, 0, ypos, xsize, ypos);
    }

    for (int xpos = 0; xpos < xsize; xpos = xpos+10) {
        tft_line(color, xpos, 0, xpos, ysize);
    }
}

void RoundRectTest()
{
    uint16_t limit = xsize;

    if (xsize > ysize)
        limit = ysize;
    tft_clear(COLOR_BLACK);

    for (int i = 5; i < limit; i += 5) {
        if (i > limit - i - 1)
            break;

        tft_round_rect(COLOR_BLUE, i, i, xsize - i - 1, ysize - i - 1, 10);
    }
}

void FillRectTest()
{
    tft_clear(COLOR_CYAN);
    for (int i = 1; i < 100; i++) {
        int red = rand() & 255;
        int green = rand() & 255;
        int blue = rand() & 255;
        int color = COLOR_RGB(red, green, blue);
        int xpos = rand() % xsize;
        int ypos = rand() % ysize;
        int size = rand() % (xsize / 5);

        tft_fill(color, xpos, ypos, xpos + size, ypos + size);
    }
}

void ColorTest()
{
    static const int color[16] = {
        COLOR_RGB(255, 0,   0),     // red
        COLOR_RGB(255, 255, 0),     // yellow
        COLOR_RGB(0,   255, 0),     // green
        COLOR_RGB(0,   255, 255),   // cyan
        COLOR_RGB(0,   0,   255),   // blue
        COLOR_RGB(255, 127, 0),
        COLOR_RGB(127, 255, 0),
        COLOR_RGB(0,   255, 127),
        COLOR_RGB(0,   127, 255),
        COLOR_RGB(127, 0,   255),
        COLOR_RGB(255, 0,   127),
        COLOR_RGB(255, 127, 127),
        COLOR_RGB(127, 255, 127),
        COLOR_RGB(127, 127, 255),
        COLOR_RGB(127, 127, 127),   // gray
        COLOR_RGB(255, 255, 255),   // white
    };
    uint16_t delta = xsize/16;
    uint16_t xpos = 0;

    tft_clear(COLOR_WHITE);
    for (int i = 0; i < 16; i++) {
        tft_fill(color[i], xpos, 0, xpos+delta, ysize-1);
        xpos += delta;
    }
}

void RGBTest()
{
    uint16_t delta = xsize/16;
    uint16_t xpos = 0;

    tft_clear(COLOR_BLACK);
    for (int i = 0; i < 16; i++) {
        tft_fill(0x8000 >> i, xpos, 0, xpos+delta, ysize-1);
        xpos += delta;
    }
}

void app_main(void)
{
    printf("Draw fonts.\n");
    //setup_pins();

    if (tft_init(0, 0, &xsize, &ysize) < 0)
        return;

    srand((unsigned) time(NULL));
    for (;;) {
        RGBTest();
        vTaskDelay(200);

        ColorTest();
        vTaskDelay(200);

        LineTest();
        vTaskDelay(200);

        RoundRectTest();
        vTaskDelay(200);

        FillRectTest();
        vTaskDelay(200);
    }
}
