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

#include "st7789.h"

int xsize, ysize;

void FillTest()
{
    lcdFillScreen(RED);
    vTaskDelay(50);
    lcdFillScreen(GREEN);
    vTaskDelay(50);
    lcdFillScreen(BLUE);
    vTaskDelay(50);
}

void ColorBarTest()
{
    if (xsize < ysize) {
        uint16_t y1,y2;
        y1 = ysize/3;
        y2 = (ysize/3)*2;
        lcdDrawFillRect(0, 0, xsize-1, y1-1, RED);
        vTaskDelay(1);
        lcdDrawFillRect(0, y1-1, xsize-1, y2-1, GREEN);
        vTaskDelay(1);
        lcdDrawFillRect(0, y2-1, xsize-1, ysize-1, BLUE);
    } else {
        uint16_t x1,x2;
        x1 = xsize/3;
        x2 = (xsize/3)*2;
        lcdDrawFillRect(0, 0, x1-1, ysize-1, RED);
        vTaskDelay(1);
        lcdDrawFillRect(x1-1, 0, x2-1, ysize-1, GREEN);
        vTaskDelay(1);
        lcdDrawFillRect(x2-1, 0, xsize-1, ysize-1, BLUE);
    }
}

void LineTest()
{
    uint16_t color;
    lcdFillScreen(BLACK);
    color = RED;
    for (int ypos = 0; ypos < ysize; ypos = ypos+10) {
        lcdDrawLine(0, ypos, xsize, ypos, color);
    }

    for (int xpos = 0; xpos < xsize; xpos = xpos+10) {
        lcdDrawLine(xpos, 0, xpos, ysize, color);
    }
}

void CircleTest()
{
    uint16_t color;
    lcdFillScreen(BLACK);
    color = CYAN;
    uint16_t xpos = xsize/2;
    uint16_t ypos = ysize/2;
    for (int i = 5; i < ysize; i = i+5) {
        lcdDrawCircle(xpos, ypos, i, color);
    }
}

void RectAngleTest()
{
    uint16_t color;
    lcdFillScreen(BLACK);
    color = CYAN;
    uint16_t xpos = xsize/2;
    uint16_t ypos = ysize/2;

    uint16_t w = xsize * 0.6;
    uint16_t h = w * 0.5;
    int angle;
    for (angle = 0; angle <= 360*3; angle = angle+30) {
        lcdDrawRectAngle(xpos, ypos, w, h, angle, color);
        usleep(10000);
        lcdDrawRectAngle(xpos, ypos, w, h, angle, BLACK);
    }

    for (angle = 0; angle <= 180; angle = angle+30) {
        lcdDrawRectAngle(xpos, ypos, w, h, angle, color);
    }
}

void TriangleTest()
{
    uint16_t color;
    lcdFillScreen(BLACK);
    color = CYAN;
    uint16_t xpos = xsize/2;
    uint16_t ypos = ysize/2;

    uint16_t w = xsize * 0.6;
    uint16_t h = w * 1.0;
    int angle;

    for (angle = 0; angle <= 360*3; angle = angle+30) {
        lcdDrawTriangle(xpos, ypos, w, h, angle, color);
        usleep(10000);
        lcdDrawTriangle(xpos, ypos, w, h, angle, BLACK);
    }

    for (angle = 0; angle <= 360; angle = angle+30) {
        lcdDrawTriangle(xpos, ypos, w, h, angle, color);
    }
}

void RoundRectTest()
{
    uint16_t color;
    uint16_t limit = xsize;
    if (xsize > ysize)
        limit = ysize;

    lcdFillScreen(BLACK);
    color = BLUE;
    for (int i = 5; i < limit; i = i+5) {
        if (i > limit - i - 1)
            break;
        lcdDrawRoundRect(i, i, (xsize-i-1), (ysize-i-1), 10, color);
    }
}

void FillRectTest()
{
    uint16_t color;
    lcdFillScreen(CYAN);

    uint16_t red;
    uint16_t green;
    uint16_t blue;
    srand((unsigned int) time(NULL));
    for (int i = 1; i < 100; i++) {
        red = rand()%255;
        green = rand()%255;
        blue = rand()%255;
        color = rgb565_conv(red, green, blue);
        uint16_t xpos = rand()%xsize;
        uint16_t ypos = rand()%ysize;
        uint16_t size = rand()%(xsize/5);
        lcdDrawFillRect(xpos, ypos, xpos+size, ypos+size, color);
    }
}

void ColorTest()
{
    uint16_t color;
    lcdFillScreen(WHITE);
    color = RED;
    uint16_t delta = ysize/16;
    uint16_t ypos = 0;
    for (int i = 0; i < 16; i++) {
        lcdDrawFillRect(0, ypos, xsize-1, ypos+delta, color);
        color = color >> 1;
        ypos = ypos + delta;
    }
}

void app_main(void)
{
    printf("Draw fonts.\n");
    //setup_pins();

    if (tft_init(0, 0, &xsize, &ysize) < 0)
        return;

    for (;;) {
        FillTest();
        vTaskDelay(200);

        ColorBarTest();
        vTaskDelay(200);

        LineTest();
        vTaskDelay(200);

        CircleTest();
        vTaskDelay(200);

        RoundRectTest();
        vTaskDelay(200);

        RectAngleTest();
        vTaskDelay(200);

        TriangleTest();
        vTaskDelay(200);

        FillRectTest();
        vTaskDelay(200);

        ColorTest();
        vTaskDelay(200);
    }
}
