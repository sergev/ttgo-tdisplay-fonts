/*
 * Interface to a color TFT display based on ST7789 controller.
 * based on sources from https://github.com/nopnop2002/esp-idf-st7789
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
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include "st7789.h"

//
// Configuration for TTGO T-Display board.
//
#define CONFIG_WIDTH        135
#define CONFIG_HEIGHT       240
#define CONFIG_OFFSETX      52
#define CONFIG_OFFSETY      40
#define CONFIG_MOSI_GPIO    19
#define CONFIG_SCLK_GPIO    18
#define CONFIG_CS_GPIO      5
#define CONFIG_DC_GPIO      16
#define CONFIG_RESET_GPIO   23
#define CONFIG_BL_GPIO      4

TFT_t tft;

static inline void tft_send(const uint8_t *data, size_t nbytes)
{
    spi_transaction_t io = {
        .tx_buffer  = data,
        .length     = nbytes * 8,
    };

    if (nbytes > 0) {
        if (spi_device_transmit(tft.spidev, &io) != ESP_OK) {
            //TODO: print error message.
            return;
        }
    }
}

static void tft_send_command(uint8_t cmd)
{
    gpio_set_level(CONFIG_DC_GPIO, 0); // command mode
    tft_send(&cmd, 1);
}

static void tft_send_byte(uint8_t data)
{
    gpio_set_level(CONFIG_DC_GPIO, 1); // data mode
    tft_send(&data, 1);
}

static void tft_send_word(uint16_t word)
{
    uint8_t data[2] = {
        (uint8_t)(word >> 8),
        (uint8_t)word,
    };
    gpio_set_level(CONFIG_DC_GPIO, 1); // data mode
    tft_send(data, 2);
}

static void tft_send_addr(uint16_t addr1, uint16_t addr2)
{
    uint8_t data[4] = {
        (uint8_t)(addr1 >> 8),
        (uint8_t)addr1,
        (uint8_t)(addr2 >> 8),
        (uint8_t)addr2,
    };
    gpio_set_level(CONFIG_DC_GPIO, 1); // data mode
    tft_send(data, 4);
}

static void tft_send_color(uint16_t color, uint16_t size)
{
    uint8_t data[size * 2];
    int index = 0;

    for (int i=0; i<size; i++) {
        data[index++] = (uint8_t)(color >> 8);
        data[index++] = (uint8_t)color;
    }
    gpio_set_level(CONFIG_DC_GPIO, 1); // data mode
    tft_send(data, size * 2);
}

static inline void mdelay(int ms)
{
    vTaskDelay((ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS);
}

//
// Initialize the display.
//
int tft_init(int portrait, int color, int *xsize, int *ysize)
{
    *xsize = CONFIG_WIDTH;
    *ysize = CONFIG_HEIGHT;

    gpio_pad_select_gpio(CONFIG_CS_GPIO);
    gpio_set_direction(CONFIG_CS_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_CS_GPIO, 0);

    gpio_pad_select_gpio(CONFIG_DC_GPIO);
    gpio_set_direction(CONFIG_DC_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_DC_GPIO, 0);

    gpio_pad_select_gpio(CONFIG_RESET_GPIO);
    gpio_set_direction(CONFIG_RESET_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_RESET_GPIO, 1);
    mdelay(50);
    gpio_set_level(CONFIG_RESET_GPIO, 0);
    mdelay(50);
    gpio_set_level(CONFIG_RESET_GPIO, 1);
    mdelay(50);

    gpio_set_direction(CONFIG_BL_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_BL_GPIO, 0);

    spi_bus_config_t buscfg = {
        .sclk_io_num    = CONFIG_SCLK_GPIO,
        .mosi_io_num    = CONFIG_MOSI_GPIO,
        .miso_io_num    = -1,
        .quadwp_io_num  = -1,
        .quadhd_io_num  = -1
    };
    if (spi_bus_initialize(HSPI_HOST, &buscfg, 1) != ESP_OK) {
        // Cannot open SPI port.
        return -1;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_MASTER_FREQ_40M,
        .queue_size     = 7,
        .mode           = 2,
        .flags          = SPI_DEVICE_NO_DUMMY,
        .spics_io_num   = CONFIG_CS_GPIO,
    };
    if (spi_bus_add_device(HSPI_HOST, &devcfg, &tft.spidev) != ESP_OK) {
        // Cannot initialize SPI port.
        return -1;
    }

    tft._width = CONFIG_WIDTH;
    tft._height = CONFIG_HEIGHT;
    tft._offsetx = CONFIG_OFFSETX;
    tft._offsety = CONFIG_OFFSETY;

    tft_send_command(0x01);    //Power Control 1
    mdelay(150);

    tft_send_command(0x11);    //Power Control 2
    mdelay(255);

    tft_send_command(0x3A);    //VCOM Control 1
    tft_send_byte(0x55);
    mdelay(10);

    tft_send_command(0x36);    //VCOM Control 2
    tft_send_byte(0x00);

    tft_send_command(0x2A);    //Memory Access Control
    tft_send_byte(0x00);
    tft_send_byte(0x00);
    tft_send_byte(0x00);
    tft_send_byte(0xF0);

    tft_send_command(0x2B);    //Pixel Format Set
    tft_send_byte(0x00);
    tft_send_byte(0x00);
    tft_send_byte(0x00);
    tft_send_byte(0xF0);

    tft_send_command(0x21);    //Display Inversion OFF
    mdelay(10);

    tft_send_command(0x13);    //Frame Rate Control
    mdelay(10);

    tft_send_command(0x29);    //Display ON
    mdelay(255);

    gpio_set_level(CONFIG_BL_GPIO, 1);
    return 0;
}

// Draw pixel
// x:X coordinate
// y:Y coordinate
// color:color
void lcdDrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= tft._width) return;
    if (y >= tft._height) return;

    uint16_t _x = x + tft._offsetx;
    uint16_t _y = y + tft._offsety;

    tft_send_command(0x2A);    // set column(x) address
    tft_send_addr(_x, _x);
    tft_send_command(0x2B);    // set Page(y) address
    tft_send_addr(_y, _y);
    tft_send_command(0x2C);    //  Memory Write
    tft_send_word(color);
}

// Draw rectangle of filling
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End X coordinate
// y2:End Y coordinate
// color:color
void lcdDrawFillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    if (x1 >= tft._width) return;
    if (x2 >= tft._width) x2=tft._width-1;
    if (y1 >= tft._height) return;
    if (y2 >= tft._height) y2=tft._height-1;

    uint16_t _x1 = x1 + tft._offsetx;
    uint16_t _x2 = x2 + tft._offsetx;
    uint16_t _y1 = y1 + tft._offsety;
    uint16_t _y2 = y2 + tft._offsety;

    tft_send_command(0x2A);    // set column(x) address
    tft_send_addr(_x1, _x2);
    tft_send_command(0x2B);    // set Page(y) address
    tft_send_addr(_y1, _y2);
    tft_send_command(0x2C);    // Memory Write

    for (int i=_x1; i<=_x2; i++) {
        uint16_t size = _y2 - _y1 + 1;
        tft_send_color(color, size);
    }
}

// Display OFF
void lcdDisplayOff()
{
    tft_send_command(0x28);    // Display off
}

// Display ON
void lcdDisplayOn()
{
    tft_send_command(0x29);    // Display on
}

// Fill screen
// color:color
void lcdFillScreen(uint16_t color)
{
    lcdDrawFillRect(0, 0, tft._width-1, tft._height-1, color);
}

// Draw line
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End X coordinate
// y2:End Y coordinate
// color:color
void lcdDrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    int i;
    int dx,dy;
    int sx,sy;
    int E;

    /* distance between two points */
    dx = (x2 > x1) ? x2 - x1 : x1 - x2;
    dy = (y2 > y1) ? y2 - y1 : y1 - y2;

    /* direction of two point */
    sx = (x2 > x1) ? 1 : -1;
    sy = (y2 > y1) ? 1 : -1;

    /* inclination < 1 */
    if (dx > dy) {
        E = -dx;
        for (i = 0 ; i <= dx ; i++) {
            lcdDrawPixel(x1, y1, color);
            x1 += sx;
            E += 2 * dy;
            if (E >= 0) {
            y1 += sy;
            E -= 2 * dx;
        }
    }

    /* inclination >= 1 */
    } else {
        E = -dy;
        for (i = 0 ; i <= dy ; i++) {
            lcdDrawPixel(x1, y1, color);
            y1 += sy;
            E += 2 * dx;
            if (E >= 0) {
                x1 += sx;
                E -= 2 * dy;
            }
        }
    }
}

// Draw rectangle
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End   X coordinate
// y2:End   Y coordinate
// color:color
void lcdDrawRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    lcdDrawLine(x1, y1, x2, y1, color);
    lcdDrawLine(x2, y1, x2, y2, color);
    lcdDrawLine(x2, y2, x1, y2, color);
    lcdDrawLine(x1, y2, x1, y1, color);
}

// Draw rectangle with angle
// xc:Center X coordinate
// yc:Center Y coordinate
// w:Width of rectangle
// h:Height of rectangle
// angle :Angle of rectangle
// color :color

//When the origin is (0, 0), the point (x1, y1) after rotating the point (x, y) by the angle is obtained by the following calculation.
// x1 = x * cos(angle) - y * sin(angle)
// y1 = x * sin(angle) + y * cos(angle)
void lcdDrawRectAngle(uint16_t xc, uint16_t yc, uint16_t w, uint16_t h, uint16_t angle, uint16_t color)
{
    double xd,yd,rd;
    int x1,y1;
    int x2,y2;
    int x3,y3;
    int x4,y4;
    rd = -angle * M_PI / 180.0;
    xd = 0.0 - w/2;
    yd = h/2;
    x1 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
    y1 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

    yd = 0.0 - yd;
    x2 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
    y2 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

    xd = w/2;
    yd = h/2;
    x3 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
    y3 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

    yd = 0.0 - yd;
    x4 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
    y4 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

    lcdDrawLine(x1, y1, x2, y2, color);
    lcdDrawLine(x1, y1, x3, y3, color);
    lcdDrawLine(x2, y2, x4, y4, color);
    lcdDrawLine(x3, y3, x4, y4, color);
}

// Draw triangle
// xc:Center X coordinate
// yc:Center Y coordinate
// w:Width of triangle
// h:Height of triangle
// angle :Angle of triangle
// color :color

//When the origin is (0, 0), the point (x1, y1) after rotating the point (x, y) by the angle is obtained by the following calculation.
// x1 = x * cos(angle) - y * sin(angle)
// y1 = x * sin(angle) + y * cos(angle)
void lcdDrawTriangle(uint16_t xc, uint16_t yc, uint16_t w, uint16_t h, uint16_t angle, uint16_t color)
{
    double xd,yd,rd;
    int x1,y1;
    int x2,y2;
    int x3,y3;
    rd = -angle * M_PI / 180.0;
    xd = 0.0;
    yd = h/2;
    x1 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
    y1 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

    xd = w/2;
    yd = 0.0 - yd;
    x2 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
    y2 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

    xd = 0.0 - w/2;
    x3 = (int)(xd * cos(rd) - yd * sin(rd) + xc);
    y3 = (int)(xd * sin(rd) + yd * cos(rd) + yc);

    lcdDrawLine(x1, y1, x2, y2, color);
    lcdDrawLine(x1, y1, x3, y3, color);
    lcdDrawLine(x2, y2, x3, y3, color);
}

// Draw circle
// x0:Central X coordinate
// y0:Central Y coordinate
// r:radius
// color:color
void lcdDrawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color)
{
    int x;
    int y;
    int err;
    int old_err;

    x=0;
    y=-r;
    err=2-2*r;
    do {
        lcdDrawPixel(x0-x, y0+y, color);
        lcdDrawPixel(x0-y, y0-x, color);
        lcdDrawPixel(x0+x, y0-y, color);
        lcdDrawPixel(x0+y, y0+x, color);
        if ((old_err=err)<=x)   err+=++x*2+1;
        if (old_err>y || err>x) err+=++y*2+1;
    } while (y < 0);
}

// Draw circle of filling
// x0:Central X coordinate
// y0:Central Y coordinate
// r:radius
// color:color
void lcdDrawFillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color)
{
    int x;
    int y;
    int err;
    int old_err;
    int ChangeX;

    x=0;
    y=-r;
    err=2-2*r;
    ChangeX=1;
    do {
        if (ChangeX) {
            lcdDrawLine(x0-x, y0-y, x0-x, y0+y, color);
            lcdDrawLine(x0+x, y0-y, x0+x, y0+y, color);
        } // endif
        ChangeX=(old_err=err)<=x;
        if (ChangeX)            err+=++x*2+1;
        if (old_err>y || err>x) err+=++y*2+1;
    } while (y <= 0);
}

// Draw rectangle with round corner
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End   X coordinate
// y2:End   Y coordinate
// r:radius
// color:color
void lcdDrawRoundRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t r, uint16_t color)
{
    int x;
    int y;
    int err;
    int old_err;
    unsigned char temp;

    if (x1>x2) {
        temp=x1; x1=x2; x2=temp;
    } // endif

    if (y1>y2) {
        temp=y1; y1=y2; y2=temp;
    } // endif

    if (x2-x1 < r) return; // Add 20190517
    if (y2-y1 < r) return; // Add 20190517

    x=0;
    y=-r;
    err=2-2*r;

    do {
        if (x) {
            lcdDrawPixel(x1+r-x, y1+r+y, color);
            lcdDrawPixel(x2-r+x, y1+r+y, color);
            lcdDrawPixel(x1+r-x, y2-r-y, color);
            lcdDrawPixel(x2-r+x, y2-r-y, color);
        } // endif
        if ((old_err=err)<=x)   err+=++x*2+1;
        if (old_err>y || err>x) err+=++y*2+1;
    } while (y < 0);

    lcdDrawLine(x1+r,y1  ,x2-r,y1  ,color);
    lcdDrawLine(x1+r,y2  ,x2-r,y2  ,color);
    lcdDrawLine(x1  ,y1+r,x1  ,y2-r,color);
    lcdDrawLine(x2  ,y1+r,x2  ,y2-r,color);
}

// Draw arrow
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End   X coordinate
// y2:End   Y coordinate
// w:Width of the botom
// color:color
// Thanks http://k-hiura.cocolog-nifty.com/blog/2010/11/post-2a62.html
void lcdDrawArrow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint16_t w,uint16_t color)
{
    double Vx= x1 - x0;
    double Vy= y1 - y0;
    double v = sqrt(Vx*Vx+Vy*Vy);
    //   printf("v=%f\n",v);
    double Ux= Vx/v;
    double Uy= Vy/v;

    uint16_t L[2],R[2];
    L[0]= x1 - Uy*w - Ux*v;
    L[1]= y1 + Ux*w - Uy*v;
    R[0]= x1 + Uy*w - Ux*v;
    R[1]= y1 - Ux*w - Uy*v;
    //   printf("L=%d-%d R=%d-%d\n",L[0],L[1],R[0],R[1]);

    //   lcdDrawLine(x0,y0,x1,y1,color);
    lcdDrawLine(x1, y1, L[0], L[1], color);
    lcdDrawLine(x1, y1, R[0], R[1], color);
    lcdDrawLine(L[0], L[1], R[0], R[1], color);
}

// Draw arrow of filling
// x1:Start X coordinate
// y1:Start Y coordinate
// x2:End   X coordinate
// y2:End   Y coordinate
// w:Width of the botom
// color:color
void lcdDrawFillArrow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint16_t w,uint16_t color)
{
    double Vx= x1 - x0;
    double Vy= y1 - y0;
    double v = sqrt(Vx*Vx+Vy*Vy);
    double Ux= Vx/v;
    double Uy= Vy/v;

    uint16_t L[2], R[2];
    L[0] = x1 - Uy*w - Ux*v;
    L[1] = y1 + Ux*w - Uy*v;
    R[0] = x1 + Uy*w - Ux*v;
    R[1] = y1 - Ux*w - Uy*v;

    lcdDrawLine(x0, y0, x1, y1, color);
    lcdDrawLine(x1, y1, L[0], L[1], color);
    lcdDrawLine(x1, y1, R[0], R[1], color);
    lcdDrawLine(L[0], L[1], R[0], R[1], color);

    int ww;
    for (ww = w-1; ww > 0; ww--) {
        L[0]= x1 - Uy*ww - Ux*v;
        L[1]= y1 + Ux*ww - Uy*v;
        R[0]= x1 + Uy*ww - Ux*v;
        R[1]= y1 - Ux*ww - Uy*v;

        lcdDrawLine(x1, y1, L[0], L[1], color);
        lcdDrawLine(x1, y1, R[0], R[1], color);
    }
}

// RGB565 conversion
// RGB565 is R(5)+G(6)+B(5)=16bit color format.
// Bit image "RRRRRGGGGGGBBBBB"
uint16_t rgb565_conv(uint16_t r,uint16_t g,uint16_t b)
{
    return (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

// Backlight OFF
void lcdBacklightOff()
{
    gpio_set_level(CONFIG_BL_GPIO, 0);
}

// Backlight ON
void lcdBacklightOn()
{
    gpio_set_level(CONFIG_BL_GPIO, 1);
}

// Display Inversion ON
void lcdInversionOn()
{
    tft_send_command(0x21);    // Display Inversion ON
}
