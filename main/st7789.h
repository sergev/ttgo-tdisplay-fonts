#ifndef MAIN_ST7789_H_
#define MAIN_ST7789_H_

#include <driver/spi_master.h>

#define RED                     0xf800
#define GREEN                   0x07e0
#define BLUE                    0x001f
#define BLACK                   0x0000
#define WHITE                   0xffff
#define GRAY                    0x8c51
#define YELLOW                  0xFFE0
#define CYAN                    0x07FF
#define PURPLE                  0xF81F

#define DIRECTION0              0
#define DIRECTION90             1
#define DIRECTION180            2
#define DIRECTION270            3

typedef struct {
    uint16_t _width;
    uint16_t _height;
    uint16_t _offsetx;
    uint16_t _offsety;
    spi_device_handle_t spidev;
} TFT_t;

void delayMS(int ms);
int tft_init(int rotate, int color, int *xsize, int *ysize);
void lcdDrawPixel(uint16_t x, uint16_t y, uint16_t color);
void lcdDrawFillRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void lcdDisplayOff(void);
void lcdDisplayOn(void);
void lcdFillScreen(uint16_t color);
void lcdDrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void lcdDrawRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void lcdDrawRectAngle(uint16_t xc, uint16_t yc, uint16_t w, uint16_t h, uint16_t angle, uint16_t color);
void lcdDrawTriangle(uint16_t xc, uint16_t yc, uint16_t w, uint16_t h, uint16_t angle, uint16_t color);
void lcdDrawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
void lcdDrawFillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
void lcdDrawRoundRect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t r, uint16_t color);
void lcdDrawArrow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t w, uint16_t color);
void lcdDrawFillArrow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t w, uint16_t color);
uint16_t rgb565_conv(uint16_t r, uint16_t g, uint16_t b);
void lcdBacklightOff(void);
void lcdBacklightOn(void);
void lcdInversionOn(void);

#endif /* MAIN_ST7789_H_ */
