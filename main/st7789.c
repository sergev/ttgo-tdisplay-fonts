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
#include "tft.h"

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

//
// ST7789V commands.
//
enum {
    Cmd_Software_Reset              = 0x01,
    Cmd_Sleep_Out                   = 0x11,
    Cmd_Partial_Mode_Off            = 0x13,
    Cmd_Display_Inversion_Off       = 0x20,
    Cmd_Display_Inversion_On        = 0x21,
    Cmd_Display_Off                 = 0x28,
    Cmd_Display_On                  = 0x29,
    Cmd_Column_Address_Set          = 0x2a,
    Cmd_Row_Address_Set             = 0x2b,
    Cmd_Memory_Write                = 0x2c,
    Cmd_Memory_Data_Access_Control  = 0x36,
    Cmd_Interface_Pixel_Format      = 0x3a,
};

tft_t tft;

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

    for (int i = 0; i < size; i++) {
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
    if (spi_bus_add_device(HSPI_HOST, &devcfg, (spi_device_handle_t*) &tft.spidev) != ESP_OK) {
        // Cannot initialize SPI port.
        return -1;
    }

    tft.width = CONFIG_WIDTH;
    tft.height = CONFIG_HEIGHT;
    tft.offsetx = CONFIG_OFFSETX;
    tft.offsety = CONFIG_OFFSETY;

    tft_send_command(Cmd_Software_Reset);
    mdelay(150);

    tft_send_command(Cmd_Sleep_Out);
    mdelay(255);

    tft_send_command(Cmd_Interface_Pixel_Format);
    tft_send_byte(0x55);
    mdelay(10);

    tft_send_command(Cmd_Memory_Data_Access_Control);
    tft_send_byte(0x00);

    tft_send_command(Cmd_Column_Address_Set);
    tft_send_byte(0x00);
    tft_send_byte(0x00);
    tft_send_byte(0x00);
    tft_send_byte(0xF0);

    tft_send_command(Cmd_Row_Address_Set);
    tft_send_byte(0x00);
    tft_send_byte(0x00);
    tft_send_byte(0x00);
    tft_send_byte(0xF0);

    tft_send_command(Cmd_Display_Inversion_On);
    mdelay(10);

    tft_send_command(Cmd_Partial_Mode_Off);
    mdelay(10);

    tft_send_command(Cmd_Display_On);
    mdelay(255);

    gpio_set_level(CONFIG_BL_GPIO, 1);
    return 0;
}

//
// Send updated image to the screen.
//
void tft_update(void)
{
    // Empty.
}

//
// Draw single pixel.
//
void tft_pixel(int color, int x, int y)
{
    if (x >= tft.width)
        return;
    if (y >= tft.height)
        return;

    x += tft.offsetx;
    y += tft.offsety;

    tft_send_command(Cmd_Column_Address_Set);
    tft_send_addr(x, x);
    tft_send_command(Cmd_Row_Address_Set);
    tft_send_addr(y, y);
    tft_send_command(Cmd_Memory_Write);
    tft_send_word(color);
}

//
// Fill rectangle with color.
//
void tft_fill(int color, int x1, int y1, int x2, int y2)
{
    if (x1 > x2) {
        int temp = x1;
        x1 = x2;
        x2 = temp;
    }

    if (y1 > y2) {
        int temp = y1;
        y1 = y2;
        y2 = temp;
    }

    if (x1 >= tft.width)
        return;
    if (y1 >= tft.height)
        return;

    if (x2 >= tft.width)
        x2 = tft.width - 1;
    if (y2 >= tft.height)
        y2 = tft.height - 1;

    x1 += tft.offsetx;
    x2 += tft.offsetx;
    y1 += tft.offsety;
    y2 += tft.offsety;

    tft_send_command(Cmd_Column_Address_Set);
    tft_send_addr(x1, x2);
    tft_send_command(Cmd_Row_Address_Set);
    tft_send_addr(y1, y2);
    tft_send_command(Cmd_Memory_Write);

    int size = y2 - y1 + 1;
    if (size > 1) {
        for (int i = x1; i <= x2; i++) {
            tft_send_color(color, size);
        }
    } else {
        tft_send_color(color, x2 - x1 + 1);
    }
}

//
// Fill screen with given color.
//
void tft_clear(int color)
{
    tft_fill(color, 0, 0, tft.width-1, tft.height-1);
}

//
// Enable or disable the display.
//
void tft_enable(int on)
{
    if (on) {
        tft_send_command(Cmd_Display_On);
    } else {
        tft_send_command(Cmd_Display_Off);
    }
}

//
// Enable or disable the backlight.
//
void tft_backlight(int on)
{
    gpio_set_level(CONFIG_BL_GPIO, on);
}

//
// Invert the display.
//
void tft_invert(int on)
{
    if (on) {
        tft_send_command(Cmd_Display_Inversion_On);
    } else {
        tft_send_command(Cmd_Display_Inversion_Off);
    }
}
