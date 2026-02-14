/* Core/Src/lcd_display.c — LCD display module (Mama board, ST7735 160x80 on SPI2) */
#include "lcd_display.h"
#include "user_config.h"
#include "board_id.h"
#include "node_id.h"
#include "w5500.h"
#include "app.h"
#include "mqtt_client.h"
#include "main.h"
#include "ST7735Canvas.h"
#include <stdio.h>
#include <string.h>

/* PROGMEM is an AVR attribute — empty on ARM */
#ifndef PROGMEM
#define PROGMEM
#endif

/* Fonts */
#include "Dialog10pNar.h"
#include "Dialog24p.h"

/* ---------- Hardware pins (from main.h) ---------- */
/*
 * SPI2:       PB13 (SCK), PB15 (MOSI)
 * LCD_nCS:    PB12 (active low)
 * LCD_AO:     PB11 (Data/Command, a.k.a. DC)
 * LCD_nRST:   PB10 (active low)
 * nLCD_BL:    PA3  (backlight, active low)
 */

/* ---------- Display parameters ---------- */
#define LCD_WIDTH        160
#define LCD_HEIGHT        80
#define LCD_ROTATION       1     /* landscape: 160 wide x 80 tall */
#define LCD_OPTIONS        INITR_MINI160x80_PLUGIN

#define LCD_UPDATE_MS    250     /* minimum ms between redraws */

/* ---------- Colors (RGB565) ---------- */
#define COL_BG           0x0000  /* black */
#define COL_STATUS_BG    0x18E3  /* dark gray bar */
#define COL_STATUS_FG    0xBDF7  /* light gray text */
#define COL_NAME_FG      0x07FF  /* cyan, matches web UI accent */
#define COL_APP_RUN      0x07E0  /* green */
#define COL_APP_STOP     0xFD20  /* orange */
#define COL_APP_EMPTY    0x4208  /* dim gray */
#define COL_MQTT_ON      0x07E0  /* green */
#define COL_MQTT_OFF     0x8410  /* gray */

/* ---------- State ---------- */
extern SPI_HandleTypeDef hspi2;

static LCD_HandleTypeDef lcd_dev;
static uint32_t last_update_tick;
static bool lcd_active;

/* ---------- Helpers ---------- */
static void lcd_backlight_on(void)
{
    /* nLCD_BL is active-low */
    HAL_GPIO_WritePin(nLCD_BL_GPIO_Port, nLCD_BL_Pin, GPIO_PIN_RESET);
}

static void lcd_backlight_off(void)
{
    HAL_GPIO_WritePin(nLCD_BL_GPIO_Port, nLCD_BL_Pin, GPIO_PIN_SET);
}

/* Draw a text string at (x,y) using the device's current font/colors */
static void lcd_print_at(uint16_t x, uint16_t y, const char *str)
{
    ST7735_SetCursor(&lcd_dev, x, y);
    ST7735_Print(&lcd_dev, str);
}

/* ---------- Init ---------- */
void lcd_init(void)
{
    /* PA3 is configured as TIM5_CH4 alternate function by CubeMX.
     * Reconfigure as plain GPIO output so backlight on/off works. */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = nLCD_BL_Pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(nLCD_BL_GPIO_Port, &gpio);

    user_config_data_t *cfg = config_get();
    if (!cfg->lcd_enabled) {
        lcd_active = false;
        lcd_backlight_off();
        return;
    }

    ST7735_Init(&lcd_dev,
                LCD_WIDTH, LCD_HEIGHT,
                &hspi2,
                LCD_nCS_GPIO_Port,   LCD_nCS_Pin,
                LCD_AO_GPIO_Port,    LCD_AO_Pin,
                LCD_nRST_GPIO_Port,  LCD_nRST_Pin,
                LCD_ROTATION,
                LCD_OPTIONS,
                true);              /* finishInit = true */

    /* SetRotation must be called AFTER Init/Start (which resets rotation to 0) */
    ST7735_SetRotation(&lcd_dev, LCD_ROTATION);

    ST7735_FillScreen(&lcd_dev, COL_BG);
    lcd_backlight_on();

    lcd_active = true;
    last_update_tick = 0;  /* force immediate first draw */
}

/* ---------- Redraw ---------- */
static void lcd_redraw(void)
{
    user_config_data_t *cfg = config_get();
    char buf[32];

    /* ---- Status bar (top 14px) ---- */
    ST7735_FillRect(&lcd_dev, 0, 0, LCD_WIDTH, 14, COL_STATUS_BG);

    ST7735_SetFont(&lcd_dev, (GFXfont *)&Dialog_plain_10Nar);
    ST7735_SetTextColor(&lcd_dev, COL_STATUS_FG, COL_STATUS_BG);

    /* IP address — left side */
    uint8_t ip[4];
    w5500_get_ip(ip);
    snprintf(buf, sizeof(buf), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    lcd_print_at(2, 11, buf);

    /* Node ID — right side */
    snprintf(buf, sizeof(buf), "ID:%d", node_id_get());
    uint8_t tw = ST7735_GetTextWidth(&lcd_dev, buf);
    lcd_print_at(LCD_WIDTH - tw - 2, 11, buf);

    /* ---- Device name (large, centered) ---- */
    ST7735_FillRect(&lcd_dev, 0, 14, LCD_WIDTH, 30, COL_BG);

    ST7735_SetFont(&lcd_dev, (GFXfont *)&Dialog_plain_24);
    ST7735_SetTextColor(&lcd_dev, COL_NAME_FG, COL_BG);

    tw = ST7735_GetTextWidth(&lcd_dev, cfg->device_name);
    uint16_t name_x = (tw < LCD_WIDTH) ? (LCD_WIDTH - tw) / 2 : 0;
    lcd_print_at(name_x, 38, cfg->device_name);

    /* ---- App status row (y 46..58) ---- */
    ST7735_FillRect(&lcd_dev, 0, 44, LCD_WIDTH, 18, COL_BG);

    ST7735_SetFont(&lcd_dev, (GFXfont *)&Dialog_plain_10Nar);
    uint16_t ax = 2;
    for (uint8_t i = 0; i < NUM_APPS; i++) {
        uint16_t col;
        const char *label;
        if (apps[i].num_lines == 0) {
            col = COL_APP_EMPTY;
            label = "---";
        } else if (apps[i].running) {
            col = COL_APP_RUN;
            label = "Run";
        } else {
            col = COL_APP_STOP;
            label = "Stp";
        }
        snprintf(buf, sizeof(buf), "A%d:", i);
        ST7735_SetTextColor(&lcd_dev, COL_STATUS_FG, COL_BG);
        lcd_print_at(ax, 57, buf);
        ax += ST7735_GetTextWidth(&lcd_dev, buf) + 1;

        ST7735_SetTextColor(&lcd_dev, col, COL_BG);
        lcd_print_at(ax, 57, label);
        ax += ST7735_GetTextWidth(&lcd_dev, label) + 4;
    }

    /* ---- Bottom status row (y 62..76) ---- */
    ST7735_FillRect(&lcd_dev, 0, 62, LCD_WIDTH, 18, COL_BG);

    ST7735_SetFont(&lcd_dev, (GFXfont *)&Dialog_plain_10Nar);

    /* MQTT status */
    ST7735_SetTextColor(&lcd_dev, COL_STATUS_FG, COL_BG);
    lcd_print_at(2, 75, "MQTT:");

    bool mqtt_on = mqtt_client_is_connected();
    ST7735_SetTextColor(&lcd_dev, mqtt_on ? COL_MQTT_ON : COL_MQTT_OFF, COL_BG);
    lcd_print_at(32, 75, mqtt_on ? "On" : "Off");

    /* Network status — right side */
    ST7735_SetTextColor(&lcd_dev, COL_STATUS_FG, COL_BG);
    lcd_print_at(LCD_WIDTH - 42, 75, "Net:OK");
}

/* ---------- Task (called from super-loop) ---------- */
void lcd_task(void)
{
    if (!lcd_active) {
        /* Check if config changed and LCD was re-enabled */
        user_config_data_t *cfg = config_get();
        if (cfg->lcd_enabled) {
            lcd_init();
        }
        return;
    }

    uint32_t now = HAL_GetTick();
    if ((now - last_update_tick) < LCD_UPDATE_MS) {
        return;
    }
    last_update_tick = now;

    /* Check if LCD was disabled at runtime */
    user_config_data_t *cfg = config_get();
    if (!cfg->lcd_enabled) {
        lcd_backlight_off();
        lcd_active = false;
        return;
    }

    lcd_redraw();
}
