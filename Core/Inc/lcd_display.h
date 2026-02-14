/* Core/Inc/lcd_display.h — LCD display module (Mama board only, ST7735 160x80) */
#pragma once
#include <stdint.h>

void lcd_init(void);   /* Call during boot if Mama board */
void lcd_task(void);   /* Call from super-loop, self-throttles to 250ms */
