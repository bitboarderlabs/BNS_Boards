///* Core/Inc/status_led.h */
//#pragma once
//#include "board_id.h"
//
//typedef enum {
//    STATUS_OFF = 0,
//    STATUS_BOOT,
//    STATUS_APP_LOADED,
//    STATUS_APP_RUNNING,
//    STATUS_ERROR
//} status_t;
//
//void status_led_init(board_type_t boardType);
//void status_led_set(status_t s);
//void status_led_blink(status_t s, uint16_t ms);
//void status_led_data_activity(void);


#ifndef STATUS_LED_H
#define STATUS_LED_H

#include "stm32f4xx_hal.h"
#include "board_id.h"                  // Your external board_type_t definition

typedef enum {
    //LED_MODE_NOT_READY = 0,   // Solid off
    LED_MODE_BOOTING = 0,
	LED_MODE_NOAPP,				// 250ms ON, 1750ms OFF  No app loaded.
	LED_MODE_STOPPED,         	// 250ms ON, 750ms OFF
    LED_MODE_RUNNING          // 500ms cycle: 250ms ON, 250ms OFF (2 Hz, 50% duty)
} status_led_mode_t;

/* Public API */
void     status_led_init(board_type_t boardType);
void     status_led_set_mode(status_led_mode_t mode);
status_led_mode_t status_led_get_mode(void);
void     status_led_data_activity(void);

/* Must be called regularly (e.g. every 1–10 ms) from main loop */
void     status_led_task(void);

#endif /* STATUS_LED_H */
