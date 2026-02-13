/* Core/Inc/comm.h */
#pragma once
#include <stdarg.h>  // For va_list in debug_log prototype
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "w5500.h"


/* --------------------------------------------------------------------- */
/* CHANNEL IDs (0-based)                                                 */
#define CH_RS422      			0
#define CH_USB        			1
#define CH_ETH_BASE   			2                							/* sockets 0-3 to CH_ETH_BASE … CH_ETH_BASE+3 */
#define CH_COUNT      			(CH_ETH_BASE + W5500_APP_SOCKET_COUNT)		/* 2 + 4 = 6 total channels */
#define CH_ALL		  			255											// send to all channels

/* --------------------------------------------------------------------- */
extern UART_HandleTypeDef huart3;  // RS422
extern SPI_HandleTypeDef  hspi1;   // W5500

/* USB TX busy flag */
extern volatile bool usb_tx_busy;


/* RS422 Output Queue */
#define RS422_TX_QUEUE_SIZE 2048
void rs422_queue(const uint8_t* data, uint16_t len);



void comm_feed_byte(uint8_t channel, uint8_t byte);

void comm_init(uint8_t node_id);
void rs422_set_crossover(bool enable);

void comm_task(void);

/* Send on a specific channel (255 = all) */
void comm_send_channel(const uint8_t *data, uint16_t len, uint8_t channel);  // channel: 0=RS422, 1=USB, 2-5=eth, 255=all

/* Low-level send helpers (used internally) */
void comm_rs422_send(const uint8_t* data, uint16_t len);
void comm_eth_send(uint8_t socket, const uint8_t *data, uint16_t len);
void comm_usb_send(const uint8_t* data, uint16_t len);


/* Debug logging (USB only) */
void debug_print(const char* msg);        // Raw output - no newline
void debug_println(const char* msg);      // Auto-adds \r\n - one-shot logging
