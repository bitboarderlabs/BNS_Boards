/* Core/Inc/aio.h */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "board_id.h"

#define AIO_INPUT_COUNT_BABY  0
#define AIO_INPUT_COUNT_MAMA 12
#define AIO_INPUT_COUNT_PAPA 32
#define AIO_INPUT_COUNT_MAX	 32

#define AIO_OUTPUT_COUNT_BABY  0
#define AIO_OUTPUT_COUNT_MAMA  8
#define AIO_OUTPUT_COUNT_PAPA 16
#define AIO_OUTPUT_COUNT_MAX  16

void aio_init(board_type_t type);

/* Analog outputs – 0-based index */
void  aio_output_set(uint8_t idx, uint16_t val);
void  aio_all_outputs_off(void);
uint16_t  aio_output_get(uint8_t idx);


/* Analog inputs – 0-based index */
uint16_t  aio_input_get(uint8_t idx);

/* New: Get counts based on board type */
uint8_t aio_get_input_count(void);
uint8_t aio_get_output_count(void);
