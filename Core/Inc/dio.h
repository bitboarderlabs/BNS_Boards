/* Core/Inc/dio.h */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "board_id.h"

#define DIO_INPUT_COUNT_BABY  4
#define DIO_INPUT_COUNT_MAMA 12
#define DIO_INPUT_COUNT_PAPA 32
#define DIO_INPUT_COUNT_MAX	 32

#define DIO_OUTPUT_COUNT_BABY  2
#define DIO_OUTPUT_COUNT_MAMA  8
#define DIO_OUTPUT_COUNT_PAPA 16
#define DIO_OUTPUT_COUNT_MAX  16

void dio_init(board_type_t type);

/* Digital outputs – 0-based index */
void  dio_output_set(uint8_t idx, bool on);
void  dio_all_outputs_off(void);
bool  dio_output_get(uint8_t idx);

/* Global enable for Mama/Papa (active-low) – does nothing on Baby */
void  dio_output_enable(bool enable);

/* Digital inputs – 0-based index */
bool  dio_input_get(uint8_t idx);

/* New: Get counts based on board type */
uint8_t dio_get_input_count(void);
uint8_t dio_get_output_count(void);
