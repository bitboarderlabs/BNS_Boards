/* Core/Inc/board_id.h */
#pragma once
#include "stm32f4xx_hal.h"

typedef enum {
    BOARD_TYPE_BABY       = 0x00,
    BOARD_TYPE_RESERVED_01,
    BOARD_TYPE_MAMA       = 0x02,
    BOARD_TYPE_PAPA       = 0x03
} board_type_t;

/* Pin definitions – change ONLY these when a new PCB revision appears */
#if defined(BOARD_BABY)
	#define HSE_FREQ_MHZ  8
    #define BOARDID0_PORT GPIOC
    #define BOARDID0_PIN  GPIO_PIN_0
    #define BOARDID1_PORT GPIOC
    #define BOARDID1_PIN  GPIO_PIN_1
    #define BOARDID2_PORT GPIOC
    #define BOARDID2_PIN  GPIO_PIN_2
    #define BOARDID3_PORT GPIOC
    #define BOARDID3_PIN  GPIO_PIN_3
#elif defined(BOARD_MAMA)
	#define HSE_FREQ_MHZ  16
	#define BOARDID0_PORT GPIOA
    #define BOARDID0_PIN  GPIO_PIN_8
    #define BOARDID1_PORT GPIOC
    #define BOARDID1_PIN  GPIO_PIN_12
    #define BOARDID2_PORT GPIOB
    #define BOARDID2_PIN  GPIO_PIN_8
    #define BOARDID3_PORT GPIOB
    #define BOARDID3_PIN  GPIO_PIN_9
#elif defined(BOARD_PAPA)
	#define HSE_FREQ_MHZ  16
	#define BOARDID0_PORT GPIOA
    #define BOARDID0_PIN  GPIO_PIN_8
    #define BOARDID1_PORT GPIOC
    #define BOARDID1_PIN  GPIO_PIN_12
    #define BOARDID2_PORT GPIOB
    #define BOARDID2_PIN  GPIO_PIN_8
    #define BOARDID3_PORT GPIOB
    #define BOARDID3_PIN  GPIO_PIN_9
#else
    #error "Define BOARD_BABY, BOARD_MAMA or BOARD_PAPA in the project preprocessor"
#endif

board_type_t board_get_type(void);
