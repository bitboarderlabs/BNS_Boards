/* Core/Inc/node_id.h */
#pragma once
#include "stm32f4xx_hal.h"

#if defined(BOARD_BABY)
    #define NODEID1_MOD_PORT  GPIOE
    #define NODEID1_MOD_PIN   GPIO_PIN_0
    #define NODEID2_MOD_PORT  GPIOB
    #define NODEID2_MOD_PIN   GPIO_PIN_8
    #define NODEID4_MOD_PORT  GPIOE
    #define NODEID4_MOD_PIN   GPIO_PIN_1
    #define NODEID8_MOD_PORT  GPIOB
    #define NODEID8_MOD_PIN   GPIO_PIN_9

#elif defined(BOARD_MAMA) || defined(BOARD_PAPA)
    #define NODEID1_MOD_PORT  GPIOE
    #define NODEID1_MOD_PIN   GPIO_PIN_6
    #define NODEID2_MOD_PORT  GPIOE
    #define NODEID2_MOD_PIN   GPIO_PIN_4
    #define NODEID4_MOD_PORT  GPIOE
    #define NODEID4_MOD_PIN   GPIO_PIN_5
    #define NODEID8_MOD_PORT  GPIOC
    #define NODEID8_MOD_PIN   GPIO_PIN_13

#else
    #error "Board type not defined. Use BOARD_BABY, BOARD_MAMA, or BOARD_PAPA"
#endif

/**
 * @brief Get the 4-bit Node ID from the rotary switch (0–15)
 * @return uint8_t Node ID
 */
uint8_t node_id_get(void);
