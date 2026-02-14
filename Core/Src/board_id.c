/* Core/Src/board_id.c */
#include "board_id.h"

static uint8_t read_board_id(void)
{
    uint8_t id = 0;
    if (HAL_GPIO_ReadPin(BOARDID0_PORT, BOARDID0_PIN)) id |= 0x01;
    if (HAL_GPIO_ReadPin(BOARDID1_PORT, BOARDID1_PIN)) id |= 0x02;
    if (HAL_GPIO_ReadPin(BOARDID2_PORT, BOARDID2_PIN)) id |= 0x04;
    if (HAL_GPIO_ReadPin(BOARDID3_PORT, BOARDID3_PIN)) id |= 0x08;
    return id;
}

board_type_t board_get_type(void)
{
    uint8_t raw = read_board_id();
    switch (raw) {
        case 0x00: return BOARD_TYPE_BABY;
        case 0x02: return BOARD_TYPE_MAMA;
        case 0x03: return BOARD_TYPE_RESERVED_01; /* Papa cancelled */
        default:   return BOARD_TYPE_RESERVED_01;
    }
}
