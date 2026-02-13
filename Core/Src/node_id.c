/* Core/Src/node_id.c */
#include "node_id.h"

uint8_t node_id_get(void)
{
    uint8_t id = 0;

    if (HAL_GPIO_ReadPin(NODEID1_MOD_PORT, NODEID1_MOD_PIN)) id |= 0x01;
    if (HAL_GPIO_ReadPin(NODEID2_MOD_PORT, NODEID2_MOD_PIN)) id |= 0x02;
    if (HAL_GPIO_ReadPin(NODEID4_MOD_PORT, NODEID4_MOD_PIN)) id |= 0x04;
    if (HAL_GPIO_ReadPin(NODEID8_MOD_PORT, NODEID8_MOD_PIN)) id |= 0x08;

    return id;  // Returns 0 to 15
}
