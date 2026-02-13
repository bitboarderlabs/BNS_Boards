/* Core/Inc/bridge.h */
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    BRIDGE_OFF = 0,
    BRIDGE_RS422_TO_ETH = 1,
    BRIDGE_RS422_TO_USB = 2,
    BRIDGE_ETH_TO_RS422 = 4,
    BRIDGE_ETH_TO_USB   = 8,
    BRIDGE_USB_TO_RS422 = 16,
    BRIDGE_USB_TO_ETH   = 32,
    BRIDGE_ALL = 0xFF
} bridge_mode_t;

void bridge_init(void);
void bridge_set_rs422(bridge_mode_t mode);
void bridge_set_eth(bridge_mode_t mode);
void bridge_set_usb(bridge_mode_t mode);
void bridge_forward(const uint8_t* packet, uint16_t len);
