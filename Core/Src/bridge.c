/* Core/Src/bridge.c */
#include "bridge.h"
#include "comm.h"
#include "packet.h"
#include "w5500.h"

static bridge_mode_t rs422_mode = BRIDGE_OFF;
static bridge_mode_t eth_mode   = BRIDGE_OFF;
static bridge_mode_t usb_mode   = BRIDGE_OFF;

void bridge_init(void)
{
    rs422_mode = BRIDGE_OFF;
    eth_mode   = BRIDGE_OFF;
    usb_mode   = BRIDGE_OFF;
}

void bridge_set_rs422(bridge_mode_t mode) { rs422_mode = mode; }
void bridge_set_eth(bridge_mode_t mode)   { eth_mode   = mode; }
void bridge_set_usb(bridge_mode_t mode)   { usb_mode   = mode; }

void bridge_forward(const uint8_t* packet, uint16_t len)
{
    if (len < 2) return;
    uint8_t src = packet[0];  // STX or interface hint

    if (src == PACKET_STX) {
        if (rs422_mode & BRIDGE_RS422_TO_ETH) {
        	//send to all connected eth sockets:
        	for(uint8_t s=W5500_APP_SOCKET_START; s<W5500_APP_SOCKET_START + W5500_APP_SOCKET_COUNT; s++){
        		comm_eth_send(s, packet, len);
        	}

        }
        if (rs422_mode & BRIDGE_RS422_TO_USB) comm_usb_send(packet, len);
    }
    // Add other directions as needed
}
