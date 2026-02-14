/* Core/Src/w5500.c */
#include "w5500.h"
#include "status_led.h"
#include <string.h>

#define BUFFER_SIZE_KB 2
#define BUFFER_MASK (BUFFER_SIZE_KB * 1024 - 1)

#define DHCP_MSG_LEN 576
static uint8_t dhcp_buffer[DHCP_MSG_LEN];
static uint8_t dhcp_xid[4] = {0x12, 0x34, 0x56, 0x78};
static uint8_t dhcp_state = 0;
static uint32_t dhcp_tick = 0;

static ip_mode_t ip_mode = IP_MODE_STATIC;
static uint8_t static_ip[4]      = {192, 168, 1, 100};
static uint8_t static_gateway[4] = {192, 168, 1,  1};
static uint8_t static_subnet[4]  = {255, 255, 255,  0};

void w5500_set_ip_mode(ip_mode_t mode) { ip_mode = mode; }

void w5500_set_network_config(const uint8_t *ip, const uint8_t *gateway,
                              const uint8_t *subnet)
{
    memcpy(static_ip, ip, 4);
    memcpy(static_gateway, gateway, 4);
    memcpy(static_subnet, subnet, 4);
}

void w5500_init(void) {
    HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(2);

    // Set buffer sizes for all sockets
    for (uint8_t s = 0; s < 8; ++s) {
        w5500_socket_write_byte(s, SN_TX_SIZE, BUFFER_SIZE_KB);
        w5500_socket_write_byte(s, SN_RX_SIZE, BUFFER_SIZE_KB);
    }

    // Unique MAC based on UID
    uint32_t uid0 = HAL_GetUIDw0();
    uint8_t mac[6] = {0x02, 0x00,
                      (uid0 >> 24) & 0xFF, (uid0 >> 16) & 0xFF,
                      (uid0 >> 8)  & 0xFF, uid0 & 0xFF};
    w5500_common_write(SHAR, mac, 6);

    if (ip_mode == IP_MODE_STATIC) {
        w5500_common_write(SIPR, static_ip, 4);
        w5500_common_write(GAR,  static_gateway, 4);
        w5500_common_write(SUBR, static_subnet, 4);
    } else {
        uint8_t zero[4] = {0};
        w5500_common_write(SIPR, zero, 4);
        w5500_common_write(GAR,  zero, 4);
        w5500_common_write(SUBR, zero, 4);
        w5500_dhcp_init();
    }

    // Open BNS protocol sockets 0-1 as TCP listeners
    for (uint8_t s = W5500_BNS_SOCKET_START; s < W5500_BNS_SOCKET_START + W5500_BNS_SOCKET_COUNT; s++) {
        w5500_socket_write_byte(s, SN_CR, SN_CR_CLOSE);
        while (w5500_socket_read_byte(s, SN_CR));
        w5500_socket_write_byte(s, SN_MR, SN_MR_TCP);
        w5500_socket_write_byte(s, SN_PORT, W5500_BNS_PORT >> 8);
        w5500_socket_write_byte(s, SN_PORT + 1, W5500_BNS_PORT & 0xFF);
        w5500_socket_write_byte(s, SN_CR, SN_CR_OPEN);
        while (w5500_socket_read_byte(s, SN_CR));
        w5500_socket_write_byte(s, SN_CR, SN_CR_LISTEN);
        while (w5500_socket_read_byte(s, SN_CR));
    }
    // HTTP sockets 2-3 are opened by webserver_init()
    // Modbus (4), MQTT (5) opened by their respective modules

}

bool w5500_is_connected(uint8_t socket) {
    return w5500_socket_read_byte(socket, SN_SR) == SOCK_ESTABLISHED;
}

uint16_t w5500_get_rx_size(uint8_t socket) {
    uint16_t size;
    w5500_socket_read(socket, SN_RX_RSR, (uint8_t*)&size, 2);
    size = (size >> 8) | ((size & 0xFF) << 8);  // Big-endian
    return size;
}

void w5500_read_data(uint8_t socket, uint8_t *buf, uint16_t len) {
    uint16_t ptr;
    w5500_socket_read(socket, SN_RX_RD, (uint8_t*)&ptr, 2);
    ptr = (ptr >> 8) | ((ptr & 0xFF) << 8);

    uint8_t cb = (socket << 5) | 0x18;  // Block 3 (RX buf), read
    w5500_read_burst(ptr, cb, buf, len);

    ptr += len;
    uint8_t ptr_be[2] = {ptr >> 8, ptr & 0xFF};
    w5500_socket_write(socket, SN_RX_RD, ptr_be, 2);
    w5500_socket_write_byte(socket, SN_CR, SN_CR_RECV);
    while (w5500_socket_read_byte(socket, SN_CR));
}

void w5500_send(uint8_t socket, const uint8_t *data, uint16_t len) {
    if (len == 0) return;

    uint16_t ptr;
    w5500_socket_read(socket, SN_TX_WR, (uint8_t*)&ptr, 2);
    ptr = (ptr >> 8) | ((ptr & 0xFF) << 8);

    uint8_t cb = (socket << 5) | 0x14;  // Block 2 (TX buf), write
    w5500_write_burst(ptr, cb, data, len);

    ptr += len;
    uint8_t ptr_be[2] = {ptr >> 8, ptr & 0xFF};
    w5500_socket_write(socket, SN_TX_WR, ptr_be, 2);
    w5500_socket_write_byte(socket, SN_CR, SN_CR_SEND);
    while (w5500_socket_read_byte(socket, SN_CR));

    // Wait for SEND_OK or timeout
    uint32_t timeout = HAL_GetTick() + 5000;
    while (HAL_GetTick() < timeout) {
        uint8_t ir = w5500_socket_read_byte(socket, SN_IR);
        if (ir & 0x08) {  // TIMEOUT
            w5500_socket_write_byte(socket, SN_IR, 0x08);  // Clear
            break;
        }
        if (ir & 0x04) {  // SEND_OK
            w5500_socket_write_byte(socket, SN_IR, 0x04);
            return;
        }
    }
    // Timeout: Close socket
    w5500_socket_write_byte(socket, SN_CR, SN_CR_CLOSE);
}

void w5500_udp_send(uint8_t socket, const uint8_t *data, uint16_t len,
                    const uint8_t *dest_ip, uint16_t dest_port) {
    w5500_socket_write(socket, SN_DIPR, dest_ip, 4);
    uint8_t port_be[2] = {dest_port >> 8, dest_port & 0xFF};
    w5500_socket_write(socket, SN_DPORT, port_be, 2);
    w5500_send(socket, data, len);
}

void w5500_get_ip(uint8_t *ip) {
    w5500_common_read(SIPR, ip, 4);
}

void w5500_get_mac(uint8_t *mac) {
    w5500_common_read(SHAR, mac, 6);
}

void w5500_dhcp_init(void) {
    if (ip_mode != IP_MODE_DHCP) return;

    // Open DHCP socket (UDP)
    w5500_socket_write_byte(W5500_DHCP_SOCKET, SN_CR, SN_CR_CLOSE);
    while (w5500_socket_read_byte(W5500_DHCP_SOCKET, SN_CR));
    w5500_socket_write_byte(W5500_DHCP_SOCKET, SN_MR, SN_MR_UDP);
    w5500_socket_write_byte(W5500_DHCP_SOCKET, SN_PORT, 68 >> 8);
    w5500_socket_write_byte(W5500_DHCP_SOCKET, SN_PORT + 1, 68 & 0xFF);
    w5500_socket_write_byte(W5500_DHCP_SOCKET, SN_CR, SN_CR_OPEN);
    while (w5500_socket_read_byte(W5500_DHCP_SOCKET, SN_CR));

    dhcp_state = 0;
    dhcp_tick = HAL_GetTick();
}

bool w5500_dhcp_run(void) {
    if (ip_mode != IP_MODE_DHCP || dhcp_state == 2) return dhcp_state == 2;

    uint32_t now = HAL_GetTick();
    if (now - dhcp_tick < 100) return false;  // Poll every 100ms
    dhcp_tick = now;

    uint16_t len = w5500_get_rx_size(W5500_DHCP_SOCKET);
    if (len > 0) {
        w5500_read_data(W5500_DHCP_SOCKET, dhcp_buffer, len > DHCP_MSG_LEN ? DHCP_MSG_LEN : len);
    } else if (dhcp_state == 0) {
        // Send DISCOVER
        memset(dhcp_buffer, 0, DHCP_MSG_LEN);
        dhcp_buffer[0] = 0x01; dhcp_buffer[1] = 0x01; dhcp_buffer[2] = 0x06; dhcp_buffer[4] = 0x01;
        memcpy(&dhcp_buffer[8], dhcp_xid, 4);
        w5500_get_mac(&dhcp_buffer[28]);

        uint8_t *opt = &dhcp_buffer[240];
        opt[0] = 0x63; opt[1] = 0x82; opt[2] = 0x53; opt[3] = 0x63;
        opt[4] = 0x35; opt[5] = 0x01; opt[6] = 0x01;
        opt[7] = 0xFF;

        uint8_t broadcast_ip[4] = {255, 255, 255, 255};
        w5500_udp_send(W5500_DHCP_SOCKET, dhcp_buffer, 240 + 8, broadcast_ip, 67);
        dhcp_state = 1;
        return false;
    } else {
        return false;  // Retry logic if needed
    }

    // Parse OFFER/ACK
    if (len < 240 || dhcp_buffer[0] != 0x02) return false;

    uint8_t *options = &dhcp_buffer[240];
    uint8_t msg_type = 0;
    for (int i = 0; i < len - 240; ) {
        uint8_t opt = options[i];
        if (opt == 0xFF) break;
        uint8_t optlen = options[i + 1];
        if (opt == 0x35 && optlen == 1) msg_type = options[i + 2];
        i += 2 + optlen;
    }

    if (msg_type == 2) {  // OFFER
        // Send REQUEST
        dhcp_state = 1;
        memset(dhcp_buffer, 0, DHCP_MSG_LEN);
        dhcp_buffer[0] = 0x01; dhcp_buffer[1] = 0x01; dhcp_buffer[2] = 0x06; dhcp_buffer[4] = 0x01;
        memcpy(&dhcp_buffer[8], dhcp_xid, 4);
        w5500_get_mac(&dhcp_buffer[28]);

        uint8_t *opt = &dhcp_buffer[240];
        opt[0] = 0x63; opt[1] = 0x82; opt[2] = 0x53; opt[3] = 0x63;
        opt[4] = 0x35; opt[5] = 0x01; opt[6] = 0x03;
        opt[7] = 0x32; opt[8] = 0x04; memcpy(&opt[9], &dhcp_buffer[16], 4);  // Requested IP from OFFER
        opt[13] = 0xFF;

        uint8_t broadcast_ip[4] = {255, 255, 255, 255};
        w5500_udp_send(W5500_DHCP_SOCKET, dhcp_buffer, 240 + 14, broadcast_ip, 67);
    } else if (msg_type == 5) {  // ACK
        uint8_t *yiaddr = &dhcp_buffer[16];
        w5500_common_write(SIPR, yiaddr, 4);

        uint8_t subnet[4] = {255, 255, 255, 0};
        uint8_t gateway[4] = {0};
        uint8_t *options = &dhcp_buffer[240];
        for (int i = 0; i < len - 240; ) {
            uint8_t opt = options[i];
            if (opt == 0xFF) break;
            uint8_t optlen = options[i + 1];
            if (opt == 1 && optlen == 4) memcpy(subnet, &options[i + 2], 4);
            if (opt == 3 && optlen == 4) memcpy(gateway, &options[i + 2], 4);
            i += 2 + optlen;
        }
        w5500_common_write(SUBR, subnet, 4);
        w5500_common_write(GAR, gateway, 4);

        dhcp_state = 2;
        return true;
    }
    return false;
}

ip_mode_t w5500_get_ip_mode(void) { return ip_mode; }
