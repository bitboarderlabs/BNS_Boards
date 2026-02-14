/* Core/Src/w5500.c */
#include "w5500.h"
#include "status_led.h"
#include <string.h>

#define BUFFER_SIZE_KB 2
#define BUFFER_MASK (BUFFER_SIZE_KB * 1024 - 1)

#define DHCP_MSG_LEN  576
#define DHCP_UDP_HDR  8     /* W5500 prepends [srcIP(4)][srcPort(2)][dataLen(2)] on UDP RX */
static uint8_t dhcp_buffer[DHCP_UDP_HDR + DHCP_MSG_LEN];
static uint8_t dhcp_xid[4] = {0x12, 0x34, 0x56, 0x78};
static uint8_t dhcp_state = 0;       /* 0=idle, 1=DISCOVER sent, 2=REQUEST sent, 3=done */
static uint32_t dhcp_tick = 0;
static uint32_t dhcp_retry_tick = 0; /* timeout for retry */
#define DHCP_RETRY_MS 4000

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

    /* Wait for SEND_OK (Sn_IR bit 4 = 0x10) or TIMEOUT (bit 3 = 0x08) */
    uint32_t timeout = HAL_GetTick() + 5000;
    while (HAL_GetTick() < timeout) {
        uint8_t ir = w5500_socket_read_byte(socket, SN_IR);
        if (ir & 0x08) {  /* TIMEOUT */
            w5500_socket_write_byte(socket, SN_IR, 0x08);
            break;
        }
        if (ir & 0x10) {  /* SEND_OK */
            w5500_socket_write_byte(socket, SN_IR, 0x10);
            return;
        }
    }
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
    dhcp_retry_tick = dhcp_tick;
}

/* Build and send a DHCP DISCOVER or REQUEST packet.
 * TX uses dhcp_buffer[0..] directly (no UDP header on send). */
static void dhcp_send_discover(void) {
    memset(dhcp_buffer, 0, DHCP_MSG_LEN);
    dhcp_buffer[0]  = 0x01;        /* op:    BOOTREQUEST */
    dhcp_buffer[1]  = 0x01;        /* htype: Ethernet   */
    dhcp_buffer[2]  = 0x06;        /* hlen:  6          */
    memcpy(&dhcp_buffer[4], dhcp_xid, 4);   /* xid   (offset 4)  */
    dhcp_buffer[10] = 0x80;        /* flags: broadcast  */
    w5500_get_mac(&dhcp_buffer[28]);        /* chaddr (offset 28) */

    /* Magic cookie at offset 236, options at 240 */
    dhcp_buffer[236] = 0x63; dhcp_buffer[237] = 0x82;
    dhcp_buffer[238] = 0x53; dhcp_buffer[239] = 0x63;
    uint8_t *opt = &dhcp_buffer[240];
    opt[0] = 53; opt[1] = 1; opt[2] = 1;   /* DHCP Message Type = DISCOVER */
    opt[3] = 0xFF;                          /* End */

    uint8_t bcast[4] = {255, 255, 255, 255};
    w5500_udp_send(W5500_DHCP_SOCKET, dhcp_buffer, 244, bcast, 67);
}

static void dhcp_send_request(const uint8_t *offered_ip, const uint8_t *server_ip) {
    memset(dhcp_buffer, 0, DHCP_MSG_LEN);
    dhcp_buffer[0]  = 0x01;
    dhcp_buffer[1]  = 0x01;
    dhcp_buffer[2]  = 0x06;
    memcpy(&dhcp_buffer[4], dhcp_xid, 4);
    dhcp_buffer[10] = 0x80;        /* flags: broadcast */
    w5500_get_mac(&dhcp_buffer[28]);

    dhcp_buffer[236] = 0x63; dhcp_buffer[237] = 0x82;
    dhcp_buffer[238] = 0x53; dhcp_buffer[239] = 0x63;
    uint8_t *opt = &dhcp_buffer[240];
    opt[0] = 53; opt[1] = 1; opt[2] = 3;               /* DHCP Message Type = REQUEST */
    opt[3] = 50; opt[4] = 4; memcpy(&opt[5], offered_ip, 4);  /* Requested IP */
    opt[9] = 54; opt[10] = 4; memcpy(&opt[11], server_ip, 4); /* Server Identifier */
    opt[15] = 0xFF;                                     /* End */

    uint8_t bcast[4] = {255, 255, 255, 255};
    w5500_udp_send(W5500_DHCP_SOCKET, dhcp_buffer, 256, bcast, 67);
}

bool w5500_dhcp_run(void) {
    if (ip_mode != IP_MODE_DHCP || dhcp_state == 3) return dhcp_state == 3;

    uint32_t now = HAL_GetTick();
    if (now - dhcp_tick < 100) return false;  /* poll every 100 ms */
    dhcp_tick = now;

    /* Retry timeout — reopen socket and go back to DISCOVER */
    if (dhcp_state != 0 && (now - dhcp_retry_tick) > DHCP_RETRY_MS) {
        w5500_socket_write_byte(W5500_DHCP_SOCKET, SN_CR, SN_CR_CLOSE);
        while (w5500_socket_read_byte(W5500_DHCP_SOCKET, SN_CR));
        w5500_socket_write_byte(W5500_DHCP_SOCKET, SN_MR, SN_MR_UDP);
        w5500_socket_write_byte(W5500_DHCP_SOCKET, SN_PORT, 0);
        w5500_socket_write_byte(W5500_DHCP_SOCKET, SN_PORT + 1, 68);
        w5500_socket_write_byte(W5500_DHCP_SOCKET, SN_CR, SN_CR_OPEN);
        while (w5500_socket_read_byte(W5500_DHCP_SOCKET, SN_CR));
        dhcp_state = 0;
    }

    /* Check for incoming data */
    uint16_t raw_len = w5500_get_rx_size(W5500_DHCP_SOCKET);

    if (raw_len == 0) {
        if (dhcp_state == 0) {
            dhcp_send_discover();
            dhcp_state = 1;
            dhcp_retry_tick = now;
        }
        return false;
    }

    /* Read raw UDP data (includes 8-byte W5500 packet-info header) */
    uint16_t buf_max = sizeof(dhcp_buffer);
    if (raw_len > buf_max) raw_len = buf_max;
    w5500_read_data(W5500_DHCP_SOCKET, dhcp_buffer, raw_len);

    /* Skip W5500 UDP header — actual DHCP message starts at offset 8 */
    if (raw_len <= DHCP_UDP_HDR + 240) return false;
    uint8_t *msg = &dhcp_buffer[DHCP_UDP_HDR];
    uint16_t msg_len = raw_len - DHCP_UDP_HDR;

    /* Validate: must be BOOTREPLY with matching XID */
    if (msg[0] != 0x02) return false;
    if (memcmp(&msg[4], dhcp_xid, 4) != 0) return false;

    /* Parse DHCP options (magic cookie at msg[236], options at msg[240]) */
    if (msg_len < 244) return false;
    if (msg[236] != 0x63 || msg[237] != 0x82 ||
        msg[238] != 0x53 || msg[239] != 0x63) return false;

    uint8_t *options = &msg[240];
    uint16_t opt_len = msg_len - 240;
    uint8_t msg_type = 0;
    uint8_t server_id[4] = {0};
    uint8_t subnet[4] = {255, 255, 255, 0};
    uint8_t gateway[4] = {0};

    for (uint16_t i = 0; i < opt_len; ) {
        uint8_t code = options[i];
        if (code == 0xFF) break;          /* End */
        if (code == 0x00) { i++; continue; }  /* Pad */
        if (i + 1 >= opt_len) break;
        uint8_t olen = options[i + 1];
        if (i + 2 + olen > opt_len) break;
        if (code == 53 && olen == 1) msg_type = options[i + 2];
        if (code == 54 && olen == 4) memcpy(server_id, &options[i + 2], 4);
        if (code == 1  && olen == 4) memcpy(subnet,    &options[i + 2], 4);
        if (code == 3  && olen == 4) memcpy(gateway,   &options[i + 2], 4);
        i += 2 + olen;
    }

    if (msg_type == 2 && dhcp_state == 1) {
        /* OFFER — save offered IP and send REQUEST */
        uint8_t offered_ip[4];
        memcpy(offered_ip, &msg[16], 4);   /* yiaddr */
        dhcp_send_request(offered_ip, server_id);
        dhcp_state = 2;
        dhcp_retry_tick = now;
    } else if (msg_type == 5 && dhcp_state == 2) {
        /* ACK — apply network config */
        w5500_common_write(SIPR, &msg[16], 4);
        w5500_common_write(SUBR, subnet, 4);
        w5500_common_write(GAR, gateway, 4);
        dhcp_state = 3;
        return true;
    }
    return false;
}

ip_mode_t w5500_get_ip_mode(void) { return ip_mode; }
