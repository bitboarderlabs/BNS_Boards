/* Core/Inc/w5500.h */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

extern SPI_HandleTypeDef hspi1;

/* ---- pinout ---- */
#define CS_GPIO_PORT   GPIOB
#define CS_PIN         GPIO_PIN_7
#define RESET_PORT     GPIOE
#define RESET_PIN      GPIO_PIN_3

/* ---- sockets ---- */
#define W5500_APP_PORT          4242
#define W5500_DHCP_SOCKET       4
#define W5500_APP_SOCKET_START  0   // Sockets 0-3 for 4 app TCP
#define W5500_APP_SOCKET_COUNT  4

/* ---- IP mode ---- */
typedef enum { IP_MODE_STATIC = 0, IP_MODE_DHCP } ip_mode_t;

/* ---- register offsets ---- */
#define MR           0x0000
#define GAR          0x0001
#define SUBR         0x0005
#define SHAR         0x0009
#define SIPR         0x000F

#define SN_MR        0x0000
#define SN_CR        0x0001
#define SN_IR        0x0002
#define SN_SR        0x0003
#define SN_PORT      0x0004
#define SN_DIPR      0x000C
#define SN_DPORT     0x0010
#define SN_TX_FSR    0x0020
#define SN_TX_WR     0x0024
#define SN_RX_RSR    0x0026
#define SN_RX_RD     0x0028
#define SN_TX_SIZE   0x001E
#define SN_RX_SIZE   0x001F

/* ---- commands ---- */
#define MR_RST       0x80
#define SN_MR_TCP    0x01
#define SN_MR_UDP    0x02
#define SN_CR_OPEN   0x01
#define SN_CR_LISTEN 0x02
#define SN_CR_CONNECT 0x04
#define SN_CR_DISCON 0x08
#define SN_CR_CLOSE  0x10
#define SN_CR_SEND   0x20
#define SN_CR_RECV   0x40

/* ---- status ---- */
#define SOCK_CLOSED      0x00
#define SOCK_INIT        0x13
#define SOCK_LISTEN      0x14
#define SOCK_ESTABLISHED 0x17
#define SOCK_CLOSE_WAIT  0x1C
#define SOCK_UDP         0x22

/* ---- low-level SPI ---- */
static inline void _w5500_select(void)   { HAL_GPIO_WritePin(CS_GPIO_PORT, CS_PIN, GPIO_PIN_RESET); }
static inline void _w5500_deselect(void) { HAL_GPIO_WritePin(CS_GPIO_PORT, CS_PIN, GPIO_PIN_SET); }

static inline void w5500_write(uint16_t offset, uint8_t cb, const uint8_t *data, uint16_t len) {
    uint8_t hdr[3] = {offset >> 8, offset & 0xFF, cb};
    _w5500_select();
    HAL_SPI_Transmit(&hspi1, hdr, 3, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)data, len, HAL_MAX_DELAY);
    _w5500_deselect();
}

static inline void w5500_read_raw(uint16_t offset, uint8_t cb, uint8_t *data, uint16_t len) {
    uint8_t hdr[3] = {offset >> 8, offset & 0xFF, cb};
    _w5500_select();
    HAL_SPI_Transmit(&hspi1, hdr, 3, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, data, len, HAL_MAX_DELAY);
    _w5500_deselect();
}

static inline void w5500_read_burst(uint16_t offset, uint8_t cb, uint8_t *data, uint16_t len) {
    // Implement burst if needed; for now use regular read
    w5500_read_raw(offset, cb, data, len);
}

static inline void w5500_write_burst(uint16_t offset, uint8_t cb, const uint8_t *data, uint16_t len) {
    // Implement burst if needed; for now use regular write
    w5500_write(offset, cb, data, len);
}

static inline uint8_t w5500_common_read_byte(uint16_t offset) {
    uint8_t v;
    w5500_read_raw(offset, 0x00, &v, 1);
    return v;
}

static inline void w5500_common_write_byte(uint16_t offset, uint8_t val) {
    w5500_write(offset, 0x04, &val, 1);
}

static inline void w5500_common_write(uint16_t offset, const uint8_t *data, uint16_t len) {
    w5500_write(offset, 0x04, data, len);
}

static inline void w5500_common_read(uint16_t offset, uint8_t *data, uint16_t len) {
    w5500_read_raw(offset, 0x00, data, len);
}

static inline uint8_t w5500_socket_read_byte(uint8_t socket, uint16_t offset) {
    uint8_t cb = (socket << 5) | 0x08;  // Block 1 (socket regs), read
    uint8_t v;
    w5500_read_raw(offset, cb, &v, 1);
    return v;
}

static inline void w5500_socket_write_byte(uint8_t socket, uint16_t offset, uint8_t val) {
    uint8_t cb = (socket << 5) | 0x0C;  // Block 1, write
    w5500_write(offset, cb, &val, 1);
}

static inline void w5500_socket_write(uint8_t socket, uint16_t offset, const uint8_t *data, uint16_t len) {
    uint8_t cb = (socket << 5) | 0x0C;
    w5500_write(offset, cb, data, len);
}

static inline void w5500_socket_read(uint8_t socket, uint16_t offset, uint8_t *data, uint16_t len) {
    uint8_t cb = (socket << 5) | 0x08;
    w5500_read_raw(offset, cb, data, len);
}

/* ---- public API ---- */
void w5500_set_ip_mode(ip_mode_t mode);
void w5500_init(void);
bool w5500_is_connected(uint8_t socket);
uint16_t w5500_get_rx_size(uint8_t socket);
void w5500_read_data(uint8_t socket, uint8_t *buf, uint16_t len);
void w5500_send(uint8_t socket, const uint8_t *data, uint16_t len);
void w5500_udp_send(uint8_t socket, const uint8_t *data, uint16_t len,
                    const uint8_t *dest_ip, uint16_t dest_port);
void w5500_get_ip(uint8_t *ip);
void w5500_get_mac(uint8_t *mac);
void w5500_dhcp_init(void);
bool w5500_dhcp_run(void);
ip_mode_t w5500_get_ip_mode(void);
