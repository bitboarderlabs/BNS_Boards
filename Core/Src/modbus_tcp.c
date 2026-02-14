/* Core/Src/modbus_tcp.c — Modbus TCP server on W5500 socket 4, port 502
 *
 * Ported from FacCon HVAC_RemCon_MB modbus_tcp.c (lwIP callbacks → W5500 polling).
 * Supports FC 0x01, 0x02, 0x04, 0x05, 0x06, 0x0F, 0x10.
 */
#include "modbus_tcp.h"
#include "w5500.h"
#include "dio.h"
#include "aio.h"
#include "node_id.h"
#include <string.h>

/* ---------- Constants ---------- */
#define MODBUS_PORT         502
#define MODBUS_RX_BUF_SIZE  260   /* MBAP header (7) + max PDU (253) */
#define MODBUS_TIMEOUT_MS   30000 /* 30 s inactivity timeout */
#define MODBUS_MBAP_HDR     7     /* Transaction(2) + Protocol(2) + Length(2) + Unit(1) */
#define MODBUS_MIN_FRAME    8     /* MBAP header + at least 1 byte FC */

/* Modbus exception codes */
#define EX_ILLEGAL_FUNCTION    0x01
#define EX_ILLEGAL_DATA_ADDR   0x02
#define EX_ILLEGAL_DATA_VALUE  0x03

/* ---------- Types ---------- */
typedef enum {
    MODBUS_IDLE,
    MODBUS_CONNECTED,
    MODBUS_CLOSE
} modbus_state_t;

typedef struct {
    modbus_state_t state;
    uint8_t        rx_buf[MODBUS_RX_BUF_SIZE];
    uint16_t       rx_len;
    uint32_t       timeout;
} modbus_conn_t;

/* ---------- State ---------- */
static modbus_conn_t conn;

/* ---------- Forward declarations ---------- */
static void modbus_socket_open(void);
static void modbus_process_frame(void);
static void modbus_send_exception(uint8_t fc, uint8_t ex_code);

/* ---------- Init ---------- */
void modbus_tcp_init(void)
{
    conn.state  = MODBUS_IDLE;
    conn.rx_len = 0;
    modbus_socket_open();
}

static void modbus_socket_open(void)
{
    uint8_t s = W5500_MODBUS_SOCKET;

    w5500_socket_write_byte(s, SN_CR, SN_CR_CLOSE);
    while (w5500_socket_read_byte(s, SN_CR));

    w5500_socket_write_byte(s, SN_MR, SN_MR_TCP);
    w5500_socket_write_byte(s, SN_PORT,     (MODBUS_PORT >> 8) & 0xFF);
    w5500_socket_write_byte(s, SN_PORT + 1,  MODBUS_PORT       & 0xFF);

    w5500_socket_write_byte(s, SN_CR, SN_CR_OPEN);
    while (w5500_socket_read_byte(s, SN_CR));
    w5500_socket_write_byte(s, SN_CR, SN_CR_LISTEN);
    while (w5500_socket_read_byte(s, SN_CR));

    conn.state  = MODBUS_IDLE;
    conn.rx_len = 0;
}

/* ---------- Main task ---------- */
void modbus_tcp_task(void)
{
    uint8_t s = W5500_MODBUS_SOCKET;
    uint8_t status = w5500_socket_read_byte(s, SN_SR);

    switch (conn.state) {

    case MODBUS_IDLE:
        if (status == SOCK_ESTABLISHED) {
            conn.state   = MODBUS_CONNECTED;
            conn.rx_len  = 0;
            conn.timeout = HAL_GetTick() + MODBUS_TIMEOUT_MS;
        } else if (status == SOCK_CLOSED) {
            modbus_socket_open();
        }
        break;

    case MODBUS_CONNECTED:
        if (HAL_GetTick() > conn.timeout) {
            conn.state = MODBUS_CLOSE;
            break;
        }
        if (status == SOCK_CLOSE_WAIT || status == SOCK_CLOSED) {
            conn.state = MODBUS_CLOSE;
            break;
        }
        {
            uint16_t avail = w5500_get_rx_size(s);
            if (avail > 0) {
                uint16_t space = MODBUS_RX_BUF_SIZE - conn.rx_len;
                uint16_t to_read = (avail < space) ? avail : space;
                if (to_read > 0) {
                    w5500_read_data(s, &conn.rx_buf[conn.rx_len], to_read);
                    conn.rx_len += to_read;
                }
                conn.timeout = HAL_GetTick() + MODBUS_TIMEOUT_MS;

                /* Check if we have a complete MBAP frame */
                while (conn.rx_len >= MODBUS_MBAP_HDR) {
                    uint16_t pdu_len = (conn.rx_buf[4] << 8) | conn.rx_buf[5];
                    uint16_t frame_len = 6 + pdu_len; /* MBAP header (6 bytes) + PDU */

                    if (frame_len > MODBUS_RX_BUF_SIZE || frame_len < MODBUS_MIN_FRAME) {
                        /* Invalid frame length — discard buffer */
                        conn.rx_len = 0;
                        break;
                    }

                    if (conn.rx_len < frame_len) {
                        break; /* Need more data */
                    }

                    /* Complete frame — process it */
                    modbus_process_frame();

                    /* Shift remaining data forward */
                    if (conn.rx_len > frame_len) {
                        memmove(conn.rx_buf, conn.rx_buf + frame_len, conn.rx_len - frame_len);
                        conn.rx_len -= frame_len;
                    } else {
                        conn.rx_len = 0;
                    }
                }
            }
        }
        break;

    case MODBUS_CLOSE:
        w5500_socket_write_byte(s, SN_CR, SN_CR_DISCON);
        while (w5500_socket_read_byte(s, SN_CR));
        modbus_socket_open();
        break;
    }
}

/* ---------- Frame processing ---------- */
static void modbus_process_frame(void)
{
    uint8_t *data = conn.rx_buf;

    /* Validate unit ID: must match node_id or broadcast (0xFF) */
    uint8_t uid = node_id_get();
    if (data[6] != uid && data[6] != 0xFF) {
        return;
    }

    uint8_t fc = data[7];
    uint8_t resp[260];
    uint16_t resp_len = 0;

    /* Copy MBAP header (transaction ID + protocol ID) */
    memcpy(resp, data, 6);
    resp[6] = uid;

    uint16_t addr, qty;

    switch (fc) {

    case 0x01:  /* Read Coils (digital outputs) */
    case 0x02:  /* Read Discrete Inputs (digital inputs) */
    {
        addr = (data[8] << 8) | data[9];
        qty  = (data[10] << 8) | data[11];

        bool is_coil = (fc == 0x01);
        uint16_t max_count = is_coil ? dio_get_output_count() : dio_get_input_count();

        if (addr >= max_count || qty == 0) {
            modbus_send_exception(fc, EX_ILLEGAL_DATA_ADDR);
            return;
        }

        /* Clamp quantity to available points */
        if (addr + qty > max_count) {
            qty = max_count - addr;
        }

        uint8_t byte_count = (qty + 7) / 8;
        resp[7] = fc;
        resp[8] = byte_count;
        resp_len = 9 + byte_count;

        memset(&resp[9], 0, byte_count);
        for (uint16_t i = 0; i < qty; i++) {
            bool val = is_coil ? dio_output_get(addr + i) : dio_input_get(addr + i);
            if (val) {
                resp[9 + i / 8] |= (1 << (i % 8));
            }
        }
        break;
    }

    case 0x04:  /* Read Input Registers (analog inputs, raw 12-bit ADC) */
    {
        addr = (data[8] << 8) | data[9];
        qty  = (data[10] << 8) | data[11];

        uint16_t max_count = aio_get_input_count();

        if (addr >= max_count || qty == 0) {
            modbus_send_exception(fc, EX_ILLEGAL_DATA_ADDR);
            return;
        }

        if (addr + qty > max_count) {
            qty = max_count - addr;
        }

        uint8_t byte_count = qty * 2;
        resp[7] = fc;
        resp[8] = byte_count;
        resp_len = 9 + byte_count;

        for (uint16_t i = 0; i < qty; i++) {
            uint16_t val = aio_input_get(addr + i);
            resp[9 + i * 2]     = val >> 8;
            resp[9 + i * 2 + 1] = val & 0xFF;
        }
        break;
    }

    case 0x05:  /* Write Single Coil */
    {
        addr = (data[8] << 8) | data[9];
        uint16_t value = (data[10] << 8) | data[11];

        if (addr >= dio_get_output_count()) {
            modbus_send_exception(fc, EX_ILLEGAL_DATA_ADDR);
            return;
        }

        dio_output_set(addr, (value == 0xFF00));

        /* Echo request as response */
        memcpy(&resp[7], &data[7], 5);
        resp_len = 12;
        break;
    }

    case 0x06:  /* Write Single Register (analog output, 0-4095) */
    {
        addr = (data[8] << 8) | data[9];
        uint16_t value = (data[10] << 8) | data[11];

        if (addr >= aio_get_output_count()) {
            modbus_send_exception(fc, EX_ILLEGAL_DATA_ADDR);
            return;
        }

        aio_output_set(addr, value);

        /* Echo request as response */
        memcpy(&resp[7], &data[7], 5);
        resp_len = 12;
        break;
    }

    case 0x0F:  /* Write Multiple Coils */
    {
        addr = (data[8] << 8) | data[9];
        qty  = (data[10] << 8) | data[11];
        uint8_t byte_count = data[12];

        if (addr >= dio_get_output_count() || qty == 0 || byte_count != (qty + 7) / 8) {
            modbus_send_exception(fc, EX_ILLEGAL_DATA_ADDR);
            return;
        }

        if (addr + qty > dio_get_output_count()) {
            qty = dio_get_output_count() - addr;
        }

        for (uint16_t i = 0; i < qty; i++) {
            bool val = (data[13 + i / 8] & (1 << (i % 8))) != 0;
            dio_output_set(addr + i, val);
        }

        resp[7]  = fc;
        resp[8]  = addr >> 8;
        resp[9]  = addr & 0xFF;
        resp[10] = qty >> 8;
        resp[11] = qty & 0xFF;
        resp_len = 12;
        break;
    }

    case 0x10:  /* Write Multiple Registers (analog outputs) */
    {
        addr = (data[8] << 8) | data[9];
        qty  = (data[10] << 8) | data[11];
        uint8_t byte_count = data[12];

        if (addr >= aio_get_output_count() || qty == 0 || byte_count != qty * 2) {
            modbus_send_exception(fc, EX_ILLEGAL_DATA_ADDR);
            return;
        }

        if (addr + qty > aio_get_output_count()) {
            qty = aio_get_output_count() - addr;
        }

        for (uint16_t i = 0; i < qty; i++) {
            uint16_t val = (data[13 + i * 2] << 8) | data[13 + i * 2 + 1];
            aio_output_set(addr + i, val);
        }

        resp[7]  = fc;
        resp[8]  = addr >> 8;
        resp[9]  = addr & 0xFF;
        resp[10] = qty >> 8;
        resp[11] = qty & 0xFF;
        resp_len = 12;
        break;
    }

    default:
        modbus_send_exception(fc, EX_ILLEGAL_FUNCTION);
        return;
    }

    /* Fill MBAP length field (bytes after the length field: unit_id + PDU) */
    uint16_t pdu_len = resp_len - 6;
    resp[4] = pdu_len >> 8;
    resp[5] = pdu_len & 0xFF;

    w5500_send(W5500_MODBUS_SOCKET, resp, resp_len);
}

static void modbus_send_exception(uint8_t fc, uint8_t ex_code)
{
    uint8_t resp[9];

    /* Copy transaction ID + protocol ID from request */
    memcpy(resp, conn.rx_buf, 4);
    /* Length: 3 bytes (unit_id + error_fc + exception_code) */
    resp[4] = 0x00;
    resp[5] = 0x03;
    resp[6] = node_id_get();
    resp[7] = fc | 0x80;
    resp[8] = ex_code;

    w5500_send(W5500_MODBUS_SOCKET, resp, 9);
}
