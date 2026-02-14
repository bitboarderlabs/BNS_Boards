/* Core/Src/mqtt_client.c — MQTT 3.1.1 client over W5500 socket 5
 *
 * Minimal MQTT 3.1.1 implementation for BNS boards.  Handles CONNECT,
 * SUBSCRIBE (QoS 1), PUBLISH (QoS 1, retain), PINGREQ/PINGRESP,
 * and DISCONNECT.  Publishes DIN/DOUT on change (50 ms poll),
 * AIN on configurable interval, AOUT on change.  Subscribes to
 * DOUT/AOUT control topics and AIN re-publish trigger topics.
 */
#include "mqtt_client.h"
#include "w5500.h"
#include "user_config.h"
#include "dio.h"
#include "aio.h"
#include "node_id.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ---------- Constants ---------- */
#define MQTT_SOCKET         W5500_MQTT_SOCKET
#define TX_BUF_SIZE         256
#define RX_BUF_SIZE         256
#define CONNECT_TIMEOUT_MS  5000
#define PUBACK_TIMEOUT_MS   5000
#define DIO_POLL_MS         50
#define TOPIC_BUF_SIZE      128

/* MQTT 3.1.1 packet type high nibble */
#define PKT_CONNECT    0x10
#define PKT_CONNACK    0x20
#define PKT_PUBLISH    0x30
#define PKT_PUBACK     0x40
#define PKT_SUBSCRIBE  0x80
#define PKT_SUBACK     0x90
#define PKT_PINGREQ    0xC0
#define PKT_PINGRESP   0xD0
#define PKT_DISCONNECT 0xE0

/* ---------- State Machine ---------- */
typedef enum {
    MQTT_IDLE,
    MQTT_CONNECTING,
    MQTT_WAIT_CONNACK,
    MQTT_SUBSCRIBING,
    MQTT_CONNECTED,
    MQTT_RECONNECT_WAIT
} mqtt_state_t;

/* ---------- Context ---------- */
typedef struct {
    mqtt_state_t state;
    uint8_t      rx_buf[RX_BUF_SIZE];
    uint16_t     rx_len;
    uint16_t     next_pkt_id;
    uint8_t      sub_index;
    uint8_t      sub_count;
    uint32_t     deadline;          /* timeout for current sub-state */
    uint32_t     last_ping_tx;
    uint32_t     last_pkt_rx;
    uint32_t     reconnect_at;
    uint32_t     last_dio_poll;
    uint32_t     last_ain_pub;
    uint16_t     last_din;
    uint16_t     last_dout;
    uint16_t     last_ain[AIO_INPUT_COUNT_MAX];
    uint16_t     last_aout[AIO_OUTPUT_COUNT_MAX];
    char         prefix[96];
    uint8_t      prefix_len;
    /* QoS 1 in-flight tracking */
    bool         pub_pending;
    uint16_t     pub_pkt_id;
    uint32_t     pub_deadline;
    /* Dynamic source port */
    uint16_t     src_port;
} mqtt_ctx_t;

static mqtt_ctx_t ctx;

/* ================================================================
 *  Helpers
 * ================================================================ */

static inline bool timed_out(uint32_t deadline)
{
    return (int32_t)(HAL_GetTick() - deadline) >= 0;
}

static uint16_t next_pkt_id(void)
{
    if (++ctx.next_pkt_id == 0) ctx.next_pkt_id = 1;
    return ctx.next_pkt_id;
}

static inline void put_u16(uint8_t *b, uint16_t v)
{
    b[0] = v >> 8;
    b[1] = v & 0xFF;
}

/* Write length-prefixed UTF-8 string, return bytes written */
static uint16_t put_str(uint8_t *b, const char *s, uint16_t len)
{
    put_u16(b, len);
    memcpy(b + 2, s, len);
    return 2 + len;
}

/* Encode MQTT remaining length (supports up to 16383) */
static uint8_t encode_remaining(uint8_t *b, uint16_t length)
{
    uint8_t n = 0;
    do {
        uint8_t byte = length & 0x7F;
        length >>= 7;
        if (length) byte |= 0x80;
        b[n++] = byte;
    } while (length);
    return n;
}

static bool parse_ip(const char *s, uint8_t *ip)
{
    unsigned a, b, c, d;
    if (sscanf(s, "%u.%u.%u.%u", &a, &b, &c, &d) != 4) return false;
    if (a > 255 || b > 255 || c > 255 || d > 255) return false;
    ip[0] = a; ip[1] = b; ip[2] = c; ip[3] = d;
    return true;
}

/* Map subscription index → io[] array index.
 * Order: [0..dout) = DOUT, [dout..dout+aout) = AOUT, [dout+aout..total) = AIN */
static uint8_t sub_to_io_idx(uint8_t si)
{
    uint8_t dout = dio_get_output_count();
    uint8_t aout = aio_get_output_count();
    if (si < dout)             return CONFIG_IO_IDX_DOUT + si;
    si -= dout;
    if (si < aout)             return CONFIG_IO_IDX_AOUT + si;
    si -= aout;
    return CONFIG_IO_IDX_AIN + si;
}

/* ================================================================
 *  W5500 Socket Management
 * ================================================================ */

static void sock_connect(const uint8_t *ip, uint16_t port)
{
    uint8_t s = MQTT_SOCKET;

    w5500_socket_write_byte(s, SN_CR, SN_CR_CLOSE);
    while (w5500_socket_read_byte(s, SN_CR));

    w5500_socket_write_byte(s, SN_MR, SN_MR_TCP);

    ctx.src_port++;
    if (ctx.src_port < 50000 || ctx.src_port > 59999) ctx.src_port = 50000;
    w5500_socket_write_byte(s, SN_PORT,     ctx.src_port >> 8);
    w5500_socket_write_byte(s, SN_PORT + 1, ctx.src_port & 0xFF);

    w5500_socket_write_byte(s, SN_CR, SN_CR_OPEN);
    while (w5500_socket_read_byte(s, SN_CR));

    w5500_socket_write(s, SN_DIPR, ip, 4);
    w5500_socket_write_byte(s, SN_DPORT,     port >> 8);
    w5500_socket_write_byte(s, SN_DPORT + 1, port & 0xFF);

    w5500_socket_write_byte(s, SN_CR, SN_CR_CONNECT);
    while (w5500_socket_read_byte(s, SN_CR));
}

static void sock_close(void)
{
    w5500_socket_write_byte(MQTT_SOCKET, SN_CR, SN_CR_CLOSE);
    while (w5500_socket_read_byte(MQTT_SOCKET, SN_CR));
    ctx.rx_len = 0;
    ctx.pub_pending = false;
}

/* Read any available data into rx_buf */
static void rx_pull(void)
{
    uint16_t avail = w5500_get_rx_size(MQTT_SOCKET);
    if (avail == 0) return;
    uint16_t space = RX_BUF_SIZE - ctx.rx_len;
    if (avail > space) avail = space;
    if (avail == 0) return;
    w5500_read_data(MQTT_SOCKET, ctx.rx_buf + ctx.rx_len, avail);
    ctx.rx_len += avail;
    ctx.last_pkt_rx = HAL_GetTick();
}

/* Try to parse one complete MQTT packet from rx_buf.
 * On success: *type = packet type nibble, *total = full packet length,
 *             *hdr = fixed-header length.  Returns true. */
static bool rx_parse(uint8_t *type, uint16_t *total, uint8_t *hdr)
{
    if (ctx.rx_len < 2) return false;
    *type = ctx.rx_buf[0] & 0xF0;
    uint32_t rem = 0, mul = 1;
    uint8_t i;
    for (i = 1; i < ctx.rx_len && i < 5; i++) {
        rem += (ctx.rx_buf[i] & 0x7F) * mul;
        mul *= 128;
        if (!(ctx.rx_buf[i] & 0x80)) break;
    }
    if (i >= ctx.rx_len) return false;
    if (ctx.rx_buf[i] & 0x80) return false;
    *hdr = i + 1;
    *total = *hdr + (uint16_t)rem;
    return ctx.rx_len >= *total;
}

static void rx_consume(uint16_t len)
{
    if (len >= ctx.rx_len) { ctx.rx_len = 0; return; }
    ctx.rx_len -= len;
    memmove(ctx.rx_buf, ctx.rx_buf + len, ctx.rx_len);
}

/* ================================================================
 *  MQTT Packet Builders
 * ================================================================ */

static void send_connect(void)
{
    user_config_data_t *cfg = config_get();
    uint8_t vp[200];
    uint16_t vl = 0;

    /* Protocol Name + Level */
    vp[vl++] = 0; vp[vl++] = 4;
    vp[vl++] = 'M'; vp[vl++] = 'Q'; vp[vl++] = 'T'; vp[vl++] = 'T';
    vp[vl++] = 4; /* protocol level 3.1.1 */

    /* Connect Flags */
    uint8_t flags = 0x02; /* Clean Session */
    bool has_user = cfg->mqtt_username[0] != '\0';
    bool has_pass = has_user && cfg->mqtt_password[0] != '\0';
    if (has_user) flags |= 0x80;
    if (has_pass) flags |= 0x40;
    vp[vl++] = flags;

    /* Keep Alive */
    uint16_t ka = cfg->mqtt_keepalive_sec ? cfg->mqtt_keepalive_sec : 60;
    put_u16(vp + vl, ka); vl += 2;

    /* Client ID */
    char cid[32];
    int cl = cfg->mqtt_client_id_prefix[0]
        ? snprintf(cid, sizeof(cid), "%s_%d", cfg->mqtt_client_id_prefix, node_id_get())
        : snprintf(cid, sizeof(cid), "bns_%d", node_id_get());
    vl += put_str(vp + vl, cid, cl);

    if (has_user) vl += put_str(vp + vl, cfg->mqtt_username, strlen(cfg->mqtt_username));
    if (has_pass) vl += put_str(vp + vl, cfg->mqtt_password, strlen(cfg->mqtt_password));

    /* Fixed header + send */
    uint8_t buf[TX_BUF_SIZE];
    uint16_t p = 0;
    buf[p++] = PKT_CONNECT;
    p += encode_remaining(buf + p, vl);
    memcpy(buf + p, vp, vl); p += vl;
    w5500_send(MQTT_SOCKET, buf, p);
}

static void send_subscribe(const char *topic, uint16_t tlen)
{
    uint8_t buf[TX_BUF_SIZE];
    uint16_t pid = next_pkt_id();
    uint16_t rem = 2 + 2 + tlen + 1;
    uint16_t p = 0;
    buf[p++] = PKT_SUBSCRIBE | 0x02; /* required reserved bits */
    p += encode_remaining(buf + p, rem);
    put_u16(buf + p, pid); p += 2;
    p += put_str(buf + p, topic, tlen);
    buf[p++] = 0x01; /* QoS 1 */
    w5500_send(MQTT_SOCKET, buf, p);
}

/* Publish with QoS 1 + retain.  Returns packet ID. */
static uint16_t send_publish(const char *name, uint16_t nlen,
                             const uint8_t *payload, uint16_t plen,
                             bool dup)
{
    uint8_t buf[TX_BUF_SIZE];
    uint16_t pid = next_pkt_id();
    uint16_t tlen = ctx.prefix_len + nlen;
    uint16_t rem = 2 + tlen + 2 + plen;

    uint16_t p = 0;
    buf[p++] = PKT_PUBLISH | 0x02 | 0x01 | (dup ? 0x08 : 0x00);
    p += encode_remaining(buf + p, rem);
    put_u16(buf + p, tlen); p += 2;
    memcpy(buf + p, ctx.prefix, ctx.prefix_len); p += ctx.prefix_len;
    memcpy(buf + p, name, nlen); p += nlen;
    put_u16(buf + p, pid); p += 2;
    memcpy(buf + p, payload, plen); p += plen;

    w5500_send(MQTT_SOCKET, buf, p);
    return pid;
}

static void send_puback(uint16_t pid)
{
    uint8_t buf[4] = { PKT_PUBACK, 0x02, pid >> 8, pid & 0xFF };
    w5500_send(MQTT_SOCKET, buf, 4);
}

static void send_pingreq(void)
{
    uint8_t buf[2] = { PKT_PINGREQ, 0x00 };
    w5500_send(MQTT_SOCKET, buf, 2);
    ctx.last_ping_tx = HAL_GetTick();
}

static void send_disconnect(void)
{
    uint8_t buf[2] = { PKT_DISCONNECT, 0x00 };
    w5500_send(MQTT_SOCKET, buf, 2);
}

/* ================================================================
 *  Topic Management
 * ================================================================ */

static void build_prefix(void)
{
    user_config_data_t *cfg = config_get();
    int len;
    if (cfg->mqtt_append_node_id)
        len = snprintf(ctx.prefix, sizeof(ctx.prefix), "%s_%d/",
                       cfg->mqtt_root_topic, node_id_get());
    else
        len = snprintf(ctx.prefix, sizeof(ctx.prefix), "%s/",
                       cfg->mqtt_root_topic);
    if (len < 0) len = 0;
    if (len >= (int)sizeof(ctx.prefix)) len = sizeof(ctx.prefix) - 1;
    ctx.prefix_len = (uint8_t)len;
}

/* Build full topic for subscription sub_index into buf.  Returns length. */
static uint16_t build_sub_topic(uint8_t si, char *buf, uint16_t bsz)
{
    user_config_data_t *cfg = config_get();
    uint8_t io = sub_to_io_idx(si);
    int len = snprintf(buf, bsz, "%s%s", ctx.prefix, cfg->io[io].name);
    if (len < 0) len = 0;
    if (len >= (int)bsz) len = bsz - 1;
    return (uint16_t)len;
}

/* Advance sub_index past I/O entries with empty names */
static void advance_sub_index(void)
{
    user_config_data_t *cfg = config_get();
    while (ctx.sub_index < ctx.sub_count) {
        uint8_t io = sub_to_io_idx(ctx.sub_index);
        if (cfg->io[io].name[0] != '\0') break;
        ctx.sub_index++;
    }
}

static void subscribe_next(void)
{
    advance_sub_index();
    if (ctx.sub_index >= ctx.sub_count) {
        /* All subscriptions done — enter CONNECTED */
        ctx.state = MQTT_CONNECTED;
        ctx.last_din  = 0xFFFF;
        ctx.last_dout = 0xFFFF;
        for (uint8_t i = 0; i < AIO_INPUT_COUNT_MAX;  i++) ctx.last_ain[i]  = 0xFFFF;
        for (uint8_t i = 0; i < AIO_OUTPUT_COUNT_MAX; i++) ctx.last_aout[i] = 0xFFFF;
        ctx.last_dio_poll = 0;
        ctx.last_ain_pub  = HAL_GetTick();
        return;
    }
    char topic[TOPIC_BUF_SIZE];
    uint16_t tlen = build_sub_topic(ctx.sub_index, topic, sizeof(topic));
    send_subscribe(topic, tlen);
    ctx.deadline = HAL_GetTick() + CONNECT_TIMEOUT_MS;
}

/* ================================================================
 *  Incoming PUBLISH Handler
 * ================================================================ */

static void handle_incoming_publish(const uint8_t *pkt, uint16_t pkt_len)
{
    user_config_data_t *cfg = config_get();
    uint8_t qos = (pkt[0] >> 1) & 0x03;

    /* Skip fixed header */
    uint16_t pos = 1;
    while (pos < pkt_len && (pkt[pos] & 0x80)) pos++;
    pos++;

    /* Topic */
    if (pos + 2 > pkt_len) return;
    uint16_t tlen = (pkt[pos] << 8) | pkt[pos + 1]; pos += 2;
    if (pos + tlen > pkt_len) return;
    const char *topic = (const char *)(pkt + pos); pos += tlen;

    /* Packet ID for QoS >= 1 */
    uint16_t pid = 0;
    if (qos >= 1) {
        if (pos + 2 > pkt_len) return;
        pid = (pkt[pos] << 8) | pkt[pos + 1]; pos += 2;
    }

    /* Payload */
    const char *payload = (const char *)(pkt + pos);
    uint16_t plen = pkt_len - pos;

    if (qos == 1) send_puback(pid);

    /* Must start with our prefix */
    if (tlen <= ctx.prefix_len) return;
    if (memcmp(topic, ctx.prefix, ctx.prefix_len) != 0) return;
    const char *name = topic + ctx.prefix_len;
    uint16_t nlen = tlen - ctx.prefix_len;

    /* Search DOUT */
    uint8_t dout_cnt = dio_get_output_count();
    for (uint8_t i = 0; i < dout_cnt; i++) {
        io_config_entry_t *io = &cfg->io[CONFIG_IO_IDX_DOUT + i];
        if (strlen(io->name) == nlen && memcmp(io->name, name, nlen) == 0) {
            uint16_t on_len  = strlen(io->mqtt_on_value);
            uint16_t off_len = strlen(io->mqtt_off_value);
            if (plen == on_len && memcmp(payload, io->mqtt_on_value, on_len) == 0)
                dio_output_set(i, true);
            else if (plen == off_len && memcmp(payload, io->mqtt_off_value, off_len) == 0)
                dio_output_set(i, false);
            return;
        }
    }

    /* Search AOUT */
    uint8_t aout_cnt = aio_get_output_count();
    for (uint8_t i = 0; i < aout_cnt; i++) {
        io_config_entry_t *io = &cfg->io[CONFIG_IO_IDX_AOUT + i];
        if (strlen(io->name) == nlen && memcmp(io->name, name, nlen) == 0) {
            char vb[8];
            uint16_t cl = plen < 7 ? plen : 7;
            memcpy(vb, payload, cl); vb[cl] = '\0';
            int v = atoi(vb);
            if (v < 0) v = 0;
            if (v > 4095) v = 4095;
            aio_output_set(i, (uint16_t)v);
            return;
        }
    }

    /* Search AIN — any message triggers re-publish */
    uint8_t ain_cnt = aio_get_input_count();
    for (uint8_t i = 0; i < ain_cnt; i++) {
        io_config_entry_t *io = &cfg->io[CONFIG_IO_IDX_AIN + i];
        if (strlen(io->name) == nlen && memcmp(io->name, name, nlen) == 0) {
            ctx.last_ain[i] = 0xFFFF; /* force re-publish */
            return;
        }
    }
}

/* ================================================================
 *  RX Processing — dispatches complete packets
 * ================================================================ */

static void process_rx(void)
{
    rx_pull();
    uint8_t type;
    uint16_t total;
    uint8_t hdr;

    while (rx_parse(&type, &total, &hdr)) {
        switch (type) {

        case PKT_CONNACK:
            if (ctx.state == MQTT_WAIT_CONNACK && total >= 4) {
                uint8_t rc = ctx.rx_buf[hdr + 1];
                if (rc == 0) {
                    ctx.sub_index = 0;
                    ctx.sub_count = dio_get_output_count()
                                  + aio_get_output_count()
                                  + aio_get_input_count();
                    ctx.last_ping_tx = HAL_GetTick();
                    ctx.last_pkt_rx  = HAL_GetTick();
                    ctx.state = MQTT_SUBSCRIBING;
                    subscribe_next();
                } else {
                    sock_close();
                    goto reconnect;
                }
            }
            break;

        case PKT_SUBACK:
            if (ctx.state == MQTT_SUBSCRIBING) {
                ctx.sub_index++;
                subscribe_next();
            }
            break;

        case PKT_PUBLISH:
            handle_incoming_publish(ctx.rx_buf, total);
            break;

        case PKT_PUBACK:
            if (ctx.pub_pending && total >= 4) {
                uint16_t ack = (ctx.rx_buf[hdr] << 8) | ctx.rx_buf[hdr + 1];
                if (ack == ctx.pub_pkt_id) ctx.pub_pending = false;
            }
            break;

        case PKT_PINGRESP:
            break; /* last_pkt_rx already updated by rx_pull */

        default:
            break;
        }
        rx_consume(total);
    }
    return;

reconnect:
    {
        user_config_data_t *cfg = config_get();
        uint32_t delay = cfg->mqtt_reconnect_ms ? cfg->mqtt_reconnect_ms : 5000;
        ctx.state = MQTT_RECONNECT_WAIT;
        ctx.reconnect_at = HAL_GetTick() + delay;
    }
}

/* ================================================================
 *  I/O Polling & Publishing
 * ================================================================ */

static uint16_t read_din_bank(void)
{
    uint16_t bank = 0;
    uint8_t n = dio_get_input_count();
    for (uint8_t i = 0; i < n; i++)
        if (dio_input_get(i)) bank |= (1u << i);
    return bank;
}

static uint16_t read_dout_bank(void)
{
    uint16_t bank = 0;
    uint8_t n = dio_get_output_count();
    for (uint8_t i = 0; i < n; i++)
        if (dio_output_get(i)) bank |= (1u << i);
    return bank;
}

/* Start a QoS 1 publish and set pending state.  Returns true. */
static bool start_publish(const char *name, uint16_t nlen,
                          const uint8_t *payload, uint16_t plen)
{
    ctx.pub_pkt_id  = send_publish(name, nlen, payload, plen, false);
    ctx.pub_pending  = true;
    ctx.pub_deadline = HAL_GetTick() + PUBACK_TIMEOUT_MS;
    return true;
}

/* Check for I/O changes and publish one message.  Returns true if sent. */
static bool check_publish(void)
{
    user_config_data_t *cfg = config_get();
    uint32_t now = HAL_GetTick();

    /* ---- Digital I/O (rate-limited to 50 ms) ---- */
    if (now - ctx.last_dio_poll >= DIO_POLL_MS) {
        ctx.last_dio_poll = now;

        /* DIN */
        uint16_t din = read_din_bank();
        uint16_t dchg = din ^ ctx.last_din;
        uint8_t din_cnt = dio_get_input_count();
        for (uint8_t i = 0; i < din_cnt; i++) {
            if (!(dchg & (1u << i))) continue;
            io_config_entry_t *io = &cfg->io[CONFIG_IO_IDX_DIN + i];
            if (io->name[0] == '\0') { ctx.last_din ^= (1u << i); continue; }
            bool on = (din >> i) & 1;
            const char *val = on ? io->mqtt_on_value : io->mqtt_off_value;
            ctx.last_din ^= (1u << i);
            return start_publish(io->name, strlen(io->name),
                                 (const uint8_t *)val, strlen(val));
        }

        /* DOUT */
        uint16_t dout = read_dout_bank();
        uint16_t ochg = dout ^ ctx.last_dout;
        uint8_t dout_cnt = dio_get_output_count();
        for (uint8_t i = 0; i < dout_cnt; i++) {
            if (!(ochg & (1u << i))) continue;
            io_config_entry_t *io = &cfg->io[CONFIG_IO_IDX_DOUT + i];
            if (io->name[0] == '\0') { ctx.last_dout ^= (1u << i); continue; }
            bool on = (dout >> i) & 1;
            const char *val = on ? io->mqtt_on_value : io->mqtt_off_value;
            ctx.last_dout ^= (1u << i);
            return start_publish(io->name, strlen(io->name),
                                 (const uint8_t *)val, strlen(val));
        }
    }

    /* ---- AOUT change detection ---- */
    uint8_t aout_cnt = aio_get_output_count();
    for (uint8_t i = 0; i < aout_cnt; i++) {
        uint16_t cur = aio_output_get(i);
        if (cur == ctx.last_aout[i]) continue;
        ctx.last_aout[i] = cur;
        io_config_entry_t *io = &cfg->io[CONFIG_IO_IDX_AOUT + i];
        if (io->name[0] == '\0') continue;
        char vs[8];
        int vl = snprintf(vs, sizeof(vs), "%u", cur);
        return start_publish(io->name, strlen(io->name),
                             (const uint8_t *)vs, vl);
    }

    /* ---- AIN periodic publish ---- */
    uint8_t ain_cnt = aio_get_input_count();
    uint16_t interval = cfg->mqtt_analog_interval_s ? cfg->mqtt_analog_interval_s : 30;
    if (ain_cnt > 0 && (now - ctx.last_ain_pub >= (uint32_t)interval * 1000)) {
        for (uint8_t i = 0; i < ain_cnt; i++) ctx.last_ain[i] = 0xFFFF;
        ctx.last_ain_pub = now;
    }

    /* AIN change / forced re-publish */
    for (uint8_t i = 0; i < ain_cnt; i++) {
        uint16_t cur = aio_input_get(i);
        if (cur == ctx.last_ain[i]) continue;
        ctx.last_ain[i] = cur;
        io_config_entry_t *io = &cfg->io[CONFIG_IO_IDX_AIN + i];
        if (io->name[0] == '\0') continue;
        char vs[8];
        int vl = snprintf(vs, sizeof(vs), "%u", cur);
        return start_publish(io->name, strlen(io->name),
                             (const uint8_t *)vs, vl);
    }

    return false;
}

/* ================================================================
 *  Reconnect helper
 * ================================================================ */

static void enter_reconnect(void)
{
    user_config_data_t *cfg = config_get();
    uint32_t delay = cfg->mqtt_reconnect_ms ? cfg->mqtt_reconnect_ms : 5000;
    ctx.state = MQTT_RECONNECT_WAIT;
    ctx.reconnect_at = HAL_GetTick() + delay;
}

/* ================================================================
 *  State Machine
 * ================================================================ */

static void state_idle(void)
{
    user_config_data_t *cfg = config_get();
    if (!cfg->mqtt_enabled) return;
    if (cfg->mqtt_broker_host[0] == '\0') return;

    uint8_t ip[4];
    if (!parse_ip(cfg->mqtt_broker_host, ip)) return;

    uint16_t port = cfg->mqtt_broker_port ? cfg->mqtt_broker_port : 1883;

    build_prefix();
    sock_connect(ip, port);
    ctx.state    = MQTT_CONNECTING;
    ctx.deadline = HAL_GetTick() + CONNECT_TIMEOUT_MS;
}

static void state_connecting(void)
{
    uint8_t sr = w5500_socket_read_byte(MQTT_SOCKET, SN_SR);

    if (sr == SOCK_ESTABLISHED) {
        ctx.rx_len = 0;
        send_connect();
        ctx.state    = MQTT_WAIT_CONNACK;
        ctx.deadline = HAL_GetTick() + CONNECT_TIMEOUT_MS;
        return;
    }
    if (sr == SOCK_CLOSED) { enter_reconnect(); return; }
    if (timed_out(ctx.deadline)) { sock_close(); enter_reconnect(); }
}

static void state_wait_connack(void)
{
    uint8_t sr = w5500_socket_read_byte(MQTT_SOCKET, SN_SR);
    if (sr != SOCK_ESTABLISHED) { sock_close(); enter_reconnect(); return; }

    process_rx();
    if (ctx.state != MQTT_WAIT_CONNACK) return; /* state changed in process_rx */
    if (timed_out(ctx.deadline)) { sock_close(); enter_reconnect(); }
}

static void state_subscribing(void)
{
    uint8_t sr = w5500_socket_read_byte(MQTT_SOCKET, SN_SR);
    if (sr != SOCK_ESTABLISHED) { sock_close(); enter_reconnect(); return; }

    process_rx();
    if (ctx.state != MQTT_SUBSCRIBING) return;
    if (timed_out(ctx.deadline)) { sock_close(); enter_reconnect(); }
}

static void state_connected(void)
{
    user_config_data_t *cfg = config_get();

    /* Socket health */
    uint8_t sr = w5500_socket_read_byte(MQTT_SOCKET, SN_SR);
    if (sr != SOCK_ESTABLISHED) { sock_close(); enter_reconnect(); return; }

    /* User disabled MQTT */
    if (!cfg->mqtt_enabled) {
        send_disconnect();
        sock_close();
        ctx.state = MQTT_IDLE;
        return;
    }

    /* Process incoming packets */
    process_rx();
    if (ctx.state != MQTT_CONNECTED) return; /* disconnected in process_rx */

    /* PUBACK timeout handling */
    if (ctx.pub_pending && timed_out(ctx.pub_deadline))
        ctx.pub_pending = false;

    /* Publish one I/O change if nothing in flight */
    if (!ctx.pub_pending)
        check_publish();

    /* Keepalive — ping at half the keepalive interval */
    uint32_t now = HAL_GetTick();
    uint16_t ka = cfg->mqtt_keepalive_sec ? cfg->mqtt_keepalive_sec : 60;
    uint32_t ka_ms = (uint32_t)ka * 1000;
    if (now - ctx.last_ping_tx >= ka_ms / 2)
        send_pingreq();

    /* Broker timeout — no packets received for 1.5x keepalive */
    if (now - ctx.last_pkt_rx >= ka_ms + ka_ms / 2) {
        sock_close();
        enter_reconnect();
    }
}

static void state_reconnect_wait(void)
{
    user_config_data_t *cfg = config_get();
    if (!cfg->mqtt_enabled) { ctx.state = MQTT_IDLE; return; }
    if (timed_out(ctx.reconnect_at)) ctx.state = MQTT_IDLE;
}

/* ================================================================
 *  Public API
 * ================================================================ */

void mqtt_client_init(void)
{
    memset(&ctx, 0, sizeof(ctx));
    ctx.next_pkt_id = 1;
    ctx.src_port    = 49999; /* will increment to 50000 on first connect */
    ctx.state       = MQTT_IDLE;
}

void mqtt_client_task(void)
{
    switch (ctx.state) {
    case MQTT_IDLE:            state_idle();            break;
    case MQTT_CONNECTING:      state_connecting();      break;
    case MQTT_WAIT_CONNACK:    state_wait_connack();    break;
    case MQTT_SUBSCRIBING:     state_subscribing();     break;
    case MQTT_CONNECTED:       state_connected();       break;
    case MQTT_RECONNECT_WAIT:  state_reconnect_wait();  break;
    }
}

bool mqtt_client_is_connected(void)
{
    return ctx.state == MQTT_CONNECTED;
}
