/* Core/Src/webserver.c — Raw HTTP server over W5500 TCP sockets 2-3 */
#include "webserver.h"
#include "w5500.h"
#include "user_config.h"
#include "web_content.h"
#include "dio.h"
#include "aio.h"
#include "app.h"
#include "board_id.h"
#include "node_id.h"
#include "mqtt_client.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ---------- Constants ---------- */
#define HTTP_RX_BUF_SIZE   1024
#define HTTP_URI_SIZE       64
#define HTTP_QUERY_SIZE     64
#define HTTP_HEADER_SIZE   256
#define JSON_BUF_SIZE     2048
#define HTTP_TIMEOUT_MS   5000
#define HTTP_SEND_CHUNK   1400  /* stay under MTU */

#define FW_VERSION "1.0.0"

/* ---------- Types ---------- */
typedef enum {
    HTTP_SOCK_IDLE,
    HTTP_SOCK_CONNECTED,
    HTTP_SOCK_RECV,
    HTTP_SOCK_PARSE,
    HTTP_SOCK_SEND_HEADER,
    HTTP_SOCK_SEND_BODY,
    HTTP_SOCK_CLOSE
} http_state_t;

typedef enum {
    HTTP_GET,
    HTTP_POST,
    HTTP_UNKNOWN
} http_method_t;

typedef struct {
    uint8_t       socket_num;
    http_state_t  state;

    /* RX */
    uint8_t       rx_buf[HTTP_RX_BUF_SIZE];
    uint16_t      rx_len;

    /* Parsed request */
    http_method_t method;
    char          uri[HTTP_URI_SIZE];
    char          query[HTTP_QUERY_SIZE];
    uint16_t      content_length;
    uint16_t      body_offset;     /* offset of POST body within rx_buf */

    /* Response */
    char          resp_header[HTTP_HEADER_SIZE];
    uint16_t      resp_header_len;
    uint16_t      resp_header_sent;
    const uint8_t *resp_body;      /* points to flash (static) or json_buf (dynamic) */
    uint32_t      resp_body_len;
    uint32_t      resp_body_sent;

    uint32_t      timeout;
} http_conn_t;

/* ---------- State ---------- */
static http_conn_t conns[W5500_HTTP_SOCKET_COUNT];
static char json_buf[JSON_BUF_SIZE];

/* ---------- Forward declarations ---------- */
static void http_socket_open(http_conn_t *c);
static void http_conn_process(http_conn_t *c);
static bool http_find_header_end(http_conn_t *c);
static void http_parse_request(http_conn_t *c);
static void http_route_request(http_conn_t *c);
static void http_send_chunk(http_conn_t *c);
static void http_serve_static(http_conn_t *c, const uint8_t *data, uint32_t len, const char *content_type);
static void http_serve_json(http_conn_t *c, const char *json, uint16_t len);
static void http_serve_json_ok(http_conn_t *c, bool ok);
static void http_serve_redirect(http_conn_t *c, const char *location);
static void http_serve_404(http_conn_t *c);

static void http_handle_status_json(http_conn_t *c);
static void http_handle_config_get(http_conn_t *c);
static void http_handle_config_post(http_conn_t *c);
static void http_handle_config_save(http_conn_t *c);
static void http_handle_config_revert(http_conn_t *c);
static void http_handle_toggle(http_conn_t *c);
static void http_handle_info_json(http_conn_t *c);
static void http_handle_app_control(http_conn_t *c);

/* ---------- Init ---------- */
void webserver_init(void)
{
    for (int i = 0; i < W5500_HTTP_SOCKET_COUNT; i++) {
        conns[i].socket_num = W5500_HTTP_SOCKET_START + i;
        conns[i].state = HTTP_SOCK_IDLE;
        http_socket_open(&conns[i]);
    }
}

static void http_socket_open(http_conn_t *c)
{
    uint8_t s = c->socket_num;
    w5500_socket_write_byte(s, SN_CR, SN_CR_CLOSE);
    while (w5500_socket_read_byte(s, SN_CR));

    w5500_socket_write_byte(s, SN_MR, SN_MR_TCP);
    uint8_t port_hi = (W5500_HTTP_PORT >> 8) & 0xFF;
    uint8_t port_lo = W5500_HTTP_PORT & 0xFF;
    w5500_socket_write_byte(s, SN_PORT, port_hi);
    w5500_socket_write_byte(s, SN_PORT + 1, port_lo);

    w5500_socket_write_byte(s, SN_CR, SN_CR_OPEN);
    while (w5500_socket_read_byte(s, SN_CR));
    w5500_socket_write_byte(s, SN_CR, SN_CR_LISTEN);
    while (w5500_socket_read_byte(s, SN_CR));

    c->state = HTTP_SOCK_IDLE;
    c->rx_len = 0;
}

/* ---------- Main task ---------- */
void webserver_task(void)
{
    for (int i = 0; i < W5500_HTTP_SOCKET_COUNT; i++) {
        http_conn_process(&conns[i]);
    }
}

static void http_conn_process(http_conn_t *c)
{
    uint8_t s = c->socket_num;
    uint8_t status = w5500_socket_read_byte(s, SN_SR);

    switch (c->state) {

    case HTTP_SOCK_IDLE:
        if (status == SOCK_ESTABLISHED) {
            c->state = HTTP_SOCK_CONNECTED;
            c->rx_len = 0;
            c->timeout = HAL_GetTick() + HTTP_TIMEOUT_MS;
        } else if (status == SOCK_CLOSED) {
            http_socket_open(c);
        }
        break;

    case HTTP_SOCK_CONNECTED:
    case HTTP_SOCK_RECV:
        if (HAL_GetTick() > c->timeout) {
            c->state = HTTP_SOCK_CLOSE;
            break;
        }
        if (status == SOCK_CLOSE_WAIT || status == SOCK_CLOSED) {
            c->state = HTTP_SOCK_CLOSE;
            break;
        }
        {
            uint16_t avail = w5500_get_rx_size(s);
            if (avail > 0) {
                uint16_t space = HTTP_RX_BUF_SIZE - c->rx_len;
                uint16_t to_read = (avail < space) ? avail : space;
                if (to_read > 0) {
                    w5500_read_data(s, &c->rx_buf[c->rx_len], to_read);
                    c->rx_len += to_read;
                }
                if (http_find_header_end(c)) {
                    c->state = HTTP_SOCK_PARSE;
                } else {
                    c->state = HTTP_SOCK_RECV;
                }
            }
        }
        break;

    case HTTP_SOCK_PARSE:
        http_parse_request(c);
        http_route_request(c);
        c->state = HTTP_SOCK_SEND_HEADER;
        c->resp_header_sent = 0;
        c->resp_body_sent = 0;
        break;

    case HTTP_SOCK_SEND_HEADER:
        if (status == SOCK_CLOSE_WAIT || status == SOCK_CLOSED) {
            c->state = HTTP_SOCK_CLOSE;
            break;
        }
        http_send_chunk(c);
        break;

    case HTTP_SOCK_SEND_BODY:
        if (status == SOCK_CLOSE_WAIT || status == SOCK_CLOSED) {
            c->state = HTTP_SOCK_CLOSE;
            break;
        }
        http_send_chunk(c);
        break;

    case HTTP_SOCK_CLOSE:
        w5500_socket_write_byte(s, SN_CR, SN_CR_DISCON);
        while (w5500_socket_read_byte(s, SN_CR));
        http_socket_open(c);
        break;
    }
}

/* ---------- HTTP parsing ---------- */
static bool http_find_header_end(http_conn_t *c)
{
    for (uint16_t i = 0; i + 3 < c->rx_len; i++) {
        if (c->rx_buf[i] == '\r' && c->rx_buf[i+1] == '\n' &&
            c->rx_buf[i+2] == '\r' && c->rx_buf[i+3] == '\n') {
            c->body_offset = i + 4;
            return true;
        }
    }
    return false;
}

static void http_parse_request(http_conn_t *c)
{
    c->method = HTTP_UNKNOWN;
    c->uri[0] = '\0';
    c->query[0] = '\0';
    c->content_length = 0;

    char *buf = (char *)c->rx_buf;

    if (memcmp(buf, "GET ", 4) == 0) {
        c->method = HTTP_GET;
        buf += 4;
    } else if (memcmp(buf, "POST ", 5) == 0) {
        c->method = HTTP_POST;
        buf += 5;
    } else {
        return;
    }

    /* Extract URI and query string */
    char *end = strstr(buf, " HTTP");
    if (!end) end = buf + strlen(buf);

    char *q = memchr(buf, '?', end - buf);
    if (q) {
        uint16_t uri_len = q - buf;
        if (uri_len >= HTTP_URI_SIZE) uri_len = HTTP_URI_SIZE - 1;
        memcpy(c->uri, buf, uri_len);
        c->uri[uri_len] = '\0';

        uint16_t q_len = end - q - 1;
        if (q_len >= HTTP_QUERY_SIZE) q_len = HTTP_QUERY_SIZE - 1;
        memcpy(c->query, q + 1, q_len);
        c->query[q_len] = '\0';
    } else {
        uint16_t uri_len = end - buf;
        if (uri_len >= HTTP_URI_SIZE) uri_len = HTTP_URI_SIZE - 1;
        memcpy(c->uri, buf, uri_len);
        c->uri[uri_len] = '\0';
    }

    /* Extract Content-Length for POST */
    if (c->method == HTTP_POST) {
        char *cl = strstr((char *)c->rx_buf, "Content-Length:");
        if (!cl) cl = strstr((char *)c->rx_buf, "content-length:");
        if (cl) c->content_length = atoi(cl + 15);
    }
}

/* ---------- Routing ---------- */
static void http_route_request(http_conn_t *c)
{
    const char *uri = c->uri;

    if (strcmp(uri, "/") == 0 || strcmp(uri, "/index.html") == 0) {
        http_serve_static(c, web_index_html, web_index_html_len, "text/html");
    }
    else if (strcmp(uri, "/settings") == 0 || strcmp(uri, "/settings.html") == 0) {
        http_serve_static(c, web_settings_html, web_settings_html_len, "text/html");
    }
    else if (strcmp(uri, "/apps") == 0 || strcmp(uri, "/apps.html") == 0) {
        http_serve_static(c, web_apps_html, web_apps_html_len, "text/html");
    }
    else if (strcmp(uri, "/status.json") == 0) {
        http_handle_status_json(c);
    }
    else if (strcmp(uri, "/config.json") == 0) {
        if (c->method == HTTP_POST)
            http_handle_config_post(c);
        else
            http_handle_config_get(c);
    }
    else if (strcmp(uri, "/config/save") == 0) {
        http_handle_config_save(c);
    }
    else if (strcmp(uri, "/config/revert") == 0) {
        http_handle_config_revert(c);
    }
    else if (strcmp(uri, "/toggle") == 0) {
        http_handle_toggle(c);
    }
    else if (strcmp(uri, "/info.json") == 0) {
        http_handle_info_json(c);
    }
    else if (strncmp(uri, "/app/", 5) == 0) {
        http_handle_app_control(c);
    }
    else {
        http_serve_404(c);
    }
}

/* ---------- Response helpers ---------- */
static void http_build_header(http_conn_t *c, int status_code, const char *status_text,
                              const char *content_type, uint32_t content_length)
{
    c->resp_header_len = snprintf(c->resp_header, HTTP_HEADER_SIZE,
        "HTTP/1.1 %d %s\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %lu\r\n"
        "Connection: close\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "\r\n",
        status_code, status_text, content_type, (unsigned long)content_length);
}

static void http_serve_static(http_conn_t *c, const uint8_t *data, uint32_t len,
                              const char *content_type)
{
    http_build_header(c, 200, "OK", content_type, len);
    c->resp_body = data;
    c->resp_body_len = len;
}

static void http_serve_json(http_conn_t *c, const char *json, uint16_t len)
{
    http_build_header(c, 200, "OK", "application/json", len);
    c->resp_body = (const uint8_t *)json;
    c->resp_body_len = len;
}

static void http_serve_json_ok(http_conn_t *c, bool ok)
{
    int len = snprintf(json_buf, JSON_BUF_SIZE, "{\"ok\":%s}", ok ? "true" : "false");
    http_serve_json(c, json_buf, len);
}

static void http_serve_redirect(http_conn_t *c, const char *location)
{
    c->resp_header_len = snprintf(c->resp_header, HTTP_HEADER_SIZE,
        "HTTP/1.1 303 See Other\r\n"
        "Location: %s\r\n"
        "Content-Length: 0\r\n"
        "Connection: close\r\n"
        "\r\n", location);
    c->resp_body = NULL;
    c->resp_body_len = 0;
}

static void http_serve_404(http_conn_t *c)
{
    static const char body[] = "Not Found";
    http_build_header(c, 404, "Not Found", "text/plain", sizeof(body) - 1);
    c->resp_body = (const uint8_t *)body;
    c->resp_body_len = sizeof(body) - 1;
}

/* ---------- Chunked send ---------- */
static void http_send_chunk(http_conn_t *c)
{
    if (c->state == HTTP_SOCK_SEND_HEADER) {
        uint16_t remaining = c->resp_header_len - c->resp_header_sent;
        if (remaining > 0) {
            uint16_t chunk = (remaining > HTTP_SEND_CHUNK) ? HTTP_SEND_CHUNK : remaining;
            w5500_send(c->socket_num, (const uint8_t *)c->resp_header + c->resp_header_sent, chunk);
            c->resp_header_sent += chunk;
        }
        if (c->resp_header_sent >= c->resp_header_len) {
            if (c->resp_body_len > 0)
                c->state = HTTP_SOCK_SEND_BODY;
            else
                c->state = HTTP_SOCK_CLOSE;
        }
        return;
    }

    /* SEND_BODY */
    uint32_t remaining = c->resp_body_len - c->resp_body_sent;
    if (remaining > 0 && c->resp_body) {
        uint16_t chunk = (remaining > HTTP_SEND_CHUNK) ? HTTP_SEND_CHUNK : (uint16_t)remaining;
        w5500_send(c->socket_num, c->resp_body + c->resp_body_sent, chunk);
        c->resp_body_sent += chunk;
    }
    if (c->resp_body_sent >= c->resp_body_len) {
        c->state = HTTP_SOCK_CLOSE;
    }
}

/* ---------- JSON API handlers ---------- */
static void http_handle_status_json(http_conn_t *c)
{
    user_config_data_t *cfg = config_get();
    uint8_t din_count  = dio_get_input_count();
    uint8_t dout_count = dio_get_output_count();
    uint8_t ain_count  = aio_get_input_count();
    uint8_t aout_count = aio_get_output_count();

    int pos = 0;

    /* Digital inputs */
    pos += snprintf(json_buf + pos, JSON_BUF_SIZE - pos, "{\"din\":[");
    for (uint8_t i = 0; i < din_count; i++) {
        pos += snprintf(json_buf + pos, JSON_BUF_SIZE - pos,
            "%s{\"n\":\"%s\",\"v\":%d}",
            i ? "," : "",
            cfg->io[CONFIG_IO_IDX_DIN + i].name,
            dio_input_get(i) ? 1 : 0);
    }

    /* Digital outputs */
    pos += snprintf(json_buf + pos, JSON_BUF_SIZE - pos, "],\"dout\":[");
    for (uint8_t i = 0; i < dout_count; i++) {
        pos += snprintf(json_buf + pos, JSON_BUF_SIZE - pos,
            "%s{\"n\":\"%s\",\"v\":%d}",
            i ? "," : "",
            cfg->io[CONFIG_IO_IDX_DOUT + i].name,
            dio_output_get(i) ? 1 : 0);
    }

    /* Analog inputs */
    pos += snprintf(json_buf + pos, JSON_BUF_SIZE - pos, "],\"ain\":[");
    for (uint8_t i = 0; i < ain_count; i++) {
        pos += snprintf(json_buf + pos, JSON_BUF_SIZE - pos,
            "%s{\"n\":\"%s\",\"v\":%u}",
            i ? "," : "",
            cfg->io[CONFIG_IO_IDX_AIN + i].name,
            aio_input_get(i));
    }

    /* Analog outputs */
    pos += snprintf(json_buf + pos, JSON_BUF_SIZE - pos, "],\"aout\":[");
    for (uint8_t i = 0; i < aout_count; i++) {
        pos += snprintf(json_buf + pos, JSON_BUF_SIZE - pos,
            "%s{\"n\":\"%s\",\"v\":%u}",
            i ? "," : "",
            cfg->io[CONFIG_IO_IDX_AOUT + i].name,
            aio_output_get(i));
    }

    /* App status */
    pos += snprintf(json_buf + pos, JSON_BUF_SIZE - pos, "],\"apps\":[");
    for (uint8_t i = 0; i < NUM_APPS; i++) {
        pos += snprintf(json_buf + pos, JSON_BUF_SIZE - pos,
            "%s{\"r\":%d,\"l\":%d,\"nl\":%d}",
            i ? "," : "",
            apps[i].running ? 1 : 0,
            apps[i].current_line,
            apps[i].num_lines);
    }

    /* Dirty flag */
    pos += snprintf(json_buf + pos, JSON_BUF_SIZE - pos,
        "],\"dirty\":%s}", config_get_state() == CONFIG_DIRTY ? "true" : "false");

    http_serve_json(c, json_buf, pos);
}

static void http_handle_config_get(http_conn_t *c)
{
    user_config_data_t *cfg = config_get();
    int pos = 0;

    pos += snprintf(json_buf + pos, JSON_BUF_SIZE - pos,
        "{\"device_name\":\"%s\","
        "\"lcd_enabled\":%s,"
        "\"ip_mode\":%d,"
        "\"static_ip\":\"%d.%d.%d.%d\","
        "\"subnet\":\"%d.%d.%d.%d\","
        "\"gateway\":\"%d.%d.%d.%d\","
        "\"dns\":\"%d.%d.%d.%d\","
        "\"mqtt_enabled\":%s,"
        "\"mqtt_broker_host\":\"%s\","
        "\"mqtt_broker_port\":%u,"
        "\"mqtt_username\":\"%s\","
        "\"mqtt_root_topic\":\"%s\","
        "\"mqtt_append_node_id\":%s,"
        "\"mqtt_client_id_prefix\":\"%s\","
        "\"mqtt_keepalive_sec\":%u,"
        "\"mqtt_reconnect_ms\":%lu,"
        "\"mqtt_analog_interval_s\":%u,",
        cfg->device_name,
        cfg->lcd_enabled ? "true" : "false",
        cfg->ip_mode,
        cfg->static_ip[0], cfg->static_ip[1], cfg->static_ip[2], cfg->static_ip[3],
        cfg->subnet[0], cfg->subnet[1], cfg->subnet[2], cfg->subnet[3],
        cfg->gateway[0], cfg->gateway[1], cfg->gateway[2], cfg->gateway[3],
        cfg->dns[0], cfg->dns[1], cfg->dns[2], cfg->dns[3],
        cfg->mqtt_enabled ? "true" : "false",
        cfg->mqtt_broker_host,
        cfg->mqtt_broker_port,
        cfg->mqtt_username,
        cfg->mqtt_root_topic,
        cfg->mqtt_append_node_id ? "true" : "false",
        cfg->mqtt_client_id_prefix,
        cfg->mqtt_keepalive_sec,
        (unsigned long)cfg->mqtt_reconnect_ms,
        cfg->mqtt_analog_interval_s);

    /* I/O config array */
    pos += snprintf(json_buf + pos, JSON_BUF_SIZE - pos, "\"io\":[");
    for (uint8_t i = 0; i < CONFIG_IO_MAX; i++) {
        io_config_entry_t *io = &cfg->io[i];
        if (io->name[0] == '\0') continue;
        pos += snprintf(json_buf + pos, JSON_BUF_SIZE - pos,
            "%s{\"i\":%d,\"n\":\"%s\",\"on\":\"%s\",\"off\":\"%s\",\"str\":%s}",
            (pos > 0 && json_buf[pos-1] != '[') ? "," : "",
            i, io->name, io->mqtt_on_value, io->mqtt_off_value,
            io->mqtt_value_is_string ? "true" : "false");
    }
    pos += snprintf(json_buf + pos, JSON_BUF_SIZE - pos,
        "],\"dirty\":%s}", config_get_state() == CONFIG_DIRTY ? "true" : "false");

    http_serve_json(c, json_buf, pos);
}

/* Simple JSON key-value parser for POST body (no external library) */
static bool json_find_string(const char *json, const char *key, char *out, uint16_t out_size)
{
    char search[32];
    snprintf(search, sizeof(search), "\"%s\":\"", key);
    const char *p = strstr(json, search);
    if (!p) return false;
    p += strlen(search);
    const char *end = strchr(p, '"');
    if (!end) return false;
    uint16_t len = end - p;
    if (len >= out_size) len = out_size - 1;
    memcpy(out, p, len);
    out[len] = '\0';
    return true;
}

static bool json_find_int(const char *json, const char *key, int *out)
{
    char search[32];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return false;
    p += strlen(search);
    while (*p == ' ') p++;
    *out = atoi(p);
    return true;
}

static bool json_find_bool(const char *json, const char *key, bool *out)
{
    char search[32];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return false;
    p += strlen(search);
    while (*p == ' ') p++;
    *out = (*p == 't');
    return true;
}

static bool json_find_ip(const char *json, const char *key, uint8_t *ip)
{
    char str[20];
    if (!json_find_string(json, key, str, sizeof(str))) return false;
    int a, b, c2, d;
    if (sscanf(str, "%d.%d.%d.%d", &a, &b, &c2, &d) == 4) {
        ip[0] = a; ip[1] = b; ip[2] = c2; ip[3] = d;
        return true;
    }
    return false;
}

static void http_handle_config_post(http_conn_t *c)
{
    user_config_data_t *cfg = config_get();
    const char *body = (const char *)&c->rx_buf[c->body_offset];

    /* Parse top-level fields */
    json_find_string(body, "device_name", cfg->device_name, sizeof(cfg->device_name));
    json_find_bool(body, "lcd_enabled", &cfg->lcd_enabled);

    int int_val;
    if (json_find_int(body, "ip_mode", &int_val)) cfg->ip_mode = int_val;
    json_find_ip(body, "static_ip", cfg->static_ip);
    json_find_ip(body, "subnet", cfg->subnet);
    json_find_ip(body, "gateway", cfg->gateway);
    json_find_ip(body, "dns", cfg->dns);

    json_find_bool(body, "mqtt_enabled", &cfg->mqtt_enabled);
    json_find_string(body, "mqtt_broker_host", cfg->mqtt_broker_host, sizeof(cfg->mqtt_broker_host));
    if (json_find_int(body, "mqtt_broker_port", &int_val)) cfg->mqtt_broker_port = int_val;
    json_find_string(body, "mqtt_username", cfg->mqtt_username, sizeof(cfg->mqtt_username));
    json_find_string(body, "mqtt_password", cfg->mqtt_password, sizeof(cfg->mqtt_password));
    json_find_string(body, "mqtt_root_topic", cfg->mqtt_root_topic, sizeof(cfg->mqtt_root_topic));
    json_find_bool(body, "mqtt_append_node_id", &cfg->mqtt_append_node_id);
    json_find_string(body, "mqtt_client_id_prefix", cfg->mqtt_client_id_prefix, sizeof(cfg->mqtt_client_id_prefix));
    if (json_find_int(body, "mqtt_keepalive_sec", &int_val)) cfg->mqtt_keepalive_sec = int_val;
    if (json_find_int(body, "mqtt_reconnect_ms", &int_val)) cfg->mqtt_reconnect_ms = int_val;
    if (json_find_int(body, "mqtt_analog_interval_s", &int_val)) cfg->mqtt_analog_interval_s = int_val;

    /* Note: I/O config array parsing is done separately via individual /io endpoint
       or included inline — for now we handle it in a simplified manner.
       Full I/O array parsing can be added as needed. */

    config_mark_dirty();

    /* Validate I/O name uniqueness */
    bool valid = config_validate_io_names();
    int len = snprintf(json_buf, JSON_BUF_SIZE,
        "{\"ok\":%s%s}",
        valid ? "true" : "false",
        valid ? "" : ",\"error\":\"Duplicate I/O names\"");
    http_serve_json(c, json_buf, len);
}

static void http_handle_config_save(http_conn_t *c)
{
    bool ok = config_save();
    http_serve_json_ok(c, ok);
}

static void http_handle_config_revert(http_conn_t *c)
{
    config_revert();
    http_serve_json_ok(c, true);
}

static void http_handle_toggle(http_conn_t *c)
{
    char *p = strstr(c->query, "num=");
    if (p) {
        int num = atoi(p + 4);
        if (num >= 0 && num < dio_get_output_count()) {
            bool cur = dio_output_get(num);
            dio_output_set(num, !cur);
        }
    }
    /* Return updated status */
    http_handle_status_json(c);
}

static void http_handle_info_json(http_conn_t *c)
{
    user_config_data_t *cfg = config_get();
    uint8_t ip[4];
    w5500_get_ip(ip);

    board_type_t board = board_get_type();
    const char *board_name = (board == BOARD_TYPE_BABY) ? "Baby" : "Mama";

    int len = snprintf(json_buf, JSON_BUF_SIZE,
        "{\"board\":\"%s\","
        "\"fw_version\":\"" FW_VERSION "\","
        "\"node_id\":%d,"
        "\"uptime\":%lu,"
        "\"ip\":\"%d.%d.%d.%d\","
        "\"device_name\":\"%s\","
        "\"mqtt_connected\":%s}",
        board_name,
        node_id_get(),
        (unsigned long)(HAL_GetTick() / 1000),
        ip[0], ip[1], ip[2], ip[3],
        cfg->device_name,
        mqtt_client_is_connected() ? "true" : "false");

    http_serve_json(c, json_buf, len);
}

static void http_handle_app_control(http_conn_t *c)
{
    /* Parse /app/N/start or /app/N/stop */
    if (strlen(c->uri) < 7) { http_serve_404(c); return; }

    uint8_t slot = c->uri[5] - '0';
    if (slot >= NUM_APPS) { http_serve_json_ok(c, false); return; }

    if (strstr(c->uri, "/start")) {
        app_start(slot);
        http_serve_json_ok(c, true);
    } else if (strstr(c->uri, "/stop")) {
        app_stop(slot);
        http_serve_json_ok(c, true);
    } else {
        http_serve_404(c);
    }
}
