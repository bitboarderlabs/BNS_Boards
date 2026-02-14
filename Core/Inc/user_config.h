/* Core/Inc/user_config.h — User configuration storage system */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "board_id.h"

/* Flash storage — Sector 10 */
#define CONFIG_MAGIC        0x424E53CF  /* "BNS" + 0xCF */
#define CONFIG_VERSION      1
#define CONFIG_FLASH_ADDR   0x080C0000
#define CONFIG_FLASH_SECTOR FLASH_SECTOR_10

/* Max I/O points across all board types (Mama max: 12 DIN + 8 DOUT + 4 AIN + 2 AOUT) */
#define CONFIG_IO_MAX_DIN   12
#define CONFIG_IO_MAX_DOUT   8
#define CONFIG_IO_MAX_AIN    4
#define CONFIG_IO_MAX_AOUT   2
#define CONFIG_IO_MAX       (CONFIG_IO_MAX_DIN + CONFIG_IO_MAX_DOUT + \
                             CONFIG_IO_MAX_AIN + CONFIG_IO_MAX_AOUT)  /* 26 */

/* I/O config index layout: [0..11]=DIN, [12..19]=DOUT, [20..23]=AIN, [24..25]=AOUT */
#define CONFIG_IO_IDX_DIN   0
#define CONFIG_IO_IDX_DOUT  CONFIG_IO_MAX_DIN
#define CONFIG_IO_IDX_AIN   (CONFIG_IO_IDX_DOUT + CONFIG_IO_MAX_DOUT)
#define CONFIG_IO_IDX_AOUT  (CONFIG_IO_IDX_AIN + CONFIG_IO_MAX_AIN)

/* ---------- Per-I/O configuration ---------- */
typedef struct __attribute__((packed)) {
    char name[16];              /* I/O name — must be unique across all I/O */
    char mqtt_on_value[16];     /* Digital only: payload for ON state */
    char mqtt_off_value[16];    /* Digital only: payload for OFF state */
    bool mqtt_value_is_string;  /* false = raw payload, true = JSON string */
} io_config_entry_t;            /* 49 bytes */

/* ---------- Main configuration data ---------- */
typedef struct __attribute__((packed)) {
    /* Device identity */
    char    device_name[32];
    bool    lcd_enabled;

    /* Network */
    uint8_t ip_mode;            /* 0 = Static, 1 = DHCP */
    uint8_t static_ip[4];
    uint8_t subnet[4];
    uint8_t gateway[4];
    uint8_t dns[4];

    /* MQTT */
    bool     mqtt_enabled;
    char     mqtt_broker_host[64];
    uint16_t mqtt_broker_port;
    char     mqtt_username[32];
    char     mqtt_password[32];
    char     mqtt_root_topic[64];
    bool     mqtt_append_node_id;
    char     mqtt_client_id_prefix[16];
    uint16_t mqtt_keepalive_sec;
    uint32_t mqtt_reconnect_ms;
    uint16_t mqtt_analog_interval_s;

    /* Per-I/O configuration */
    io_config_entry_t io[CONFIG_IO_MAX];
} user_config_data_t;

/* ---------- Flash envelope ---------- */
typedef struct __attribute__((packed)) {
    uint32_t           magic;
    uint16_t           version;
    user_config_data_t data;
    uint32_t           crc32;
} user_config_flash_t;

/* ---------- Runtime state ---------- */
typedef enum {
    CONFIG_CLEAN,       /* RAM matches flash */
    CONFIG_DIRTY,       /* RAM has unsaved changes */
    CONFIG_DEFAULTS     /* No valid config in flash, using defaults */
} config_state_t;

/* ---------- Public API ---------- */
void                config_init(board_type_t board);
user_config_data_t* config_get(void);
config_state_t      config_get_state(void);
bool                config_save(void);
void                config_revert(void);
void                config_mark_dirty(void);
void                config_set_defaults(board_type_t board);
bool                config_validate_io_names(void);
