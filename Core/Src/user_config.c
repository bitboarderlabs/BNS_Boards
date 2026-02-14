/* Core/Src/user_config.c — User configuration storage system */
#include "user_config.h"
#include "stm32f4xx_hal.h"
#include "dio.h"
#include "aio.h"
#include <string.h>
#include <stdio.h>

/* ---------- Working copy in RAM ---------- */
static user_config_data_t working_config;
static config_state_t     config_state = CONFIG_DEFAULTS;
static board_type_t       config_board;

/* ---------- Direct pointer to flash ---------- */
#define FLASH_CONFIG ((const user_config_flash_t *)CONFIG_FLASH_ADDR)

/* ---------- CRC32 (software, no lookup table) ---------- */
static uint32_t crc32_update(uint32_t crc, uint8_t byte)
{
    crc ^= byte;
    for (int i = 0; i < 8; i++) {
        if (crc & 1)
            crc = (crc >> 1) ^ 0xEDB88320;
        else
            crc >>= 1;
    }
    return crc;
}

static uint32_t crc32_calc(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++)
        crc = crc32_update(crc, data[i]);
    return crc ^ 0xFFFFFFFF;
}

/* ---------- Defaults ---------- */
void config_set_defaults(board_type_t board)
{
    memset(&working_config, 0, sizeof(working_config));

    /* Device identity */
    if (board == BOARD_TYPE_BABY)
        strncpy(working_config.device_name, "BNS-Baby", sizeof(working_config.device_name) - 1);
    else
        strncpy(working_config.device_name, "BNS-Mama", sizeof(working_config.device_name) - 1);

    working_config.lcd_enabled = (board == BOARD_TYPE_MAMA);

    /* Network defaults */
    working_config.ip_mode = 0; /* Static */
    working_config.static_ip[0] = 192; working_config.static_ip[1] = 168;
    working_config.static_ip[2] = 1;   working_config.static_ip[3] = 100;
    working_config.subnet[0] = 255; working_config.subnet[1] = 255;
    working_config.subnet[2] = 255; working_config.subnet[3] = 0;
    working_config.gateway[0] = 192; working_config.gateway[1] = 168;
    working_config.gateway[2] = 1;   working_config.gateway[3] = 1;
    working_config.dns[0] = 8; working_config.dns[1] = 8;
    working_config.dns[2] = 8; working_config.dns[3] = 8;

    /* MQTT defaults */
    working_config.mqtt_enabled = false;
    working_config.mqtt_broker_port = 1883;
    strncpy(working_config.mqtt_root_topic, "/bns/dev", sizeof(working_config.mqtt_root_topic) - 1);
    working_config.mqtt_append_node_id = true;
    strncpy(working_config.mqtt_client_id_prefix, "bns_", sizeof(working_config.mqtt_client_id_prefix) - 1);
    working_config.mqtt_keepalive_sec = 60;
    working_config.mqtt_reconnect_ms = 5000;
    working_config.mqtt_analog_interval_s = 60;

    /* Per-I/O defaults — generate names based on board I/O counts */
    uint8_t din_count  = (board == BOARD_TYPE_BABY) ? DIO_INPUT_COUNT_BABY  : DIO_INPUT_COUNT_MAMA;
    uint8_t dout_count = (board == BOARD_TYPE_BABY) ? DIO_OUTPUT_COUNT_BABY : DIO_OUTPUT_COUNT_MAMA;
    uint8_t ain_count  = (board == BOARD_TYPE_BABY) ? AIO_INPUT_COUNT_BABY  : AIO_INPUT_COUNT_MAMA;
    uint8_t aout_count = (board == BOARD_TYPE_BABY) ? AIO_OUTPUT_COUNT_BABY : AIO_OUTPUT_COUNT_MAMA;

    for (uint8_t i = 0; i < CONFIG_IO_MAX; i++) {
        io_config_entry_t *io = &working_config.io[i];
        strncpy(io->mqtt_on_value,  "1", sizeof(io->mqtt_on_value) - 1);
        strncpy(io->mqtt_off_value, "0", sizeof(io->mqtt_off_value) - 1);
        io->mqtt_value_is_string = false;

        /* Generate default names for active I/O points */
        if (i < CONFIG_IO_IDX_DOUT && i < din_count)
            snprintf(io->name, sizeof(io->name), "din%d", i);
        else if (i >= CONFIG_IO_IDX_DOUT && i < CONFIG_IO_IDX_AIN &&
                 (i - CONFIG_IO_IDX_DOUT) < dout_count)
            snprintf(io->name, sizeof(io->name), "dout%d", i - CONFIG_IO_IDX_DOUT);
        else if (i >= CONFIG_IO_IDX_AIN && i < CONFIG_IO_IDX_AOUT &&
                 (i - CONFIG_IO_IDX_AIN) < ain_count)
            snprintf(io->name, sizeof(io->name), "ain%d", i - CONFIG_IO_IDX_AIN);
        else if (i >= CONFIG_IO_IDX_AOUT &&
                 (i - CONFIG_IO_IDX_AOUT) < aout_count)
            snprintf(io->name, sizeof(io->name), "aout%d", i - CONFIG_IO_IDX_AOUT);
        /* else: unused slot, name stays empty */
    }
}

/* ---------- Init ---------- */
void config_init(board_type_t board)
{
    config_board = board;

    /* Try to load from flash */
    if (FLASH_CONFIG->magic == CONFIG_MAGIC &&
        FLASH_CONFIG->version <= CONFIG_VERSION)
    {
        uint32_t crc = crc32_calc((const uint8_t *)&FLASH_CONFIG->data,
                                  sizeof(user_config_data_t));
        if (crc == FLASH_CONFIG->crc32) {
            memcpy(&working_config, &FLASH_CONFIG->data, sizeof(user_config_data_t));
            config_state = CONFIG_CLEAN;
            return;
        }
    }

    /* Flash invalid or empty — load defaults */
    config_set_defaults(board);
    config_state = CONFIG_DEFAULTS;
}

/* ---------- Accessors ---------- */
user_config_data_t* config_get(void)
{
    return &working_config;
}

config_state_t config_get_state(void)
{
    return config_state;
}

void config_mark_dirty(void)
{
    if (config_state != CONFIG_DIRTY)
        config_state = CONFIG_DIRTY;
}

/* ---------- Save to flash ---------- */
bool config_save(void)
{
    /* Build the flash envelope in a temp buffer */
    user_config_flash_t envelope;
    envelope.magic   = CONFIG_MAGIC;
    envelope.version = CONFIG_VERSION;
    memcpy(&envelope.data, &working_config, sizeof(user_config_data_t));
    envelope.crc32 = crc32_calc((const uint8_t *)&envelope.data,
                                sizeof(user_config_data_t));

    /* Erase sector 10 — same pattern as app_save_to_flash() */
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef eraseInit = {0};
    eraseInit.TypeErase    = FLASH_TYPEERASE_SECTORS;
    eraseInit.Sector       = CONFIG_FLASH_SECTOR;
    eraseInit.NbSectors    = 1;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    uint32_t sectorError = 0;
    if (HAL_FLASHEx_Erase(&eraseInit, &sectorError) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    /* Write byte-by-byte */
    uint8_t *data    = (uint8_t *)&envelope;
    uint32_t address = CONFIG_FLASH_ADDR;
    uint32_t size    = sizeof(user_config_flash_t);

    for (uint32_t i = 0; i < size; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address + i, data[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();

    /* Verify readback */
    if (memcmp((const void *)CONFIG_FLASH_ADDR, &envelope, size) != 0)
        return false;

    config_state = CONFIG_CLEAN;
    return true;
}

/* ---------- Revert ---------- */
void config_revert(void)
{
    config_init(config_board);
}

/* ---------- Validate I/O name uniqueness ---------- */
bool config_validate_io_names(void)
{
    for (uint8_t i = 0; i < CONFIG_IO_MAX; i++) {
        if (working_config.io[i].name[0] == '\0') continue;
        for (uint8_t j = i + 1; j < CONFIG_IO_MAX; j++) {
            if (working_config.io[j].name[0] == '\0') continue;
            if (strcmp(working_config.io[i].name, working_config.io[j].name) == 0)
                return false;
        }
    }
    return true;
}
