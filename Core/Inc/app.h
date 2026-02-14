/* New file: Core/Inc/app.h */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "board_id.h"

#define MAX_LINES 100
#define NUM_APPS 4
#define APP_VERSION_LEN 10
#define MQTT_TABLE_ENTRIES 8

typedef struct {
    char topic[64];     /* MQTT topic (absolute path) */
    char payload[32];   /* PUBLISH: value to send; WAIT: value to match */
} mqtt_app_entry_t;

typedef enum {
    APP_CMD_ACTIVATE_OUTPUT = 0x01,
    APP_CMD_DEACTIVATE_OUTPUT = 0x02,
    APP_CMD_TEST_INPUT = 0x03,
    APP_CMD_WAIT_INPUT_TRUE = 0x04,
    APP_CMD_DELAY = 0x05,
    APP_CMD_GOTO = 0x06,
    APP_CMD_WAIT_INPUT_FALSE = 0x07,
    APP_CMD_WAIT_FOR_MSG = 0x08,
    APP_CMD_SEND_MSG = 0x09,
    APP_CMD_TEST_FOR_MSG = 0x12,
    APP_CMD_ALL_OUTPUTS_OFF = 0x13,
    APP_CMD_MQTT_PUBLISH = 0x14,
    APP_CMD_MQTT_WAIT = 0x15,
} app_command_t;

typedef struct {
    uint8_t command;
    uint8_t port;
    uint8_t goto_true;
    uint8_t goto_false;
    uint32_t param1;
} app_line_t;

typedef struct {
    app_line_t lines[MAX_LINES];
    uint8_t	app_version[APP_VERSION_LEN];
    uint8_t num_lines;
    bool running;
    uint8_t current_line;
    uint32_t delay_end_time;
    bool in_delay;
    //uint8_t last_msg;
    uint16_t incoming_msg;
    uint16_t outgoing_msg;
    bool has_msg;
    mqtt_app_entry_t mqtt_table[MQTT_TABLE_ENTRIES];
} app_t;

extern app_t apps[NUM_APPS];

void app_init(void);
void app_task(void);
void app_load_from_flash(void);
void app_save_to_flash(void);
void app_start(uint8_t slot);
void app_stop(uint8_t slot);
void app_receive_msg(uint8_t slot, uint8_t msg);
