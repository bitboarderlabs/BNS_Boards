/* New file: Core/Src/app.c */
#include "app.h"
#include "dio.h"
#include "mqtt_client.h"
#include "packet.h"
#include "status_led.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"
#include <string.h>
#include "board_id.h"  // For board_id_detect()

app_t apps[NUM_APPS];

#define APP_FLASH_ADDR 0x080E0000  // Example: Sector 11 on STM32F407 (128KB sector, adjust if needed)
#define APP_FLASH_SECTOR FLASH_SECTOR_11

static uint8_t host_node_id = 0;  // If unused, can remove

static bool app_validate(app_t *app, uint8_t max_in, uint8_t max_out) {
    if (app->num_lines > MAX_LINES) return false;

    // Validate version characters
    for (int i = 0; i < APP_VERSION_LEN; i++) {
        uint8_t c = app->app_version[i];
        if (c < 0x20 || c > 0x7E) return false;
    }

    for (int l = 0; l < app->num_lines; l++) {
        app_line_t *line = &app->lines[l];
        switch (line->command) {
            case APP_CMD_ACTIVATE_OUTPUT:
            case APP_CMD_DEACTIVATE_OUTPUT:
                if (line->port >= max_out) return false;
                break;
            case APP_CMD_TEST_INPUT:
                if (line->port >= max_in) return false;
                if (line->goto_true >= app->num_lines || line->goto_false >= app->num_lines) return false;
                break;
            case APP_CMD_WAIT_INPUT_TRUE:
            case APP_CMD_WAIT_INPUT_FALSE:
                if (line->port >= max_in) return false;
                break;
            case APP_CMD_DELAY:
                // param1 is uint32_t, no specific validation needed
                break;
            case APP_CMD_GOTO:
                if (line->goto_true >= app->num_lines) return false;
                break;
            case APP_CMD_WAIT_FOR_MSG:
            case APP_CMD_SEND_MSG:
                // param1 is message value (uint32_t, but effectively uint8_t)
                break;
            case APP_CMD_TEST_FOR_MSG:
                if (line->goto_true >= app->num_lines || line->goto_false >= app->num_lines) return false;
                break;
            case APP_CMD_ALL_OUTPUTS_OFF:
                // No parameters to validate
                break;
            case APP_CMD_MQTT_PUBLISH:
                if (line->param1 >= MQTT_TABLE_ENTRIES) return false;
                break;
            case APP_CMD_MQTT_WAIT:
                if (line->param1 >= MQTT_TABLE_ENTRIES) return false;
                if (line->goto_true >= app->num_lines || line->goto_false >= app->num_lines) return false;
                break;
            default:
                return false;  // Unknown command
        }
    }
    return true;
}

void app_init(void) {
    app_load_from_flash();
    // Other init if needed
}

void app_task(void) {
//    board_type_t type = board_get_type();
//    uint8_t max_in = dio_get_input_count();
    uint8_t max_out = dio_get_output_count();
    uint8_t numAppsRunning = 0;

    for (uint8_t a = 0; a < NUM_APPS; a++) {
        app_t *app = &apps[a];
        if (!app->running) continue;

        numAppsRunning++;
        app_line_t *line = &app->lines[app->current_line];

        if (app->in_delay) {
            if (HAL_GetTick() >= app->delay_end_time) {
                app->in_delay = false;
                app->current_line++;
            }
            continue;
        }

        switch (line->command) {
            case APP_CMD_ACTIVATE_OUTPUT:
                dio_output_set(line->port, true);
                app->current_line++;
                break;
            case APP_CMD_DEACTIVATE_OUTPUT:
                dio_output_set(line->port, false);
                app->current_line++;
                break;
            case APP_CMD_TEST_INPUT:
                if (dio_input_get(line->port)) {
                    app->current_line = line->goto_true;
                } else {
                    app->current_line = line->goto_false;
                }
                break;
            case APP_CMD_WAIT_INPUT_TRUE:
                if (dio_input_get(line->port)) {
                    app->current_line++;
                }
                break;
            case APP_CMD_WAIT_INPUT_FALSE:
                if (!dio_input_get(line->port)) {
                    app->current_line++;
                }
                break;
            case APP_CMD_DELAY:
                app->delay_end_time = HAL_GetTick() + line->param1;
                app->in_delay = true;
                break;
            case APP_CMD_GOTO:
                app->current_line = line->goto_true;
                break;
            case APP_CMD_WAIT_FOR_MSG:
                if (app->has_msg && app->incoming_msg == (uint8_t)line->param1) {
                	app->incoming_msg = 0;
                    app->has_msg = false;
                    app->current_line++;
                }
                break;
            case APP_CMD_SEND_MSG:
                // Send message (assuming via packet)
                //uint8_t msg_payload[1] = {(uint8_t)line->param1};
                //packet_send(host_node_id, CMD_APP_MESSAGE, msg_payload, 1, CH_ALL);  // Example
            	app->outgoing_msg = (uint8_t)line->param1;
                app->current_line++;
                break;
            case APP_CMD_TEST_FOR_MSG:
                if (app->has_msg && app->incoming_msg == (uint8_t)line->param1) {
                	app->incoming_msg = 0;
                    app->has_msg = false;
                    app->current_line = line->goto_true;
                } else {
                    app->current_line = line->goto_false;
                }
                break;
            case APP_CMD_ALL_OUTPUTS_OFF:
                //for (uint8_t i = 0; i < max_out; i++) dio_output_set(i, false);
            	dio_all_outputs_off();
                app->current_line++;
                break;
            case APP_CMD_MQTT_PUBLISH:
                if (line->param1 < MQTT_TABLE_ENTRIES) {
                    mqtt_app_entry_t *e = &app->mqtt_table[line->param1];
                    if (e->topic[0] != '\0')
                        mqtt_client_app_publish(e->topic, e->payload);
                }
                app->current_line++;
                break;
            case APP_CMD_MQTT_WAIT:
                if (line->param1 >= MQTT_TABLE_ENTRIES) { app->current_line++; break; }
                {
                    mqtt_app_entry_t *e = &app->mqtt_table[line->param1];
                    if (e->topic[0] == '\0' || !mqtt_client_is_connected()) break; /* block */
                    char rxbuf[32];
                    if (!mqtt_client_app_get_received(a, line->param1, rxbuf, sizeof(rxbuf)))
                        break; /* block — no message yet */
                    mqtt_client_app_clear_received(a, line->param1);
                    if (strcmp(rxbuf, e->payload) == 0)
                        app->current_line = line->goto_true;
                    else
                        app->current_line = line->goto_false;
                }
                break;
            default:
                app->current_line++;
                break;
        }
    }
    status_led_mode_t curLedStatus = status_led_get_mode();
    if( (curLedStatus == LED_MODE_STOPPED) || (curLedStatus == LED_MODE_RUNNING) ){
    	status_led_set_mode((numAppsRunning == 0 ? LED_MODE_STOPPED : LED_MODE_RUNNING));
    }
    status_led_task();
}

void app_load_from_flash(void) {
    memcpy(apps, (void *)APP_FLASH_ADDR, sizeof(apps));

    // Validate after load
//    board_type_t type = board_get_type();
    uint8_t max_in = dio_get_input_count();
    uint8_t max_out = dio_get_output_count();

    status_led_set_mode(LED_MODE_NOAPP);  // Start out as "no app".  If any apps are loaded then it'll change to "stopped".

    for (uint8_t a = 0; a < NUM_APPS; a++) {
        app_t *app = &apps[a];
        if (!app_validate(app, max_in, max_out)) {
            memset(app, 0, sizeof(app_t));  // Zero out the entire app
        }else{
        	status_led_set_mode(LED_MODE_STOPPED);
        }
    }
}

void app_save_to_flash(void) {
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef eraseInit = {0};
    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInit.Sector = APP_FLASH_SECTOR;
    eraseInit.NbSectors = 1;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    uint32_t sectorError = 0;
    if (HAL_FLASHEx_Erase(&eraseInit, &sectorError) != HAL_OK) {
        HAL_FLASH_Lock();
        return;
    }

    uint8_t *data = (uint8_t *)apps;
    uint32_t address = APP_FLASH_ADDR;
    uint32_t size = sizeof(apps);

    for (uint32_t i = 0; i < size; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address + i, data[i]) != HAL_OK) {
            break;
        }
    }

    HAL_FLASH_Lock();
}

void app_start(uint8_t slot) {
    if (slot >= NUM_APPS) return;
    app_t *app = &apps[slot];
    app->running = true;
    app->current_line = 0;
    app->delay_end_time = 0;
    app->in_delay = false;
    app->has_msg = false;
    //status_led_set_mode(LED_MODE_RUNNING);
}

void app_stop(uint8_t slot) {
    if (slot >= NUM_APPS) return;
    apps[slot].running = false;
}

void app_receive_msg(uint8_t slot, uint8_t msg) {
    if (slot >= NUM_APPS) return;
    app_t *app = &apps[slot];
    app->incoming_msg = msg;
    app->has_msg = true;
}
