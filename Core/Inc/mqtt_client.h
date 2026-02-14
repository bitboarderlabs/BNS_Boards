/* Core/Inc/mqtt_client.h — MQTT 3.1.1 client over W5500 socket 5 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

void mqtt_client_init(void);
void mqtt_client_task(void);
bool mqtt_client_is_connected(void);

/* App sequencer MQTT support */
bool mqtt_client_app_publish(const char *topic, const char *payload);
bool mqtt_client_app_get_received(uint8_t app_idx, uint8_t entry_idx,
                                  char *buf, uint16_t buflen);
void mqtt_client_app_clear_received(uint8_t app_idx, uint8_t entry_idx);
