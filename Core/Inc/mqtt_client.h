/* Core/Inc/mqtt_client.h — MQTT 3.1.1 client over W5500 socket 5 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

void mqtt_client_init(void);
void mqtt_client_task(void);
bool mqtt_client_is_connected(void);
