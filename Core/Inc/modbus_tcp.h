/* Core/Inc/modbus_tcp.h — Modbus TCP server over W5500 socket 4 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

void modbus_tcp_init(void);
void modbus_tcp_task(void);
