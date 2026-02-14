/* Core/Inc/webserver.h — HTTP web server over W5500 sockets */
#pragma once
#include <stdint.h>
#include <stdbool.h>

void webserver_init(void);
void webserver_task(void);
