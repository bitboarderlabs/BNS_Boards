/* Core/Inc/packet.h */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "comm.h"

#define PACKET_ESC      0x1B
#define PACKET_STX      0x02
#define PACKET_ETX      0x03

//#define MIN_DATA_SIZE 		1  // Adjusted
//#define MAX_DATA_SIZE		90
//#define MAX_DATAPACKETLEN	(5 + MAX_DATA_SIZE + 3)
#define MAX_PAYLOAD                     200     // Safe max

#define CMD_SETOUTPUTS   			0x01
#define CMD_GETINPUTS    			0x02
#define CMD_APP_STEP_UPDATE			0x03
#define CMD_BEGIN_END_APP_EXECUTION	0x04	// Payload: app_slot (uint8), action (uint8: 0=stop, 1=start)
#define CMD_APP_MESSAGE        		0x05  	// Payload: app_slot (uint8), value (uint8)
#define CMD_APP_STATUS				0x06
#define CMD_GETOUTPUTS   			0x07

#define CMD_EXTENDED_FUNCS			0x08
#define CMD_CONFIG_LAYOUT			0x09

#define CMD_APP_LOAD_START  0x11  // Payload: app_slot (uint8)
#define CMD_APP_LOAD_LINE   0x12  // Payload: app_slot (uint8), line_num (uint8), command (uint8), port (uint8), goto_true (uint8), goto_false (uint8), param1 (uint32 as 4 bytes)
#define CMD_APP_LOAD_END    0x13  // Payload: app_slot (uint8)

#define CMD_APP_GETVERSION	0x20  // Payload: app_slot (uint8)
#define CMD_APP_SETVERSION	0x21  // Payload: app_slot (uint8), value (10 ascii chars from 0x20 (space) to 0x7e (~) )
#define CMD_ACK          	0xFE
#define CMD_NAK          	0xFF

	//msg format: [ESC][STX][FromId][DataSize][ToId][Data...........][CRC][ESC][ETX]
	//                                             /\--------------/\  				= DataSize
	//            /\-----------------------------------------------/\    /\---/\  	= Included in CRC calculation
	//
	// total size:  1    1      1       1       1  /\------ n -----/\  1    1    1  = 8 + n, where n=[DataSize]
	//
	// Note: the "CRC" embedded in the message includes everything except the CRC field itself and the ETX at
	// the end.  So it DOES include the ESC STX at the beginning and the ESC that follows the CRC, but
	// oddly it doesn't include the ETX at the end.  Weird.

void packet_init(uint8_t node_id);
void packet_feed_byte(uint8_t channel, uint8_t byte);  // renamed from packet_parse
void packet_sendNew(uint8_t to_id, const uint8_t *payload, uint8_t payload_len, uint8_t channel);
void packet_send_ack(uint8_t dest_id, uint8_t channel);
void packet_send_nak(uint8_t dest_id, uint8_t channel);
