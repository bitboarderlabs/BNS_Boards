/* Core/Src/packet.c */
#include "packet.h"
#include "comm.h"
#include "aio.h"
#include "dio.h"
#include "status_led.h"
#include "bridge.h"
#include "app.h"
#include <string.h>

static uint8_t node_id = 0;

/* Per-channel receive state machine */
typedef enum {
    RX_IDLE,
    RX_SEEN_ESC,
    RX_SEEN_ESC_STX,
    RX_GETTING,
    RX_ESCAPED
} rx_state_t;

typedef struct {
    uint8_t buffer[512];
    uint16_t idx;
    rx_state_t state;
   // uint8_t src_addr;
    uint8_t from_id;
    uint8_t to_id;
} channel_rx_t;

static channel_rx_t rx_state[CH_COUNT];

/* App loading state (unchanged) */
static uint8_t load_app_slot[CH_COUNT] = {255};
static uint8_t load_next_line[CH_COUNT] = {0};

void packet_init(uint8_t id)
{
    node_id = id;
    for (int ch = 0; ch < CH_COUNT; ch++) {
        rx_state[ch].idx = 0;
        rx_state[ch].state = RX_IDLE;
        load_app_slot[ch] = 255;
        load_next_line[ch] = 0;
    }
}

/* Stuff a byte if it's ESC */
static inline void stuff_byte(uint8_t **dst, uint8_t byte)
{
    if (byte == PACKET_ESC) {
        *(*dst)++ = PACKET_ESC;
    }
    *(*dst)++ = byte;
}


void packet_sendNew(uint8_t dest_id, const uint8_t *payload, uint8_t payload_len, uint8_t channel){
    uint8_t tx_buf[512];
    uint8_t *p = tx_buf;

    *p++ = PACKET_ESC;
    *p++ = PACKET_STX;
    *p++ = node_id;  //source id (or "fromId")

    // Length byte
    stuff_byte(&p, payload_len);

    // Destination address (or "toId")
    stuff_byte(&p, dest_id);

    // Payload with stuffing
    for (uint16_t i = 0; i < payload_len; i++) {
        stuff_byte(&p, payload[i]);
    }

    // Checksum (includes first ESC and final ESC before ETX)
    uint32_t sum = PACKET_ESC;  // first ESC
    for (uint8_t *s = tx_buf + 1; s < p; s++) sum += *s;
    sum += PACKET_ESC;  // the ESC before ETX
    uint8_t chk = sum & 0xFF;

    // Special VB6 rule
    if (chk == PACKET_ESC) chk = PACKET_ESC + 1;

    *p++ = chk;
    *p++ = PACKET_ESC;
    *p++ = PACKET_ETX;

    comm_send_channel(tx_buf, p - tx_buf, channel);
}

void packet_send_ack(uint8_t dest_id, uint8_t channel)
{
    uint8_t ack[] = {CMD_ACK};
    packet_sendNew(dest_id, ack, sizeof(ack), channel);
}

void packet_send_nak(uint8_t dest_id, uint8_t channel)
{
	uint8_t nak[] = {CMD_NAK};
	packet_sendNew(dest_id, nak, sizeof(nak), channel);
}

/* Unstuff buffer in-place and return new length */
static uint16_t unstuff(uint8_t *buf, uint16_t len)
{
    uint16_t w = 0;
    for (uint16_t r = 0; r < len; r++) {
        if (buf[r] == PACKET_ESC) {
            r++;
            if (r >= len) return 0;  // malformed
        }
        buf[w++] = buf[r];
    }
    return w;
}

/* Main byte feeder */
void packet_feed_byte(uint8_t channel, uint8_t byte)
{
    if (channel >= CH_COUNT) return;
    channel_rx_t *st = &rx_state[channel];

    switch (st->state) {
        case RX_IDLE:
            if (byte == PACKET_ESC) st->state = RX_SEEN_ESC;
            break;

        case RX_SEEN_ESC:
            if (byte == PACKET_STX) {
                st->state = RX_SEEN_ESC_STX;
                st->idx = 0;
            } else {
                st->state = RX_IDLE;
            }
            break;

        case RX_SEEN_ESC_STX:
            st->buffer[st->idx++] = byte;
            if (st->idx == 3) st->state = RX_GETTING;  // src, len, dest collected
            break;

        case RX_GETTING:
            if (byte == PACKET_ESC) {
                st->state = RX_ESCAPED;
            } else {
                st->buffer[st->idx++] = byte;
            }
            break;

        case RX_ESCAPED:
			if (byte == PACKET_ETX) {
				uint16_t raw_len = st->idx;
				uint16_t unstuffed_len = unstuff(st->buffer, raw_len);

				if (unstuffed_len < 4 || st->buffer[1] > MAX_PAYLOAD) goto rx_reset;

				st->from_id        	= st->buffer[0];
				uint8_t payload_len = st->buffer[1];
				st->to_id        	= st->buffer[2];

				if (st->to_id != node_id){
					//TODO: This should route to the communication bridge for forwarding I think?
					goto rx_reset;
				}

				/* CHECKSUM — includes ESC + STX + data + ESC */
				uint32_t sum = PACKET_ESC;
				sum += PACKET_STX;
				for (int i = 0; i < 3 + payload_len; i++) {
					sum += st->buffer[i];
				}
				sum += PACKET_ESC;
				uint8_t calculated = (uint8_t)sum;
				uint8_t received   = st->buffer[3 + payload_len];
				if (received == PACKET_ESC + 1) received = PACKET_ESC;

				if (calculated != received) {
					packet_send_nak(st->from_id, channel);
					goto rx_reset;
				}

				/* Packet valid! */
				const uint8_t *payload = &st->buffer[3];
				bridge_forward(st->buffer - 2, raw_len + 2);

				status_led_data_activity();

				uint8_t cmd = payload[0];
				switch (cmd) {
					case CMD_SETOUTPUTS:
						//set outputs payload: (1, maxIndx, startIndx, 1byte per output...)
						// Unfortunately, the original BNS boards use 1 byte per output in the message rather than using bitmasked byte values.
						// Remote host expects an ACK or NAK reply.
						//[0 cmd=CMD_SETOUTPUTS] [1 Num of dig output bytes included -1] [2 startIndx of dig outputs included] [3 value(startIndx)] [4 value(startIndx+1] ...
						if (payload_len >= 2) {  // cmd + mask
							uint8_t maxIndx = payload[1] +1;
							uint8_t startIndx = payload[2];  //The first output index who's value begins at payload[3]
							uint8_t val = 0;

							if(maxIndx < 1){
								packet_send_nak(st->from_id, channel);  //Not enough data included
								break;
							}

							for(uint8_t i = 0; i < maxIndx; i++){ // i is the loop count
								val = payload[3+i];				//First output's val starts at payload[3]
								if(val == 0){
									dio_output_set(i+startIndx, false);
								}else if(val == 1){
									dio_output_set(i+startIndx, true);
								}else{
									packet_send_nak(st->from_id, channel);  //Invalid value.
									break;
								}
							}
							packet_send_ack(st->from_id, channel);
						} else {
							packet_send_nak(st->from_id, channel);
						}
						break;

					case CMD_GETINPUTS:
						// payload: cmd=2,
						// remote host expects the reply in the following format:
						//  CMD=GETINPUTS, NumDig&AnaBytesTotal, NumDigBytes, [DigInput Bitmasked Values], [AnaInput uint16_t values as (low byte)(high byte) ]
						//	     0                 1                 2             3 to ( [2] + 3 - 1 )
							if (payload_len < 1) {  // just cmd
								packet_send_nak(st->from_id, channel);
								break;
							}
//							uint8_t inputs[4] = {0};
//							uint8_t max = dio_get_input_count();
//							for (uint8_t i = 0; i < max && i < 32; i++) {
//								if (dio_input_get(i)) inputs[i/8] |= (1 << (i%8));
//							}
//							packet_send(st->from_id, CMD_GETINPUTS, inputs, (max+7)/8, channel);

							// Calculate how many bytes needed for dig input data:
							uint8_t numDigInputs = dio_get_input_count();
							uint8_t numDigInBytes = (numDigInputs / 8) + (numDigInputs % 8 > 0 ? 1 : 0);
							uint8_t numAnaInputs = aio_get_input_count();
							uint8_t numAnaInBytes = numAnaInputs * 2;
							uint8_t getInVal[DIO_INPUT_COUNT_MAX + (AIO_INPUT_COUNT_MAX * 2)] = {0};

							uint8_t n = 0;
							getInVal[0] = CMD_GETINPUTS;
							getInVal[1] = numDigInBytes + numAnaInBytes;
							getInVal[2] = numDigInBytes;
							for (uint8_t i = 0; i < numDigInputs; i++) {
								n = 3 + (i/8);
								if (dio_input_get(i)) getInVal[n] |= (1 << (i%8));
							}
							n++;

							for(uint8_t a = 0; a < numAnaInputs; a++){
								uint16_t ainVal = aio_input_get(a);
								getInVal[n] = (uint8_t)(ainVal & 0xFF);
								getInVal[n+1] = (uint8_t)(ainVal >> 8);
								n+=2;
							}

							packet_sendNew(st->from_id, getInVal, n, channel);

						break;

					case CMD_APP_STEP_UPDATE:
						//This is the original BNS method of loading apps into volatile memory.
						//payload: cmd=3, length, startIndex[=0], slot, linenum, lineCmd, portNum, GTT, GTF, Param1[2 bytes]
						//expected response: ack or nak
						if(payload_len == 10+1) {
							uint8_t slot 		= payload[3]; //incoming indexes are all 0-based.
							uint8_t line_num 	= payload[4];
							if (slot < NUM_APPS && line_num < MAX_LINES) {
								app_line_t *line = &apps[slot].lines[line_num];
								line->command     = payload[5];
								line->port        = payload[6];
								line->goto_true   = payload[7];
								line->goto_false  = payload[8];
								line->param1      = (payload[9] << 8) | payload[10];

								if(line_num >= apps[slot].num_lines){
									apps[slot].num_lines = line_num + 1;
								}
								packet_send_ack(st->from_id, channel);

								//If the led mode is still indicating "no app" then advance it to "stopped".
								if(status_led_get_mode() == LED_MODE_NOAPP){
									status_led_set_mode(LED_MODE_STOPPED);
								}
							}else{
								packet_send_nak(st->from_id, channel);
							}
						}else{
							packet_send_nak(st->from_id, channel);
						}
						break;

					case CMD_BEGIN_END_APP_EXECUTION:
						//payload: cmd=4, length=4, startIndex=0 (unused), slotnum, action (0=stop 1=start)
						//expected response: ack or nak
						if (payload_len == 5) {
							uint8_t slot   = payload[3]; 	//Appwriter sends 0-based indexes.
							uint8_t action = payload[4];
							if (slot < NUM_APPS) {
								if (action == 1) app_start(slot);
								else if (action == 0) app_stop(slot);
								packet_send_ack(st->from_id, channel);
							} else {
								packet_send_nak(st->from_id, channel);
							}
						} else {
							packet_send_nak(st->from_id, channel);
						}
						break;

					case CMD_APP_MESSAGE: //(incoming message)
						//payload: cmd=5, length=2, startindex=0, appnumber, inMsg(low), inMsg(high)
						//expected response: ack or nak
						if (payload_len >= 6) {
							uint8_t slot = payload[3];
							uint16_t msg  = (payload[5] << 8) | payload[4];

							if (slot < NUM_APPS) {
								app_receive_msg(slot, msg);
								packet_send_ack(st->from_id, channel);
							} else {
								packet_send_nak(st->from_id, channel);
							}
						} else {
							packet_send_nak(st->from_id, channel);
						}
						break;

					case CMD_APP_STATUS:
						//payload: cmd=6, length=2, startIndex=0, appNum, 0, 0
						//expected response: cmd=6, topicBytes,  startIndex,   appnumber, msgFromBoard(lowbyte), msgFromBoard(highbyte), curLineNumber, appIsRunning
						//						0		1		 	2			3			4						5						6				7
						if (payload_len >= 6) {
							uint8_t slot = payload[3]; //Appwriter does the translation between 0-based and 1-based, so we always use 0-based.
							if (slot < NUM_APPS) {
								uint8_t getAppStat[8];  // ← NOW 7 BYTES
								getAppStat[0] = CMD_APP_STATUS;
								getAppStat[1] = 5;   		// len of data in this sub payload(???) --This is what was seen in all baby board responses and isn't used by appwriter. Just a placeholder for now.
								getAppStat[2] = 0;		// same as resp[0]- this is standard baby board response, unknown use, so keep it as placeholder for now.
								getAppStat[3] = slot;
								getAppStat[4] = (uint8_t)(apps[slot].outgoing_msg & 0xFF);			//outgoing message, low byte
								getAppStat[5] = (uint8_t)((apps[slot].outgoing_msg >> 8) & 0xFF);	//outgoing message, high byte
								getAppStat[6] = apps[slot].current_line;
								getAppStat[7] = apps[slot].running ? 1 : 0;
								packet_sendNew(st->from_id, getAppStat, 8, channel);
								goto rx_reset;
							} else {
								packet_send_nak(st->from_id, channel);
							}
						} else {
							packet_send_nak(st->from_id, channel);
						}
						break;

					case CMD_GETOUTPUTS:
						//payload: cmd=7
						//expected response: cmd=7, topicBytes, startIndex, databytes...
						//						0		1			2			3 to [1]+3
							if (payload_len < 1) {  // just cmd
								packet_send_nak(st->from_id, channel);
								break;
							}
//							uint8_t outputs[2] = {0};
//							uint8_t max = dio_get_output_count();
//							for (uint8_t i = 0; i < max && i < 16; i++) {
//								if (dio_output_get(i)) outputs[i/8] |= (1 << (i%8));
//							}
//							packet_send(st->from_id, CMD_GETOUTPUTS, outputs, (max+7)/8, channel);

							// Calculate how many bytes needed for dig output data:
							uint8_t numDigOutputs = dio_get_output_count();
							uint8_t numDigOutBytes = ((numDigOutputs +7) / 8);
							uint8_t getOutVal[3 + ((DIO_OUTPUT_COUNT_MAX + 7) / 8)] = {0};

							uint8_t d = 0;
							getOutVal[0] = CMD_GETOUTPUTS;
							getOutVal[1] = numDigOutBytes;
							getOutVal[2] = 0;
							for (uint8_t i = 0; i < numDigOutputs; i++) {
								d = 3 + (i/8);
								if (dio_output_get(i)) getOutVal[d] |= (1 << (i%8));
							}
							d++;

							packet_sendNew(st->from_id, getOutVal, d, channel);
						break;


					case CMD_EXTENDED_FUNCS:
					case CMD_CONFIG_LAYOUT:
						break;

					case CMD_APP_LOAD_START:
//						if (payload_len >= 2) {  // cmd + slot
//							uint8_t slot = payload[1] - 1; //Appwriter uses 1-based app number (slot) but we use 0-based indexes.
//							if (slot < NUM_APPS) {
//								load_app_slot[channel] = slot;
//								load_next_line[channel] = 0;
//								apps[slot].num_lines = 0;
//								packet_send_ack(st->from_id, channel);
//							} else {
//								packet_send_nak(st->from_id, channel);
//							}
//						} else {
//							packet_send_nak(st->from_id, channel);
//						}
						break;

					case CMD_APP_LOAD_LINE:
//						if (payload_len >= 7) {  // cmd + slot + line_num + cmd + port + goto_t + goto_f
//							uint8_t slot     = payload[1] - 1; //Appwriter uses 1-based app number (slot) but we use 0-based indexes.
//							uint8_t line_num = payload[2];
//							if (slot == load_app_slot[channel] && line_num == load_next_line[channel] && line_num < MAX_LINES) {
//								app_line_t *line = &apps[slot].lines[line_num];
//								line->command     = payload[3];
//								line->port        = payload[4];
//								line->goto_true   = payload[5];
//								line->goto_false  = payload[6];
//								line->param1      = (payload[7] << 24) | (payload[8] << 16) | (payload[9] << 8) | payload[10];
//								load_next_line[channel]++;
//								apps[slot].num_lines = load_next_line[channel];
//								packet_send_ack(st->from_id, channel);
//							} else {
//								packet_send_nak(st->from_id, channel);
//							}
//						} else {
//							packet_send_nak(st->from_id, channel);
//						}
						break;

					case CMD_APP_LOAD_END:
//						if (payload_len >= 2) {  // cmd + slot
//							uint8_t slot = payload[1] - 1; //Appwriter uses 1-based app number (slot) but we use 0-based indexes.
//							if (slot == load_app_slot[channel]) {
//								app_save_to_flash();
//								load_app_slot[channel] = 255;
//								packet_send_ack(st->from_id, channel);
//							} else {
//								packet_send_nak(st->from_id, channel);
//							}
//						} else {
//							packet_send_nak(st->from_id, channel);
//						}
						break;

					case CMD_APP_GETVERSION:
//						if (payload_len >= 2) {  // cmd + slot
//							uint8_t slot = payload[1] - 1; //Appwriter uses 1-based app number (slot) but we use 0-based indexes.
//							if (slot < NUM_APPS) {
//								uint8_t resp[10];
//								memcpy(resp, apps[slot].app_version, 10);
//								packet_send(st->from_id, CMD_APP_GETVERSION, resp, 10, channel);
//							} else {
//								packet_send_nak(st->from_id, channel);
//							}
//						} else {
//							packet_send_nak(st->from_id, channel);
//						}
						break;

					case CMD_APP_SETVERSION:
//						if (payload_len >= 11) {  // cmd + slot + 10 chars
//							uint8_t slot = payload[1] - 1; //Appwriter uses 1-based app number (slot) but we use 0-based indexes.
//							if (slot < NUM_APPS) {
//								bool valid_chars = true;
//								for (int i = 0; i < 10; i++) {
//									uint8_t c = payload[2 + i];
//									if (c < 0x20 || c > 0x7E) {
//										valid_chars = false;
//										break;
//									}
//								}
//								if (valid_chars) {
//									memcpy(apps[slot].app_version, &payload[2], 10);
//									app_save_to_flash();
//									packet_send_ack(st->from_id, channel);
//								} else {
//									packet_send_nak(st->from_id, channel);
//								}
//							} else {
//								packet_send_nak(st->from_id, channel);
//							}
//						} else {
//							packet_send_nak(st->from_id, channel);
//						}
						break;

					case CMD_ACK:
					case CMD_NAK:
						break;

					default:
						packet_send_ack(st->from_id, channel);
						break;
				}

			rx_reset:
				st->state = RX_IDLE;
				st->idx   = 0;
			} else {
				st->buffer[st->idx++] = byte;
				st->state = RX_GETTING;
			}
			break;
    }
}
