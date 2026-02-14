/* Core/Src/comm.c */
#include "comm.h"
#include <stdio.h>   // For vsnprintf
#include <stdarg.h>  // For va_list
#include "packet.h"
#include "bridge.h"
#include "w5500.h"
#include "status_led.h"
#include <string.h>
#include "usbd_cdc_if.h"	// For CDC_Transmit_FS


bool debug_use_rs422 = true;

volatile bool usb_tx_busy = false;

/* --------------------------------------------------------------------- */
/* RX circular buffer (90 bytes max message)                             */
#define RX_BUF_SIZE   256  // Increased for stability
typedef struct {
    uint8_t  buf[RX_BUF_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t used;
} circ_buf_t;

static circ_buf_t rx_buf[CH_COUNT];
static uint8_t my_node_id;

/* --------------------------------------------------------------------- */
// ----RS422 specific----
extern UART_HandleTypeDef huart3;		//rs422
static bool crossover = false;			//rs422
static uint8_t rx_dma_buffer[1];
static volatile bool uart_rx_ready = false;
static volatile uint8_t uart_rx_byte;

static uint16_t rs422_de_normal_pin 	= GPIO_PIN_10;
static uint16_t rs422_de_reverse_pin 	= GPIO_PIN_11;
static uint16_t rs422_nre_normal_pin 	= GPIO_PIN_12;
static uint16_t rs422_nre_reverse_pin 	= GPIO_PIN_13;

static uint16_t rs422_de_pin = GPIO_PIN_10;	// rs422 de pin: D10 = normal, D11 = crossover
//static uint16_t rs422_re_pin = rs422_re_normal_pin; // rs422 re pin: D12 = normal, D13 = crossover

/* RS422 TX Queue */
static uint8_t rs422_tx_queue[RS422_TX_QUEUE_SIZE];
static volatile uint16_t rs422_tx_head = 0;
static volatile uint16_t rs422_tx_tail = 0;
static volatile bool rs422_tx_active = false;


/* --------------------------------------------------------------------- */
// ----DHCP specific----
static bool     dhcp_done      = false;



static uint32_t stack_canary[8] = {0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF,
                                  0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF};

void check_stack_canary(void)
{
    for (int i = 0; i < 8; i++) {
        if (stack_canary[i] != 0xDEADBEEF) {
            debug_println("STACK OVERFLOW DETECTED!");
            while (1);
        }
    }
}





/* --------------------------------------------------------------------- */
static void circ_init(circ_buf_t *cb) {
    cb->head = cb->tail = cb->used = 0;
}

static bool circ_push(circ_buf_t *cb, uint8_t byte) {
    if (cb->used >= RX_BUF_SIZE) return false;
    cb->buf[cb->head] = byte;
    cb->head = (cb->head + 1) % RX_BUF_SIZE;
    cb->used++;
    return true;
}

static bool circ_pop(circ_buf_t *cb, uint8_t *byte) {
    if (cb->used == 0) return false;
    *byte = cb->buf[cb->tail];
    cb->tail = (cb->tail + 1) % RX_BUF_SIZE;
    cb->used--;
    return true;
}

/* --------------------------------------------------------------------- */
void comm_init(uint8_t node_id) {
    my_node_id = node_id;

    packet_init(node_id);

    for (uint8_t ch = 0; ch < CH_COUNT; ch++) {
        circ_init(&rx_buf[ch]);
    }

    bridge_init();

    // IP mode already set by main via w5500_set_ip_mode() before comm_init()
	w5500_init();

    // RS422 init
    rs422_set_crossover(false);
    HAL_UART_Receive_IT(&huart3, rx_dma_buffer, 1);  // Start RX IT

    // DHCP if enabled (called in w5500_init already)
    dhcp_done = (w5500_get_ip_mode() == IP_MODE_STATIC);
}

void rs422_set_crossover(bool enable) {
    crossover = enable;
    //rs422_de_pin = crossover ? GPIO_PIN_11 : GPIO_PIN_10;
    if(crossover == false){
    	//set up de pin for normal tx
    	rs422_de_pin = rs422_de_normal_pin;

		//enable nRE pin for normal receive
    	HAL_GPIO_WritePin(GPIOD, rs422_nre_reverse_pin, GPIO_PIN_SET);	 //Disable crossover nRE
    	HAL_GPIO_WritePin(GPIOD, rs422_nre_normal_pin, GPIO_PIN_RESET);  //Enable  normal nRE

    }else{
    	//set up de pin for crossover tx
    	rs422_de_pin = rs422_de_reverse_pin;

    	//enable nRE pin for crossover receive
		HAL_GPIO_WritePin(GPIOD, rs422_nre_normal_pin, GPIO_PIN_SET);  		//Disable normal nRE
		HAL_GPIO_WritePin(GPIOD, rs422_nre_reverse_pin, GPIO_PIN_RESET);	//Enable crossover nRE
    }

    //Turn off both de pins to make sure only the correct (new) pin will be turned on later.
    HAL_GPIO_WritePin(GPIOD, rs422_de_normal_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, rs422_de_reverse_pin, GPIO_PIN_RESET);
}

/* --------------------------------------------------------------------- */
// Forward declare to fix implicit declaration
static bool rs422_dequeue(uint8_t *byte);

// RS422 TX queue (non-blocking)
void rs422_queue(const uint8_t* data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next_head = (rs422_tx_head + 1) % RS422_TX_QUEUE_SIZE;
        if (next_head == rs422_tx_tail) return;  // Queue full, drop
        rs422_tx_queue[rs422_tx_head] = data[i];
        rs422_tx_head = next_head;
    }

    if (!rs422_tx_active) {
        rs422_tx_active = true;
        HAL_GPIO_WritePin(GPIOD, rs422_de_pin, GPIO_PIN_SET);  // Enable TX
        uint8_t byte;
        if (rs422_dequeue(&byte)) {
            HAL_UART_Transmit_IT(&huart3, &byte, 1);
        } else {
            rs422_tx_active = false;
            HAL_GPIO_WritePin(GPIOD, rs422_de_pin, GPIO_PIN_RESET);
        }
    }
}

static bool rs422_dequeue(uint8_t *byte) {
    if (rs422_tx_head == rs422_tx_tail) return false;
    *byte = rs422_tx_queue[rs422_tx_tail];
    rs422_tx_tail = (rs422_tx_tail + 1) % RS422_TX_QUEUE_SIZE;
    return true;
}

/* --------------------------------------------------------------------- */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart3) {
        uint8_t byte;
        if (rs422_dequeue(&byte)) {
            HAL_UART_Transmit_IT(&huart3, &byte, 1);
        } else {
            rs422_tx_active = false;
            HAL_GPIO_WritePin(GPIOD, rs422_de_pin, GPIO_PIN_RESET);
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart3) {
        comm_feed_byte(CH_RS422, rx_dma_buffer[0]);
        HAL_UART_Receive_IT(&huart3, rx_dma_buffer, 1);  // Restart
    }
}

/* --------------------------------------------------------------------- */
void comm_feed_byte(uint8_t channel, uint8_t byte) {
    if (channel >= CH_COUNT) return;
    if (!circ_push(&rx_buf[channel], byte)) {
        //debug_println("RX overflow on ch");
    }
}

/* --------------------------------------------------------------------- */
void comm_task(void) {
    // Handle RS422 RX
    uint8_t byte;
    while (circ_pop(&rx_buf[CH_RS422], &byte)) {
    	packet_feed_byte(CH_RS422, byte);
    }

    // Handle USB RX (assuming CDC callback feeds to comm_feed_byte(CH_USB))
    while (circ_pop(&rx_buf[CH_USB], &byte)) {
    	packet_feed_byte(CH_USB, byte);
    }

    // Handle ETH sockets 0-3
    for (uint8_t s = W5500_APP_SOCKET_START; s < W5500_APP_SOCKET_START + W5500_APP_SOCKET_COUNT; s++) {
        uint8_t status = w5500_socket_read_byte(s, SN_SR);
        uint8_t channel = CH_ETH_BASE + (s - W5500_APP_SOCKET_START);

        switch (status) {
            case SOCK_ESTABLISHED: {
                uint16_t len = w5500_get_rx_size(s);
                if (len > 0) {
                    uint8_t buf[256];
                    uint16_t read_len = len > sizeof(buf) ? sizeof(buf) : len;
                    w5500_read_data(s, buf, read_len);
                    for (uint16_t i = 0; i < read_len; i++) {
                    	packet_feed_byte(channel, buf[i]);
                    }
                }
                // Keep-alive if idle (send empty packet every 5s)
                static uint32_t last_keep[4] = {0};
                if (HAL_GetTick() - last_keep[s] > 5000) {
                    w5500_send(s, NULL, 0);  // Empty send for keep-alive
                    last_keep[s] = HAL_GetTick();
                }
                break;
            }

            case SOCK_CLOSE_WAIT:
                w5500_socket_write_byte(s, SN_CR, SN_CR_DISCON);
                while (w5500_socket_read_byte(s, SN_CR));
                break;

            case SOCK_CLOSED:
                // Reopen
                w5500_socket_write_byte(s, SN_MR, SN_MR_TCP);
                w5500_socket_write_byte(s, SN_PORT, W5500_APP_PORT >> 8);
                w5500_socket_write_byte(s, SN_PORT + 1, W5500_APP_PORT & 0xFF);
                w5500_socket_write_byte(s, SN_CR, SN_CR_OPEN);
                while (w5500_socket_read_byte(s, SN_CR));
                w5500_socket_write_byte(s, SN_CR, SN_CR_LISTEN);
                while (w5500_socket_read_byte(s, SN_CR));
                break;

            default:
                break;
        }

        // Check for timeout
        uint8_t ir = w5500_socket_read_byte(s, SN_IR);
        if (ir & 0x08) {  // TIMEOUT
            w5500_socket_write_byte(s, SN_IR, 0x08);
            w5500_socket_write_byte(s, SN_CR, SN_CR_CLOSE);
        }
    }

    // DHCP poll if not done
    if (!dhcp_done) {
        dhcp_done = w5500_dhcp_run();
    }

    HAL_Delay(1);  // Light delay to avoid CPU hog
}

/* --------------------------------------------------------------------- */
void comm_rs422_send(const uint8_t *data, uint16_t len) {
    rs422_queue(data, len);  // Non-blocking
}

void comm_eth_send(uint8_t socket, const uint8_t *data, uint16_t len) {
    w5500_send(socket, data, len);
}

void comm_usb_send(const uint8_t *data, uint16_t len) {
    while (usb_tx_busy);  // Block if busy
    usb_tx_busy = true;
    if (CDC_Transmit_FS((uint8_t*)data, len) != USBD_OK) {
        usb_tx_busy = false;
    }
}

/* --------------------------------------------------------------------- */
void comm_send_channel(const uint8_t *data, uint16_t len, uint8_t channel) {

	status_led_data_activity();

	if (channel == CH_ALL) {                     /* broadcast */
        comm_rs422_send(data, len);
        for (uint8_t n = 0; n < W5500_APP_SOCKET_COUNT; n++){
        	uint8_t s = W5500_APP_SOCKET_START + n;
        	if (w5500_is_connected(s)){
                comm_eth_send(s, data, len);
        	}
        }
        comm_usb_send(data, len);
        return;
    }

    if (channel == CH_RS422){
    	comm_rs422_send(data, len);
    }
    else if (channel == CH_USB){
    	comm_usb_send(data, len);
    }
    else if (channel >= CH_ETH_BASE && channel < CH_ETH_BASE + W5500_APP_SOCKET_COUNT) {
        uint8_t s = W5500_APP_SOCKET_START + (channel - CH_ETH_BASE);
        if (w5500_is_connected(s))
            comm_eth_send(s, data, len);
    }
}





/* --------------------------------------------------------------------- */
/* Debug print ------------------------------------------------ */
// Usage: debug_log("Holy shit, variable x is %d, what the fuck?\r\n", x);

void debug_print(const char* msg)
{
    if (debug_use_rs422) {
        rs422_queue((uint8_t*)msg, strlen(msg));
    }
}

void debug_println(const char* msg)
{
    if (debug_use_rs422) {
        debug_print(msg);
        rs422_queue((uint8_t*)"\r\n", 2);
    }
}
