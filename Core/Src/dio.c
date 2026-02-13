/* Core/Src/dio.c */
#include "dio.h"
#include "stm32f4xx_hal.h"
//#include "web_socket.h"

static board_type_t board;
static uint8_t outputBankAddr[3] = {0x00, 0x01, 0x02};
static uint8_t outputBankCurVal[3] = {0,0,0};

/* ---------- Baby board – direct GPIO ---------- */
#if defined(BOARD_BABY)
    static const uint16_t baby_out_pins[2] = { GPIO_PIN_8, GPIO_PIN_7 };
    static GPIO_TypeDef* const baby_out_port[2] = { GPIOE, GPIOE };

    static const uint16_t baby_in_pins[4] = { GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5 };
    static GPIO_TypeDef* const baby_in_port[4] = { GPIOE, GPIOE, GPIOC, GPIOC };
#endif

/* ---------- Mama / Papa – 8-bit parallel bus ---------- */
#if defined(BOARD_MAMA) || defined(BOARD_PAPA)
    #define IOBUS_DATA_PORT GPIOD
    #define IOBUS_DATA_PINS (GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4|\
                             GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0)

    #define IOBUS_ADDR_PORT GPIOE
    #define IOBUS_ADDR0_PIN GPIO_PIN_8
    #define IOBUS_ADDR1_PIN GPIO_PIN_9
    #define IOBUS_ADDR2_PIN GPIO_PIN_10

    #define IOBUS_OE_PORT GPIOE
    #define IOBUS_OE_PIN  GPIO_PIN_13   /* active-low Output Enable */
	#define OE_ENABLE	GPIO_PIN_RESET
	#define OE_DISABLE	GPIO_PIN_SET

    #define IOBUS_DIR_PORT GPIOB
    #define IOBUS_DIR_PIN  GPIO_PIN_6    /* high = write, low = read */
	#define DIR_READ	GPIO_PIN_RESET
	#define DIR_WRITE	GPIO_PIN_SET

    #define DIG_OUTPUT_ENABLE_PORT GPIOE
    #define DIG_OUTPUT_ENABLE_PIN  GPIO_PIN_12


    static inline void iobus_set_addr(uint8_t addr)
    {
    	//Setting the address bus to a DIGOUT_BANKx causes the output bank to strobe the data values from the
    	//data line into the output bank's register.
    	//For dig outputs, set the data bits first, then set the address bits.  Finally, set the address bits to an
    	//input bank so as to not give any output bank another clock pulse.  Addr 0x07 (DigIn_Bank4) is a safe idle address.

        HAL_GPIO_WritePin(IOBUS_ADDR_PORT, IOBUS_ADDR0_PIN, (addr & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IOBUS_ADDR_PORT, IOBUS_ADDR1_PIN, (addr & 2) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IOBUS_ADDR_PORT, IOBUS_ADDR2_PIN, (addr & 4) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    }

    static inline void iobus_set_addr_idle(){
    	//return to idle address, make sure ADDR2 is first, then the other two (ADDR0 & ADDR1)
    	HAL_GPIO_WritePin(IOBUS_ADDR_PORT, IOBUS_ADDR2_PIN, GPIO_PIN_SET);		// 1
    	HAL_GPIO_WritePin(IOBUS_ADDR_PORT, IOBUS_ADDR0_PIN, GPIO_PIN_SET);		// 1
		HAL_GPIO_WritePin(IOBUS_ADDR_PORT, IOBUS_ADDR1_PIN, GPIO_PIN_SET);		// 1
    }

    static void iobus_write(uint8_t addr, uint8_t data)
    {
    	//For writing to output banks:
    	// 1. Set the address to the designated idle address.
    	// 2. Disable the data bus
    	// 3. Set the data bits
    	// 4. Enable the data bus
    	// 5. Set the address pins to the output bank's address
    	// 6. Set the address pins to the designated idle address.

    	// 1: Set the address to the designated idle address.
    	iobus_set_addr_idle();
    	__NOP(); __NOP(); __NOP();

    	// 2: Disable the data bus
    	//HAL_GPIO_WritePin(IOBUS_WE_PORT, IOBUS_WE_PIN, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(IOBUS_OE_PORT, IOBUS_OE_PIN, OE_DISABLE);
    	__NOP(); __NOP(); __NOP();

    	// 3. Set the data bits
        GPIO_PinState bits[8];
        for (int i = 0; i < 8; ++i) bits[i] = (data & (1 << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        //HAL_GPIO_WritePin(IOBUS_DATA_PORT, IOBUS_DATA_PINS,
        //    bits[0] | (bits[1] << 1) | (bits[2] << 2) | (bits[3] << 3) |
        //    (bits[4] << 4) | (bits[5] << 5) | (bits[6] << 6) | (bits[7] << 7));
        HAL_GPIO_WritePin(IOBUS_DATA_PORT, GPIO_PIN_7, bits[0]);
        HAL_GPIO_WritePin(IOBUS_DATA_PORT, GPIO_PIN_6, bits[1]);
        HAL_GPIO_WritePin(IOBUS_DATA_PORT, GPIO_PIN_5, bits[2]);
        HAL_GPIO_WritePin(IOBUS_DATA_PORT, GPIO_PIN_4, bits[3]);
        HAL_GPIO_WritePin(IOBUS_DATA_PORT, GPIO_PIN_3, bits[4]);
        HAL_GPIO_WritePin(IOBUS_DATA_PORT, GPIO_PIN_2, bits[5]);
        HAL_GPIO_WritePin(IOBUS_DATA_PORT, GPIO_PIN_1, bits[6]);
        HAL_GPIO_WritePin(IOBUS_DATA_PORT, GPIO_PIN_0, bits[7]);


        // 4. Set the databus to Write and Enable the data bus
        HAL_GPIO_WritePin(IOBUS_DIR_PORT, IOBUS_DIR_PIN, DIR_WRITE);
        __NOP(); __NOP(); __NOP();
        HAL_GPIO_WritePin(IOBUS_OE_PORT, IOBUS_OE_PIN, OE_ENABLE);

        // 5. Set the address pins to the output bank's address
        iobus_set_addr(addr);
        __NOP(); __NOP(); __NOP();

        // 6. Set the address pins to the designated idle address.
        iobus_set_addr_idle();

        // 7: Disable the data bus and set it back to Read mode
        HAL_GPIO_WritePin(IOBUS_OE_PORT, IOBUS_OE_PIN, OE_DISABLE);
		HAL_GPIO_WritePin(IOBUS_DIR_PORT, IOBUS_DIR_PIN, DIR_READ);
    }

    static uint8_t iobus_read(uint8_t addr)
    {
    	if(addr < 0x03){
    		//Inputs are 0x03 to 0x07.
    		return 0;
    	}

    	HAL_GPIO_WritePin(IOBUS_DIR_PORT, IOBUS_DIR_PIN, DIR_READ);
        iobus_set_addr(addr);
        HAL_GPIO_WritePin(IOBUS_OE_PORT, IOBUS_OE_PIN, OE_ENABLE);
        __NOP(); __NOP(); __NOP();
        //uint8_t data = (uint8_t)(HAL_GPIO_ReadPin(IOBUS_DATA_PORT, IOBUS_DATA_PINS) & 0xFF);
        uint8_t data = 0;
        data |= ( (HAL_GPIO_ReadPin(IOBUS_DATA_PORT, GPIO_PIN_7) == GPIO_PIN_SET ? 1 : 0) << 0);
        data |= ( (HAL_GPIO_ReadPin(IOBUS_DATA_PORT, GPIO_PIN_6) == GPIO_PIN_SET ? 1 : 0) << 1);
        data |= ( (HAL_GPIO_ReadPin(IOBUS_DATA_PORT, GPIO_PIN_5) == GPIO_PIN_SET ? 1 : 0) << 2);
        data |= ( (HAL_GPIO_ReadPin(IOBUS_DATA_PORT, GPIO_PIN_4) == GPIO_PIN_SET ? 1 : 0) << 3);
        data |= ( (HAL_GPIO_ReadPin(IOBUS_DATA_PORT, GPIO_PIN_3) == GPIO_PIN_SET ? 1 : 0) << 4);
        data |= ( (HAL_GPIO_ReadPin(IOBUS_DATA_PORT, GPIO_PIN_2) == GPIO_PIN_SET ? 1 : 0) << 5);
        data |= ( (HAL_GPIO_ReadPin(IOBUS_DATA_PORT, GPIO_PIN_1) == GPIO_PIN_SET ? 1 : 0) << 6);
        data |= ( (HAL_GPIO_ReadPin(IOBUS_DATA_PORT, GPIO_PIN_0) == GPIO_PIN_SET ? 1 : 0) << 7);

        iobus_set_addr_idle();

        HAL_GPIO_WritePin(IOBUS_OE_PORT, IOBUS_OE_PIN, OE_DISABLE);
        return data;
    }
#endif

/* ---------------------------------------------------------------------- */
/* Init                                                                   */
/* ---------------------------------------------------------------------- */
void dio_init(board_type_t type)
{
    board = type;

#if defined(BOARD_BABY)
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Outputs */
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    for (int i = 0; i < 2; ++i) {
        GPIO_InitStruct.Pin = baby_out_pins[i];
        HAL_GPIO_Init(baby_out_port[i], &GPIO_InitStruct);
        HAL_GPIO_WritePin(baby_out_port[i], baby_out_pins[i], GPIO_PIN_RESET);
    }

    /* Inputs */
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    for (int i = 0; i < 4; ++i) {
        GPIO_InitStruct.Pin = baby_in_pins[i];
        HAL_GPIO_Init(baby_in_port[i], &GPIO_InitStruct);
    }

#elif defined(BOARD_MAMA) || defined(BOARD_PAPA)
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Data bus */
    GPIO_InitStruct.Pin   = IOBUS_DATA_PINS;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(IOBUS_DATA_PORT, &GPIO_InitStruct);

    /* Address */
    GPIO_InitStruct.Pin = IOBUS_ADDR0_PIN | IOBUS_ADDR1_PIN | IOBUS_ADDR2_PIN;
    HAL_GPIO_Init(IOBUS_ADDR_PORT, &GPIO_InitStruct);

    /* Control */
    GPIO_InitStruct.Pin = IOBUS_OE_PIN;
    HAL_GPIO_Init(IOBUS_OE_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = IOBUS_DIR_PIN;
    HAL_GPIO_Init(IOBUS_DIR_PORT, &GPIO_InitStruct);

    /* Global enable */
    GPIO_InitStruct.Pin = DIG_OUTPUT_ENABLE_PIN;
    HAL_GPIO_Init(DIG_OUTPUT_ENABLE_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(DIG_OUTPUT_ENABLE_PORT, DIG_OUTPUT_ENABLE_PIN, GPIO_PIN_SET); /* disabled */

    /* Idle state */
    HAL_GPIO_WritePin(IOBUS_OE_PORT, IOBUS_OE_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IOBUS_DIR_PORT, IOBUS_DIR_PIN, GPIO_PIN_SET);
#endif
}

/* ---------------------------------------------------------------------- */
/* Digital Outputs                                                        */
/* ---------------------------------------------------------------------- */
void dio_output_set(uint8_t idx, bool on)
{
#if defined(BOARD_BABY)
    if (board == BOARD_TYPE_BABY) {
        if (idx >= 2) return;
        HAL_GPIO_WritePin(baby_out_port[idx], baby_out_pins[idx],
                          on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
#endif

#if defined(BOARD_MAMA) || defined(BOARD_PAPA)
    uint8_t max = (board == BOARD_TYPE_MAMA) ? 8 : 16;
    if (idx >= max) return;

    uint8_t addr = idx / 8;
    uint8_t bit  = idx % 8;
    //uint8_t data = outputBankCurVal[addr];
    //if (on) data |=  (1 << bit);
    //else    data &= ~(1 << bit);
    if(on){
    	outputBankCurVal[addr] |=  (1 << bit);
    }else{
    	outputBankCurVal[addr] &= ~(1 << bit);
    }

    iobus_write(addr, outputBankCurVal[addr]);
#endif

//    web_socket_broadcast_status();  //Update websocket clients that an output changed state
}

void dio_all_outputs_off(void){
	uint8_t numOuts = dio_get_output_count();
	for(uint8_t n=0; n<numOuts; n++){
		dio_output_set(n, false);
	}
}

bool dio_output_get(uint8_t idx)
{
#if defined(BOARD_BABY)
    if (board == BOARD_TYPE_BABY) {
        if (idx >= 2) return false;
        return HAL_GPIO_ReadPin(baby_out_port[idx], baby_out_pins[idx]) == GPIO_PIN_SET;
    }
#endif

#if defined(BOARD_MAMA) || defined(BOARD_PAPA)
    uint8_t max = (board == BOARD_TYPE_MAMA) ? 8 : 16;
    if (idx >= max) return false;

    uint8_t addr = idx / 8;
    uint8_t bit  = idx % 8;
    return (outputBankCurVal[addr] & (1 << bit)) != 0;
#else
    return false;
#endif
}

/* ---------------------------------------------------------------------- */
/* Global Output Enable                                                   */
/* ---------------------------------------------------------------------- */
void dio_output_enable(bool enable)
{
#if defined(BOARD_MAMA) || defined(BOARD_PAPA)
    HAL_GPIO_WritePin(DIG_OUTPUT_ENABLE_PORT, DIG_OUTPUT_ENABLE_PIN,
                      enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
#else
    (void)enable;
#endif
}

/* ---------------------------------------------------------------------- */
/* Digital Inputs                                                         */
/* ---------------------------------------------------------------------- */
bool dio_input_get(uint8_t idx)
{
#if defined(BOARD_BABY)
    if (board == BOARD_TYPE_BABY) {
        if (idx >= 4) return false;
        return HAL_GPIO_ReadPin(baby_in_port[idx], baby_in_pins[idx]) == GPIO_PIN_SET;
    }
#endif

#if defined(BOARD_MAMA) || defined(BOARD_PAPA)
    uint8_t max = (board == BOARD_TYPE_MAMA) ? 12 : 32;
    if (idx >= max) return false;

    uint8_t addr;
    uint8_t bit = idx % 8;

    if (board == BOARD_TYPE_MAMA) {
        if (idx <= 7) {
            addr = 0x03;
        } else {
            addr = 0x04;
            bit = idx - 8;
            if (bit >= 4) return false;
        }
    } else {
        addr = 0x03 + (idx / 8);
    }

    return (iobus_read(addr) & (1 << bit)) != 0;
#else
    return false;
#endif
}



/* New: Get input count based on board */
uint8_t dio_get_input_count(void) {
    switch (board) {
        case BOARD_TYPE_BABY: return DIO_INPUT_COUNT_BABY;
        case BOARD_TYPE_MAMA: return DIO_INPUT_COUNT_MAMA;
        case BOARD_TYPE_PAPA: return DIO_INPUT_COUNT_PAPA;
        default: return 0;
    }
}

/* New: Get output count based on board */
uint8_t dio_get_output_count(void) {
    switch (board) {
        case BOARD_TYPE_BABY: return DIO_OUTPUT_COUNT_BABY;
        case BOARD_TYPE_MAMA: return DIO_OUTPUT_COUNT_MAMA;
        case BOARD_TYPE_PAPA: return DIO_OUTPUT_COUNT_PAPA;
        default: return 0;
    }
}
