/* Core/Src/aio.c */
#include "aio.h"
#include "stm32f4xx_hal.h"

static board_type_t board;
uint16_t curAInVals[AIO_INPUT_COUNT_MAX] = {0};
uint16_t curAOutVals[AIO_OUTPUT_COUNT_MAX] = {0};

uint8_t curInCount = 0;
uint8_t curOutCount = 0;

void aio_init(board_type_t type){
	board = type;
	curInCount = aio_get_input_count();
	curOutCount = aio_get_output_count();
}


void  aio_output_set(uint8_t idx, uint16_t val){
	if(idx < curOutCount){
		curAOutVals[idx] = val;
	}
}

void  aio_all_outputs_off(void){
	for(uint8_t n=0; n< curOutCount; n++){
		curAOutVals[n] = 0;
	}
}

uint16_t  aio_output_get(uint8_t idx){
	if(idx < curOutCount){
		return curAOutVals[idx];
	}else{
		return 0;
	}
}



uint16_t aio_input_get(uint8_t idx){
	if(idx < curInCount){
		return curAInVals[idx];
	}else{
		return 0;
	}
}


uint8_t aio_get_input_count(void) {
    switch (board) {
        case BOARD_TYPE_BABY: return AIO_INPUT_COUNT_BABY;
        case BOARD_TYPE_MAMA: return AIO_INPUT_COUNT_MAMA;
        default: return 0;
    }
}

/* New: Get output count based on board */
uint8_t aio_get_output_count(void) {
    switch (board) {
        case BOARD_TYPE_BABY: return AIO_OUTPUT_COUNT_BABY;
        case BOARD_TYPE_MAMA: return AIO_OUTPUT_COUNT_MAMA;
        default: return 0;
    }
}
