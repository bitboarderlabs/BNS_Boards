#include "ST7735Canvas.h"
#include <string.h>

#ifndef pgm_read_byte
#define pgm_read_byte(addr) (*(const uint8_t *)(addr))
#endif
#ifndef pgm_read_word
#define pgm_read_word(addr) (*(const uint16_t *)(addr))
#endif
#ifndef pgm_read_dword
#define pgm_read_dword(addr) (*(const uint32_t *)(addr))
#endif



#ifndef PROGMEM
#define PROGMEM
#endif


#define _swap_int16_t(a, b)                                                    \
  {                                                                            \
    int16_t t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }

#define min(a, b) (((a) < (b)) ? (a) : (b))


const uint8_t
	init_cmds1[] = {					// Init for 7735R, part 1 (red or green tab)
		14,								// 15 commands in list:
		ST7735_SWRESET, ST_CMD_DELAY,	//  1: Software reset, 0 args, w/delay
		150,							//     150 ms delay
		ST7735_SLPOUT, ST_CMD_DELAY,	//  2: Out of sleep mode, 0 args, w/delay
		255,							//     500 ms delay
		ST7735_FRMCTR1, 3,				//  3: Frame rate ctrl - normal mode, 3 args:
			0x01, 0x2C, 0x2D,			//     Rate = fosc/(1x2+40) * (LINE+2C+2D)
		ST7735_FRMCTR2, 3,				//  4: Frame rate control - idle mode, 3 args:
			0x01, 0x2C, 0x2D,			//     Rate = fosc/(1x2+40) * (LINE+2C+2D)
		ST7735_FRMCTR3, 6,				//  5: Frame rate ctrl - partial mode, 6 args:
			0x01, 0x2C, 0x2D,			//     Dot inversion mode
			0x01, 0x2C, 0x2D,			//     Line inversion mode
		ST7735_INVCTR , 1,				//  6: Display inversion ctrl, 1 arg, no delay:
			0x07,						//     No inversion
		ST7735_PWCTR1 , 3,				//  7: Power control, 3 args, no delay:
			0xA2,
			0x02,						//     -4.6V
			0x84,						//     AUTO mode
		ST7735_PWCTR2 , 1,				//  8: Power control, 1 arg, no delay:
			0xC5,						//     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
		ST7735_PWCTR3 , 2,				//  9: Power control, 2 args, no delay:
			0x0A,						//     Opamp current small
			0x00,						//     Boost frequency
		ST7735_PWCTR4 , 2,				// 10: Power control, 2 args, no delay:
			0x8A,						//     BCLK/2, Opamp current small & Medium low
			0x2A,
		ST7735_PWCTR5 , 2,				// 11: Power control, 2 args, no delay:
			0x8A, 0xEE,
		ST7735_VMCTR1 , 1,				// 12: Power control, 1 arg, no delay:
			0x0E,
		ST7735_INVOFF , 0,				// 13: Don't invert display, no args, no delay
		ST7735_COLMOD , 1,				// 15: set color mode, 1 arg, no delay:
			0x05 },						//     16-bit color



//#if (defined(ST7735_IS_128X128) || defined(ST7735_IS_160X128))
	init_cmds2a[] = {					// Init for 7735R, part 2 (1.44" display)
		2,								//  2 commands in list:
		ST7735_CASET, 4,				//  1: Column addr set, 4 args, no delay:
			0x00, 0x00,					//     XSTART = 0
			0x00, 0x7F,					//     XEND = 127
		ST7735_RASET  , 4,				//  2: Row addr set, 4 args, no delay:
			0x00, 0x00,					//     XSTART = 0
			0x00, 0x7F },				//     XEND = 127
//#endif // ST7735_IS_128X128

//#ifdef ST7735_IS_160X80
	init_cmds2b[] = {					// Init for 7735S, part 2 (160x80 display)
		3,								//  3 commands in list:
		ST7735_CASET, 4,				//  1: Column addr set, 4 args, no delay:
			0x00, 0x00,					//     XSTART = 0
			0x00, 0x4F,					//     XEND = 79
		ST7735_RASET, 4,				//  2: Row addr set, 4 args, no delay:
			0x00, 0x00,					//     XSTART = 0
			0x00, 0x9F ,				//     XEND = 159
		ST7735_INVON, 0 },				//  3: Invert colors
//#endif

	init_cmds3[] = {					// Init for 7735R, part 3 (red or green tab)
		4,								//  4 commands in list:
		ST7735_GMCTRP1, 16,				//  1: Magical unicorn dust, 16 args, no delay:
			0x02, 0x1c, 0x07, 0x12,
			0x37, 0x32, 0x29, 0x2d,
			0x29, 0x25, 0x2B, 0x39,
			0x00, 0x01, 0x03, 0x10,
		ST7735_GMCTRN1, 16,				//  2: Sparkles and rainbows, 16 args, no delay:
			0x03, 0x1d, 0x07, 0x06,
			0x2E, 0x2C, 0x29, 0x2D,
			0x2E, 0x2E, 0x37, 0x3F,
			0x00, 0x00, 0x02, 0x10,
		ST7735_NORON, ST_CMD_DELAY,		//  3: Normal display on, no args, w/delay
			10,							//     10 ms delay
		ST7735_DISPON, ST_CMD_DELAY,	//  4: Main screen turn on, no args w/delay
			100 };						//     100 ms delay




static const uint8_t
  Bcmd[] = {                        // Init commands for 7735B screens
    18,                             // 18 commands in list:
    ST7735_SWRESET,   ST_CMD_DELAY, //  1: Software reset, no args, w/delay
      150,                           //     50 ms delay
    ST7735_SLPOUT,    ST_CMD_DELAY, //  2: Out of sleep mode, no args, w/delay
      255,                          //     255 = max (500 ms) delay
    ST7735_COLMOD,  1+ST_CMD_DELAY, //  3: Set color mode, 1 arg + delay:
      0x05,                         //     16-bit color
      10,                           //     10 ms delay
    ST7735_FRMCTR1, 3+ST_CMD_DELAY, //  4: Frame rate control, 3 args + delay:
      0x00,                         //     fastest refresh
      0x06,                         //     6 lines front porch
      0x03,                         //     3 lines back porch
      10,                           //     10 ms delay
    ST7735_MADCTL,  1,              //  5: Mem access ctl (directions), 1 arg:
      0x08,                         //     Row/col addr, bottom-top refresh
    ST7735_DISSET5, 2,              //  6: Display settings #5, 2 args:
      0x15,                         //     1 clk cycle nonoverlap, 2 cycle gate
                                    //     rise, 3 cycle osc equalize
      0x02,                         //     Fix on VTL
    ST7735_INVCTR,  1,              //  7: Display inversion control, 1 arg:
      0x0,                          //     Line inversion
    ST7735_PWCTR1,  2+ST_CMD_DELAY, //  8: Power control, 2 args + delay:
      0x02,                         //     GVDD = 4.7V
      0x70,                         //     1.0uA
      10,                           //     10 ms delay
    ST7735_PWCTR2,  1,              //  9: Power control, 1 arg, no delay:
      0x05,                         //     VGH = 14.7V, VGL = -7.35V
    ST7735_PWCTR3,  2,              // 10: Power control, 2 args, no delay:
      0x01,                         //     Opamp current small
      0x02,                         //     Boost frequency
    ST7735_VMCTR1,  2+ST_CMD_DELAY, // 11: Power control, 2 args + delay:
      0x3C,                         //     VCOMH = 4V
      0x38,                         //     VCOML = -1.1V
      10,                           //     10 ms delay
    ST7735_PWCTR6,  2,              // 12: Power control, 2 args, no delay:
      0x11, 0x15,
    ST7735_GMCTRP1,16,              // 13: Gamma Adjustments (pos. polarity), 16 args + delay:
      0x09, 0x16, 0x09, 0x20,       //     (Not entirely necessary, but provides
      0x21, 0x1B, 0x13, 0x19,       //      accurate colors)
      0x17, 0x15, 0x1E, 0x2B,
      0x04, 0x05, 0x02, 0x0E,
    ST7735_GMCTRN1,16+ST_CMD_DELAY, // 14: Gamma Adjustments (neg. polarity), 16 args + delay:
      0x0B, 0x14, 0x08, 0x1E,       //     (Not entirely necessary, but provides
      0x22, 0x1D, 0x18, 0x1E,       //      accurate colors)
      0x1B, 0x1A, 0x24, 0x2B,
      0x06, 0x06, 0x02, 0x0F,
      10,                           //     10 ms delay
    ST7735_CASET,   4,              // 15: Column addr set, 4 args, no delay:
      0x00, 0x02,                   //     XSTART = 2
      0x00, 0x81,                   //     XEND = 129
    ST7735_RASET,   4,              // 16: Row addr set, 4 args, no delay:
      0x00, 0x02,                   //     XSTART = 1
      0x00, 0x81,                   //     XEND = 160
    ST7735_NORON,     ST_CMD_DELAY, // 17: Normal display on, no args, w/delay
      10,                           //     10 ms delay
    ST7735_DISPON,    ST_CMD_DELAY, // 18: Main screen turn on, no args, delay
      255 },                        //     255 = max (500 ms) delay

  Rcmd1[] = {                       // 7735R init, part 1 (red or green tab)
    15,                             // 15 commands in list:
    ST7735_SWRESET,   ST_CMD_DELAY, //  1: Software reset, 0 args, w/delay
      150,                          //     150 ms delay
    ST7735_SLPOUT,    ST_CMD_DELAY, //  2: Out of sleep mode, 0 args, w/delay
      255,                          //     500 ms delay
    ST7735_FRMCTR1, 3,              //  3: Framerate ctrl - normal mode, 3 arg:
      0x01, 0x2C, 0x2D,             //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3,              //  4: Framerate ctrl - idle mode, 3 args:
      0x01, 0x2C, 0x2D,             //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6,              //  5: Framerate - partial mode, 6 args:
      0x01, 0x2C, 0x2D,             //     Dot inversion mode
      0x01, 0x2C, 0x2D,             //     Line inversion mode
    ST7735_INVCTR,  1,              //  6: Display inversion ctrl, 1 arg:
      0x07,                         //     No inversion
    ST7735_PWCTR1,  3,              //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                         //     -4.6V
      0x84,                         //     AUTO mode
    ST7735_PWCTR2,  1,              //  8: Power control, 1 arg, no delay:
      0xC5,                         //     VGH25=2.4C VGSEL=-10 VGH=3 * AVDD
    ST7735_PWCTR3,  2,              //  9: Power control, 2 args, no delay:
      0x0A,                         //     Opamp current small
      0x00,                         //     Boost frequency
    ST7735_PWCTR4,  2,              // 10: Power control, 2 args, no delay:
      0x8A,                         //     BCLK/2,
      0x2A,                         //     opamp current small & medium low
    ST7735_PWCTR5,  2,              // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1,  1,              // 12: Power control, 1 arg, no delay:
      0x0E,
    ST7735_INVOFF,  0,              // 13: Don't invert display, no args
    ST7735_MADCTL,  1,              // 14: Mem access ctl (directions), 1 arg:
      0xC8,                         //     row/col addr, bottom-top refresh
    ST7735_COLMOD,  1,              // 15: set color mode, 1 arg, no delay:
      0x05 },                       //     16-bit color


  Rcmd2green[] = {                  // 7735R init, part 2 (green tab only)
    2,                              //  2 commands in list:
    ST7735_CASET,   4,              //  1: Column addr set, 4 args, no delay:
      0x00, 0x02,                   //     XSTART = 0
      0x00, 0x7F+0x02,              //     XEND = 127
    ST7735_RASET,   4,              //  2: Row addr set, 4 args, no delay:
      0x00, 0x01,                   //     XSTART = 0
      0x00, 0x9F+0x01 },            //     XEND = 159

  Rcmd2red[] = {                    // 7735R init, part 2 (red tab only)
    2,                              //  2 commands in list:
    ST7735_CASET,   4,              //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,                   //     XSTART = 0
      0x00, 0x7F,                   //     XEND = 127
    ST7735_RASET,   4,              //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,                   //     XSTART = 0
      0x00, 0x9F },                 //     XEND = 159

  Rcmd2green144[] = {               // 7735R init, part 2 (green 1.44 tab)
    2,                              //  2 commands in list:
    ST7735_CASET,   4,              //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,                   //     XSTART = 0
      0x00, 0x7F,                   //     XEND = 127
    ST7735_RASET,   4,              //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,                   //     XSTART = 0
      0x00, 0x7F },                 //     XEND = 127

  Rcmd2green160x80[] = {            // 7735R init, part 2 (mini 160x80)
    2,                              //  2 commands in list:
    ST7735_CASET,   4,              //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,                   //     XSTART = 0
      0x00, 0x4F,                   //     XEND = 79
    ST7735_RASET,   4,              //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,                   //     XSTART = 0
      0x00, 0x9F },                 //     XEND = 159

  Rcmd2green160x80plugin[] = {      // 7735R init, part 2 (mini 160x80 with plugin FPC)
    3,                              //  3 commands in list:
    ST7735_INVON,  0,              //   1: Display is inverted
    ST7735_CASET,   4,              //  2: Column addr set, 4 args, no delay:
      0x00, 0x00,                   //     XSTART = 0
      0x00, 0x4F,                   //     XEND = 79
    ST7735_RASET,   4,              //  3: Row addr set, 4 args, no delay:
      0x00, 0x00,                   //     XSTART = 0
      0x00, 0x9F },                 //     XEND = 159

  Rcmd3[] = {                       // 7735R init, part 3 (red or green tab)
    4,                              //  4 commands in list:
    ST7735_GMCTRP1, 16      ,       //  1: Gamma Adjustments (pos. polarity), 16 args + delay:
      0x02, 0x1c, 0x07, 0x12,       //     (Not entirely necessary, but provides
      0x37, 0x32, 0x29, 0x2d,       //      accurate colors)
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      ,       //  2: Gamma Adjustments (neg. polarity), 16 args + delay:
      0x03, 0x1d, 0x07, 0x06,       //     (Not entirely necessary, but provides
      0x2E, 0x2C, 0x29, 0x2D,       //      accurate colors)
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST7735_NORON,     ST_CMD_DELAY, //  3: Normal display on, no args, w/delay
      10,                           //     10 ms delay
    ST7735_DISPON,    ST_CMD_DELAY, //  4: Main screen turn on, no args w/delay
      100 };                        //     100 ms delay




void ST7735_Init(LCD_HandleTypeDef* dev, uint16_t w, uint16_t h, SPI_HandleTypeDef* spi, GPIO_TypeDef* csPort, uint16_t csPin,
  			GPIO_TypeDef* dcPort, uint16_t dcPin, GPIO_TypeDef* rstPort, uint16_t rstPin, uint8_t rotation,
  			uint8_t tabcolor, bool finishInit){

  	//From "init" section:
  	dev->hspi = spi;
  	dev->csPort = csPort;
  	dev->csPin = csPin;
  	dev->dcPort = dcPort;
  	dev->dcPin = dcPin;
  	dev->rstPort = rstPort;
  	dev->rstPin = rstPin;

  	dev->rotation = rotation;
  	dev->tabColor = tabcolor;

  	dev->textColor = BLACK;
  	dev->textBgColor = BLACK;

  	dev->dmaWriteActive = 0;

  	if(finishInit){
  		ST7735_Start(dev, 500);
  	}else{
  		//Before the display is ready for use, the the calling program will need
  		//to delay for several ms, then call ST7735_Start().
  	}
}

void ST7735_Start(LCD_HandleTypeDef* dev, uint32_t preDelay){
  	HAL_Delay(preDelay);

  	ST7735_Select(dev);
  	if(dev->tabColor == INITB_NOOPTIONS){
  		ST7735_InitB(dev);
  	}else{
  		ST7735_InitR(dev);
  	}
  	ST7735_Unselect(dev);
}

void ST7735_InitB(LCD_HandleTypeDef* dev){
	ST7735_DisplayInit(dev, Bcmd);
	ST7735_SetRotation(dev, 0);
}

void ST7735_InitR(LCD_HandleTypeDef* dev){
	ST7735_DisplayInit(dev, Rcmd1);
	if (dev->tabColor == INITR_GREENTAB) {
		ST7735_DisplayInit(dev, Rcmd2green);
		dev->colstart = 2;
		dev->rowstart = 1;
	} else if ((dev->tabColor == INITR_144GREENTAB) || (dev->tabColor == INITR_HALLOWING)) {
		dev->height = ST7735_TFTHEIGHT_128;
		dev->width = ST7735_TFTWIDTH_128;
		ST7735_DisplayInit(dev, Rcmd2green144);
		dev->colstart = 2;
		dev->rowstart = 3; // For default rotation 0
	} else if (dev->tabColor == INITR_MINI160x80) {
		dev->height = ST7735_TFTWIDTH_80;
		dev->width = ST7735_TFTHEIGHT_160;
		ST7735_DisplayInit(dev, Rcmd2green160x80);
		dev->colstart = 24;
		dev->rowstart = 0;
	} else if (dev->tabColor == INITR_MINI160x80_PLUGIN) {
		dev->height = ST7735_TFTWIDTH_80;
		dev->width = ST7735_TFTHEIGHT_160;
		ST7735_DisplayInit(dev, Rcmd2green160x80plugin);
		dev->colstart = 26;
		dev->rowstart = 1;
		dev->invertOnCmd = ST7735_INVOFF;
		dev->invertOffCmd = ST7735_INVON;
	} else {
		// colstart, rowstart left at default '0' values
		ST7735_DisplayInit(dev, Rcmd2red);
	}
	ST7735_DisplayInit(dev, Rcmd3);

	// Black tab, change MADCTL color filter
	if ((dev->tabColor == INITR_BLACKTAB) || (dev->tabColor == INITR_MINI160x80)){// || (dev->tabColor == INITR_MINI160x80_PLUGIN)) {
		uint8_t data = 0xC0;
		ST7735_WriteCommand(dev, ST7735_MADCTL);
		ST7735_WriteData(dev, &data, 1);
	}

	if (dev->tabColor == INITR_HALLOWING) {
		// Hallowing is simply a 1.44" green tab upside-down:
		dev->tabColor = INITR_144GREENTAB;
		ST7735_SetRotation(dev, 2);
	} else {
		//_tabColor = options;
		ST7735_SetRotation(dev, 0);
	}

}

void ST7735_DisplayInit(LCD_HandleTypeDef* dev, const uint8_t *addr){
    uint8_t numCommands, numArgs;
	uint16_t ms;

	numCommands = *addr++;
	while(numCommands--) {
		uint8_t cmd = *addr++;
		ST7735_WriteCommand(dev, cmd);

		numArgs = *addr++;
		// If high bit set, delay follows args
		ms = numArgs & ST_CMD_DELAY;
		numArgs &= ~ST_CMD_DELAY;
		if(numArgs) {
			ST7735_WriteData(dev, (uint8_t*)addr, numArgs);
			addr += numArgs;
		}

		if(ms) {
			ms = *addr++;
			if(ms == 255) ms = 500;
			HAL_Delay(ms);
		}
	}
}

void ST7735_DisplayOn(LCD_HandleTypeDef* dev){
	ST7735_Select(dev);
	ST7735_WriteCommand(dev, ST7735_DISPON);
	ST7735_Unselect(dev);
}

void ST7735_DisplayOff(LCD_HandleTypeDef* dev){
	ST7735_Select(dev);
	ST7735_WriteCommand(dev, ST7735_DISPOFF);
	ST7735_Unselect(dev);
}

void ST7735_Select(LCD_HandleTypeDef* dev){
	HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_RESET);
}

void ST7735_Unselect(LCD_HandleTypeDef* dev){
	HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_SET);
}

void ST7735_Reset(LCD_HandleTypeDef* dev){
	HAL_GPIO_WritePin(dev->rstPort, dev->rstPin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(dev->rstPort, dev->rstPin, GPIO_PIN_SET);
	HAL_Delay(7);
}

void ST7735_WriteCommand(LCD_HandleTypeDef* dev, uint8_t cmd){
	//while(dev->dmaWriteActive != 0);
	HAL_GPIO_WritePin(dev->dcPort, dev->dcPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(dev->hspi, &cmd, sizeof(cmd), HAL_MAX_DELAY);
}

void ST7735_WriteData(LCD_HandleTypeDef* dev, uint8_t* buff, size_t buff_size){
	//while(dev->dmaWriteActive != 0);
	HAL_GPIO_WritePin(dev->dcPort, dev->dcPin, GPIO_PIN_SET);
	HAL_SPI_Transmit(dev->hspi, buff, buff_size, HAL_MAX_DELAY);
}

void ST7735_WriteDataDMA(LCD_HandleTypeDef* dev, uint8_t* buff, size_t buff_size){
	//if(dev->dmaWriteActive == 0){
		dev->dmaWriteActive = 1;
		HAL_GPIO_WritePin(dev->dcPort, dev->dcPin, GPIO_PIN_SET);
		HAL_SPI_Transmit_DMA(dev->hspi, buff, buff_size);
	//}
}



void ST7735_SetAddressWindow(LCD_HandleTypeDef* dev, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    // column address set
    ST7735_WriteCommand(dev, ST7735_CASET);
    uint8_t data[] = { 0x00, x0 + dev->xstart, 0x00, x1 + dev->xstart };
    ST7735_WriteData(dev, data, sizeof(data));

    // row address set
    ST7735_WriteCommand(dev, ST7735_RASET);
    data[1] = y0 + dev->ystart;
    data[3] = y1 + dev->ystart;
    ST7735_WriteData(dev, data, sizeof(data));

    // write to RAM
    ST7735_WriteCommand(dev, ST7735_RAMWR);
}



void ST7735_SetRotation(LCD_HandleTypeDef* dev, uint8_t m){
	uint8_t madctl = 0;
	dev->rotation = m % 4; // can't be higher than 3

	switch (dev->rotation){
	case 0:
//======
		if ((dev->tabColor == INITR_BLACKTAB) || (dev->tabColor == INITR_MINI160x80)) {
			madctl = ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_RGB;
		} else {
			madctl = ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_BGR;
		}

		if (dev->tabColor == INITR_144GREENTAB) {
			dev->height = ST7735_TFTHEIGHT_128;
			dev->width = ST7735_TFTWIDTH_128;
		} else if (dev->tabColor == INITR_MINI160x80 ||
				dev->tabColor == INITR_MINI160x80_PLUGIN) {
			dev->height = ST7735_TFTHEIGHT_160;
			dev->width = ST7735_TFTWIDTH_80;
		} else {
			dev->height = ST7735_TFTHEIGHT_160;
			dev->width = ST7735_TFTWIDTH_128;
		}
		dev->xstart = dev->colstart;
		dev->ystart = dev->rowstart;
		break;
	case 1:
		if ((dev->tabColor == INITR_BLACKTAB) || (dev->tabColor == INITR_MINI160x80)) {
			madctl = ST7735_MADCTL_MY | ST7735_MADCTL_MV | ST7735_MADCTL_RGB;
		} else {
			madctl = ST7735_MADCTL_MY | ST7735_MADCTL_MV | ST7735_MADCTL_BGR;
		}

		if (dev->tabColor == INITR_144GREENTAB) {
			dev->width = ST7735_TFTHEIGHT_128;
			dev->height = ST7735_TFTWIDTH_128;
		} else if (dev->tabColor == INITR_MINI160x80 ||
				dev->tabColor == INITR_MINI160x80_PLUGIN) {
			dev->width = ST7735_TFTHEIGHT_160;
			dev->height = ST7735_TFTWIDTH_80;
		} else {
			dev->width = ST7735_TFTHEIGHT_160;
			dev->height = ST7735_TFTWIDTH_128;
		}
		dev->ystart = dev->colstart;
		dev->xstart = dev->rowstart;
		break;
	case 2:
		if ((dev->tabColor == INITR_BLACKTAB) || (dev->tabColor == INITR_MINI160x80)) {
			madctl = ST7735_MADCTL_RGB;
		} else {
			madctl = ST7735_MADCTL_BGR;
		}

		if (dev->tabColor == INITR_144GREENTAB) {
			dev->height = ST7735_TFTHEIGHT_128;
			dev->width = ST7735_TFTWIDTH_128;
		} else if (dev->tabColor == INITR_MINI160x80 ||
				dev->tabColor == INITR_MINI160x80_PLUGIN) {
			dev->height = ST7735_TFTHEIGHT_160;
			dev->width = ST7735_TFTWIDTH_80;
		} else {
			dev->height = ST7735_TFTHEIGHT_160;
			dev->width = ST7735_TFTWIDTH_128;
		}
		dev->xstart = dev->colstart;
		dev->ystart = dev->rowstart;
		break;
	case 3:
		if ((dev->tabColor == INITR_BLACKTAB) || (dev->tabColor == INITR_MINI160x80)) {
			madctl = ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_RGB;
		} else {
			madctl = ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_BGR;
		}

		if (dev->tabColor == INITR_144GREENTAB) {
			dev->width = ST7735_TFTHEIGHT_128;
			dev->height = ST7735_TFTWIDTH_128;
		} else if (dev->tabColor == INITR_MINI160x80 ||
				dev->tabColor == INITR_MINI160x80_PLUGIN) {
			dev->width = ST7735_TFTHEIGHT_160;
			dev->height = ST7735_TFTWIDTH_80;
		} else {
			dev->width = ST7735_TFTHEIGHT_160;
			dev->height = ST7735_TFTWIDTH_128;
		}
		dev->ystart = dev->colstart;
		dev->xstart = dev->rowstart;
		break;
//======
/*
#if ST7735_IS_160X80
	  madctl = ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_BGR;
#else
      madctl = ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_RGB;
      dev->height = ST7735_HEIGHT;
      dev->width = ST7735_WIDTH;
      dev->xstart = dev->colStart;
      dev->ystart = dev->rowStart;
#endif
      break;
	case 1:
#if ST7735_IS_160X80
	  madctl = ST7735_MADCTL_MY | ST7735_MADCTL_MV | ST7735_MADCTL_BGR;
#else
      madctl = ST7735_MADCTL_MY | ST7735_MADCTL_MV | ST7735_MADCTL_RGB;
      dev->width = ST7735_HEIGHT;
      dev->height = ST7735_WIDTH;
      dev->ystart = dev->colStart;
      dev->xstart = dev->rowStart;
#endif
      break;
	case 2:
#if ST7735_IS_160X80
	  madctl = ST7735_MADCTL_BGR;
#else
      madctl = ST7735_MADCTL_RGB;
      dev->height = ST7735_HEIGHT;
      dev->width = ST7735_WIDTH;
      dev->xstart = dev->colStart;
      dev->ystart = dev->rowStart;
#endif
      break;
  case 3:
#if ST7735_IS_160X80
	  madctl = ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_BGR;
#else
      madctl = ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_RGB;
      dev->width = ST7735_HEIGHT;
      dev->height = ST7735_WIDTH;
      dev->ystart = dev->colStart;
      dev->xstart = dev->rowStart;
#endif
    break;
*/
  }
  ST7735_Select(dev);
  ST7735_WriteCommand(dev, ST7735_MADCTL);
  ST7735_WriteData(dev, &madctl,1);
  ST7735_Unselect(dev);
}

void ST7735_DrawPixel(LCD_HandleTypeDef* dev, uint16_t x, uint16_t y, uint16_t color) {
    if((x >= dev->width) || (y >= dev->height))
        return;

    ST7735_Select(dev);

    ST7735_SetAddressWindow(dev, x, y, x+1, y+1);
    uint8_t data[] = { color >> 8, color & 0xFF };
    //uint8_t data[] = { color & 0xFF, color >> 8 };
    ST7735_WriteData(dev, data, sizeof(data));

    ST7735_Unselect(dev);
}


/*
void ST7735_WriteChar(LCD_HandleTypeDef* dev, uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor) {
    uint32_t i, b, j;

    ST7735_SetAddressWindow(dev, x, y, x+font.width-1, y+font.height-1);

    for(i = 0; i < font.height; i++) {
        b = font.data[(ch - 32) * font.height + i];
        for(j = 0; j < font.width; j++) {
            if((b << j) & 0x8000)  {
                uint8_t data[] = { color >> 8, color & 0xFF };
                ST7735_WriteData(dev, data, sizeof(data));
            } else {
                uint8_t data[] = { bgcolor >> 8, bgcolor & 0xFF };
                ST7735_WriteData(dev, data, sizeof(data));
            }
        }
    }
}
*/
void ST7735_DrawChar(LCD_HandleTypeDef* dev, char c){
	// Character is assumed previously filtered by write() to eliminate
	// newlines, returns, non-printable characters, etc.  Calling
	// drawChar() directly with 'bad' characters of font may cause mayhem!

	c -= (uint8_t)pgm_read_byte(&dev->gfxFont->first);
	//GFXglyph *glyph = pgm_read_glyph_ptr(gfxFont, c);
	//
	//	inline GFXglyph *pgm_read_glyph_ptr(const GFXfont *gfxFont, uint8_t c) {
	//		return gfxFont->glyph + c;
	//	}
	GFXglyph *glyph = dev->gfxFont->glyph + c;

	uint8_t *bitmap = dev->gfxFont->bitmap;
	uint16_t x = dev->cursor_x;
	uint16_t y = dev->cursor_y;
	uint16_t bo = pgm_read_word(&glyph->bitmapOffset);
	uint8_t w = pgm_read_byte(&glyph->width), h = pgm_read_byte(&glyph->height);
	int8_t xo = pgm_read_byte(&glyph->xOffset),
		   yo = pgm_read_byte(&glyph->yOffset);
	uint8_t xx, yy, bits = 0, bit = 0;

	for (yy = 0; yy < h; yy++) {
		for (xx = 0; xx < w; xx++) {
			if (!(bit++ & 7)) {
				bits = pgm_read_byte(&bitmap[bo++]);
			}
			if(bits & 0x80){
				ST7735_DrawPixel(dev, x + xo + xx, y + yo + yy, dev->textColor);
			}else{
				ST7735_DrawPixel(dev, x + xo + xx, y + yo + yy, dev->textBgColor);
			}
			bits <<= 1;
		}
	}
}

uint8_t ST7735_WriteChar(LCD_HandleTypeDef* dev, char c){
	if(!dev->gfxFont) return 0;

	if (c == '\n') {
		dev->cursor_x = 0;
		dev->cursor_y += (uint8_t)pgm_read_byte(&dev->gfxFont->yAdvance);
	} else if (c != '\r') {
		uint8_t first = pgm_read_byte(&dev->gfxFont->first);
		if ((c >= first) && (c <= (uint8_t)pgm_read_byte(&dev->gfxFont->last))) {
			//GFXglyph *glyph = pgm_read_glyph_ptr(gfxFont, c - first);
			GFXglyph *glyph = dev->gfxFont->glyph + (c - first);
			uint8_t w = pgm_read_byte(&glyph->width),
					h = pgm_read_byte(&glyph->height);
			if ((w > 0) && (h > 0)) { // Is there an associated bitmap?
				int16_t xo = (int8_t)pgm_read_byte(&glyph->xOffset); // sic
				if (dev->wrapText && ((dev->cursor_x + (xo + w)) > dev->width)) {
					dev->cursor_x = 0;
					dev->cursor_y += (uint8_t)pgm_read_byte(&dev->gfxFont->yAdvance);
				}
				//ST7567_DrawChar(lcd.cursor_x, lcd.cursor_y, c, SET, lcd.textsize_x, lcd.textsize_y);
				ST7735_DrawChar(dev, c);
			}
			dev->cursor_x += (uint8_t)pgm_read_byte(&glyph->xAdvance);
		}
	}
	return 1;
}

/*
void ST7735_WriteString(LCD_HandleTypeDef* dev, uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor) {
    ST7735_Select(dev);

    while(*str) {
        if(x + font.width >= dev->width) {
            x = 0;
            y += font.height;
            if(y + font.height >= dev->height) {
                break;
            }

            if(*str == ' ') {
                // skip spaces in the beginning of the new line
                str++;
                continue;
            }
        }

        ST7735_WriteChar(dev, *str);
        x += font.width;
        str++;
    }

    uint16_t n=0;
    uint16_t size = sizeof()
    ST7735_Unselect(dev);
}
*/

size_t ST7735_Write(LCD_HandleTypeDef* dev, uint8_t *buffer, size_t size){
	ST7735_Select(dev);
	size_t n = 0;
	while (size--) {
		if (ST7735_WriteChar(dev, *buffer++)) n++;
		else break;
	}
	ST7735_Unselect(dev);
	return n;
}

size_t ST7735_Print(LCD_HandleTypeDef* dev, const char str[]){
	size_t n = 0;
	size_t size = strlen(str);
	ST7735_Select(dev);
	while (size--) {
		if (ST7735_WriteChar(dev, str[n])) n++;
		else break;
	}
	ST7735_Unselect(dev);
	return n;
}

void ST7735_FillRect(LCD_HandleTypeDef* dev, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if((x >= dev->width) || (y >= dev->height)) return;
    if((x + w - 1) >= dev->width) w = dev->width - x;
    if((y + h - 1) >= dev->height) h = dev->height - y;

    ST7735_Select(dev);
    ST7735_SetAddressWindow(dev, x, y, x+w-1, y+h-1);

    uint8_t data[] = { color >> 8, color & 0xFF };
    //uint8_t data[] = { color & 0xFF, color >> 8 };
    HAL_GPIO_WritePin(dev->dcPort, dev->dcPin, GPIO_PIN_SET);
    for(y = h; y > 0; y--) {
        for(x = w; x > 0; x--) {
            //HAL_SPI_Transmit(dev->hspi, data, sizeof(data), HAL_MAX_DELAY);
        	ST7735_WriteData(dev, data, sizeof(data));
        }
    }

    ST7735_Unselect(dev);
}

void ST7735_FillScreen(LCD_HandleTypeDef* dev, uint16_t color){
	ST7735_FillRect(dev, 0, 0, dev->width, dev->height, color);
}




void ST7735_DrawImage(LCD_HandleTypeDef* dev, uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data) {
    if((x >= dev->width) || (y >= dev->height)) return;
    if((x + w - 1) >= dev->width) return;
    if((y + h - 1) >= dev->height) return;

    ST7735_Select(dev);
    ST7735_SetAddressWindow(dev, x, y, x+w-1, y+h-1);
    ST7735_WriteData(dev, (uint8_t*)data, sizeof(uint16_t)*w*h);
    ST7735_Unselect(dev);
}

void ST7735_DrawImageDMA(LCD_HandleTypeDef* dev, uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data){
    if((x >= dev->width) || (y >= dev->height)) return;
    if((x + w - 1) >= dev->width) return;
    if((y + h - 1) >= dev->height) return;

    ST7735_Select(dev);
    ST7735_SetAddressWindow(dev, x, y, x+w-1, y+h-1);
    ST7735_WriteDataDMA(dev, (uint8_t*)data, sizeof(uint16_t)*w*h);
    //ST7735_Unselect(dev);
}

void ST7735_InvertColors(LCD_HandleTypeDef* dev, bool invert) {
    ST7735_Select(dev);
    ST7735_WriteCommand(dev, invert ? dev->invertOnCmd : dev->invertOffCmd);
    ST7735_Unselect(dev);
}



void ST7735_SetFont(LCD_HandleTypeDef* dev, const GFXfont *f) {
	dev->gfxFont = (GFXfont *)f;
}

void ST7735_SetCursor(LCD_HandleTypeDef* dev, uint16_t x, uint16_t y){
	dev->cursor_x = x;
	dev->cursor_y = y;
}

void ST7735_SetTextColor(LCD_HandleTypeDef* dev, uint16_t newTextColor, uint16_t newBackgroundColor){
	dev->textColor = newTextColor;
	dev->textBgColor = newBackgroundColor;
}

void ST7735_SetTextBGColor(LCD_HandleTypeDef* dev, uint16_t newColor){
	dev->textBgColor = newColor;
}

uint8_t ST7735_GetTextWidth(LCD_HandleTypeDef* dev, const char str[]){
	if(!dev->gfxFont) return 0;

	uint8_t lineW = 0, maxW = 0;	//in case there are multiple lines, track the widest line.

	uint8_t numChars = strlen(str);
	for(uint8_t n=0; n<numChars; n++){
		if( (str[n] == '\n') || (str[n] == '\r') ){
			//new line.
			lineW = 0;
		}else{
			uint8_t first = pgm_read_byte(&dev->gfxFont->first);
			if ((str[n] >= first) && (str[n] <= (uint8_t)pgm_read_byte(&dev->gfxFont->last))) {
				GFXglyph *glyph = dev->gfxFont->glyph + (str[n] - first);
				if(n < (numChars-1)){
					//If not the last char in the array then use the xAdvance for the width
					lineW += (uint8_t)pgm_read_byte(&glyph->xAdvance);
				}else{
					//For the last char in the array, just use the width and not the xAdvance.
					lineW+= pgm_read_byte(&glyph->width);
				}
			}
		}
		maxW = (lineW > maxW) ? lineW : maxW;
	}
	return maxW;
}

uint8_t ST7735_GetTextHeight(LCD_HandleTypeDef* dev, const char str[], bool includeBelowTheLine){
	if(!dev->gfxFont) return 0;

	uint8_t maxHAbove=0, maxHTotal=0, curAbove=0, curTotal=0;	//Max height above the line and max height overall
	uint8_t numChars = strlen(str);
	for(uint8_t n=0; n<numChars; n++){
		int8_t first = pgm_read_byte(&dev->gfxFont->first);
		if ((str[n] >= first) && (str[n] <= (uint8_t)pgm_read_byte(&dev->gfxFont->last))) {
			GFXglyph *glyph = dev->gfxFont->glyph + (str[n] - first);
			curAbove = (int8_t)(pgm_read_byte(&glyph->yOffset) * -1);
			curTotal = (uint8_t)pgm_read_byte(&glyph->height);
			maxHAbove = (curAbove > maxHAbove) ? curAbove : maxHAbove;
			maxHTotal = (curTotal > maxHTotal) ? curTotal : maxHTotal;
		}
	}
	if(includeBelowTheLine){
		return maxHTotal;
	}else{
		return maxHAbove;
	}
}



void ST7735_WriteLine(LCD_HandleTypeDef* dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color){
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        _swap_int16_t(x0, y0);
        _swap_int16_t(x1, y1);
    }

    if (x0 > x1) {
        _swap_int16_t(x0, x1);
        _swap_int16_t(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    for (; x0<=x1; x0++) {
        if (steep) {
        	//ST7735_WritePixel(dev, y0, x0, color);
        	ST7735_DrawPixel(dev, y0, x0, color);
        } else {
        	//ST7735_WritePixel(dev, x0, y0, color);
        	ST7735_DrawPixel(dev, x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}

void ST7735_DrawFastVLine(LCD_HandleTypeDef* dev, int16_t x, int16_t y, int16_t h, uint16_t color){
	ST7735_WriteLine(dev, x, y, x, y + h - 1, color);
}

void ST7735_DrawFastHLine(LCD_HandleTypeDef* dev, int16_t x, int16_t y, int16_t w, uint16_t color){
	ST7735_WriteLine(dev, x, y, x + w - 1, y, color);
}

void ST7735_DrawLine(LCD_HandleTypeDef* dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color){
    if(x0 == x1){
        if(y0 > y1) _swap_int16_t(y0, y1);
        ST7735_DrawFastVLine(dev, x0, y0, y1 - y0 + 1, color);
    } else if(y0 == y1){
        if(x0 > x1) _swap_int16_t(x0, x1);
        ST7735_DrawFastHLine(dev, x0, y0, x1 - x0 + 1, color);
    } else {
    	ST7735_WriteLine(dev, x0, y0, x1, y1, color);
    }
}

void ST7735_DrawCircle(LCD_HandleTypeDef* dev, int16_t x0, int16_t y0, int16_t r, uint16_t color){
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    ST7735_DrawPixel(dev, x0  , y0+r, color);
    ST7735_DrawPixel(dev, x0  , y0-r, color);
    ST7735_DrawPixel(dev, x0+r, y0  , color);
    ST7735_DrawPixel(dev, x0-r, y0  , color);

    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        ST7735_DrawPixel(dev, x0 + x, y0 + y, color);
        ST7735_DrawPixel(dev, x0 - x, y0 + y, color);
        ST7735_DrawPixel(dev, x0 + x, y0 - y, color);
        ST7735_DrawPixel(dev, x0 - x, y0 - y, color);
        ST7735_DrawPixel(dev, x0 + y, y0 + x, color);
        ST7735_DrawPixel(dev, x0 - y, y0 + x, color);
        ST7735_DrawPixel(dev, x0 + y, y0 - x, color);
        ST7735_DrawPixel(dev, x0 - y, y0 - x, color);
    }
}

void ST7735_DrawCircleHelper(LCD_HandleTypeDef* dev, int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color){
    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;

    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;
        if (cornername & 0x4) {
        	ST7735_DrawPixel(dev, x0 + x, y0 + y, color);
        	ST7735_DrawPixel(dev, x0 + y, y0 + x, color);
        }
        if (cornername & 0x2) {
        	ST7735_DrawPixel(dev, x0 + x, y0 - y, color);
        	ST7735_DrawPixel(dev, x0 + y, y0 - x, color);
        }
        if (cornername & 0x8) {
        	ST7735_DrawPixel(dev, x0 - y, y0 + x, color);
        	ST7735_DrawPixel(dev, x0 - x, y0 + y, color);
        }
        if (cornername & 0x1) {
        	ST7735_DrawPixel(dev, x0 - y, y0 - x, color);
        	ST7735_DrawPixel(dev, x0 - x, y0 - y, color);
        }
    }
}

void ST7735_FillCircleHelper(LCD_HandleTypeDef* dev, int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color){
    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;
    int16_t px    = x;
    int16_t py    = y;

    delta++; // Avoid some +1's in the loop

    while(x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;
        // These checks avoid double-drawing certain lines, important
        // for the SSD1306 library which has an INVERT drawing mode.
        if(x < (y + 1)) {
            if(corners & 1) ST7735_DrawFastVLine(dev, x0+x, y0-y, 2*y+delta, color);
            if(corners & 2) ST7735_DrawFastVLine(dev, x0-x, y0-y, 2*y+delta, color);
        }
        if(y != py) {
            if(corners & 1) ST7735_DrawFastVLine(dev, x0+py, y0-px, 2*px+delta, color);
            if(corners & 2) ST7735_DrawFastVLine(dev, x0-py, y0-px, 2*px+delta, color);
            py = y;
        }
        px = x;
    }
}

void ST7735_FillCircle(LCD_HandleTypeDef* dev, int16_t x0, int16_t y0, int16_t r, uint16_t color){
	ST7735_DrawFastVLine(dev, x0, y0-r, 2*r+1, color);
    ST7735_FillCircleHelper(dev, x0, y0, r, 3, 0, color);
}

void ST7735_DrawRect(LCD_HandleTypeDef* dev, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color){
	ST7735_DrawFastHLine(dev, x, y, w, color);
	ST7735_DrawFastHLine(dev, x, y+h-1, w, color);
	ST7735_DrawFastVLine(dev, x, y, h, color);
	ST7735_DrawFastVLine(dev, x+w-1, y, h, color);
}

void ST7735_DrawRoundRect(LCD_HandleTypeDef* dev, int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color){
    int16_t max_radius = ((w < h) ? w : h) / 2; // 1/2 minor axis
    if(r > max_radius) r = max_radius;
    // smarter version
    ST7735_DrawFastHLine(dev, x+r  , y    , w-2*r, color); // Top
    ST7735_DrawFastHLine(dev, x+r  , y+h-1, w-2*r, color); // Bottom
    ST7735_DrawFastVLine(dev, x    , y+r  , h-2*r, color); // Left
    ST7735_DrawFastVLine(dev, x+w-1, y+r  , h-2*r, color); // Right
    // draw four corners
    ST7735_DrawCircleHelper(dev, x+r    , y+r    , r, 1, color);
    ST7735_DrawCircleHelper(dev, x+w-r-1, y+r    , r, 2, color);
    ST7735_DrawCircleHelper(dev, x+w-r-1, y+h-r-1, r, 4, color);
    ST7735_DrawCircleHelper(dev, x+r    , y+h-r-1, r, 8, color);
}


void ST7735_FillRoundRect(LCD_HandleTypeDef* dev, int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color){
    int16_t max_radius = ((w < h) ? w : h) / 2; // 1/2 minor axis
    if(r > max_radius) r = max_radius;
    // smarter version
    ST7735_FillRect(dev, x+r, y, w-2*r, h, color);
    // draw four corners
    ST7735_FillCircleHelper(dev, x+w-r-1, y+r, r, 1, h-2*r-1, color);
    ST7735_FillCircleHelper(dev, x+r    , y+r, r, 2, h-2*r-1, color);
}


void ST7735_DrawTriangle(LCD_HandleTypeDef* dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color){
	ST7735_DrawLine(dev, x0, y0, x1, y1, color);
	ST7735_DrawLine(dev, x1, y1, x2, y2, color);
	ST7735_DrawLine(dev, x2, y2, x0, y0, color);
}


void ST7735_FillTriangle(LCD_HandleTypeDef* dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color){
    int16_t a, b, y, last;
    // Sort coordinates by Y order (y2 >= y1 >= y0)
    if (y0 > y1) {
        _swap_int16_t(y0, y1); _swap_int16_t(x0, x1);
    }
    if (y1 > y2) {
        _swap_int16_t(y2, y1); _swap_int16_t(x2, x1);
    }
    if (y0 > y1) {
        _swap_int16_t(y0, y1); _swap_int16_t(x0, x1);
    }

    if(y0 == y2) { // Handle awkward all-on-same-line case as its own thing
        a = b = x0;
        if(x1 < a)      a = x1;
        else if(x1 > b) b = x1;
        if(x2 < a)      a = x2;
        else if(x2 > b) b = x2;
        ST7735_DrawFastHLine(dev, a, y0, b-a+1, color);
        return;
    }

    int16_t
    dx01 = x1 - x0,
    dy01 = y1 - y0,
    dx02 = x2 - x0,
    dy02 = y2 - y0,
    dx12 = x2 - x1,
    dy12 = y2 - y1;
    int32_t
    sa   = 0,
    sb   = 0;

    // For upper part of triangle, find scanline crossings for segments
    // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
    // is included here (and second loop will be skipped, avoiding a /0
    // error there), otherwise scanline y1 is skipped here and handled
    // in the second loop...which also avoids a /0 error here if y0=y1
    // (flat-topped triangle).
    if(y1 == y2) last = y1;   // Include y1 scanline
    else         last = y1-1; // Skip it

    for(y=y0; y<=last; y++) {
        a   = x0 + sa / dy01;
        b   = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;
        /* longhand:
        a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
        b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
        */
        if(a > b) _swap_int16_t(a,b);
        ST7735_DrawFastHLine(dev, a, y, b-a+1, color);
    }

    // For lower part of triangle, find scanline crossings for segments
    // 0-2 and 1-2.  This loop is skipped if y1=y2.
    sa = (int32_t)dx12 * (y - y1);
    sb = (int32_t)dx02 * (y - y0);
    for(; y<=y2; y++) {
        a   = x1 + sa / dy12;
        b   = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;
        /* longhand:
        a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
        b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
        */
        if(a > b) _swap_int16_t(a,b);
        ST7735_DrawFastHLine(dev, a, y, b-a+1, color);
    }
}






//===Canvas===



void ST7735_InitCanvas(LCD_HandleTypeDef* dev, LCD_CanvasHandleTypeDef* newCanvas, uint16_t w, uint16_t h){
	newCanvas->dev = dev;
	newCanvas->width = w;
	newCanvas->height = h;
	uint32_t bytes = w * h;
	if ((newCanvas->buffer = (uint16_t *)malloc(bytes))){
		memset(newCanvas->buffer, 0, bytes);
	}
}
void ST7735_DeleteCanvas(LCD_CanvasHandleTypeDef* can){
	free(can->buffer);
}

void ST7735_DrawPixelCanvas(LCD_CanvasHandleTypeDef* can, int16_t x, int16_t y, uint16_t color){
	if (can->buffer) {
		if ((x < 0) || (y < 0) || (x >= can->width) || (y >= can->height))
			return;

//		int16_t t;
//		switch (can->dev->rotation) {
//		case 1:
//			t = x;
//			//x = WIDTH - 1 - y;
//			x = can->dev->width - 1 - y;
//			y = t;
//			break;
//		case 2:
//			x = can->dev->width - 1 - x;
//			y = can->dev->height - 1 - y;
//			break;
//		case 3:
//			t = x;
//			x = y;
//			y = can->dev->height - 1 - t;
//			break;
//		}

		//buffer[x + y * WIDTH] = color;
		//can->buffer[x + y * can->dev->width] = color;
		//uint8_t data[] = { color >> 8, color & 0xFF };
		uint16_t revColor = (color >> 8) | (color << 8);
		can->buffer[x + y * can->dev->width] = revColor;
	}
}

uint16_t ST7735_GetPixelCanvas(LCD_CanvasHandleTypeDef* can, int16_t x, int16_t y){
//	int16_t t;
//	switch (can->dev->rotation) {
//	case 1:
//		t = x;
//		x = can->dev->width - 1 - y;
//		y = t;
//		break;
//	case 2:
//		x = can->dev->width - 1 - x;
//		y = can->dev->height - 1 - y;
//		break;
//	case 3:
//		t = x;
//		x = y;
//		y = can->dev->height - 1 - t;
//		break;
//	}
	return ST7735_GetRawPixelCanvas(can, x, y);
}
uint16_t ST7735_GetRawPixelCanvas(LCD_CanvasHandleTypeDef* can, int16_t x, int16_t y){
	if ((x < 0) || (y < 0) || (x >= can->dev->width) || (y >= can->dev->height))
		return 0;
	if (can->buffer) {
		return can->buffer[x + y * can->dev->width];
	}
	return 0;
}


void ST7735_FillScreenCanvas(LCD_CanvasHandleTypeDef* can, uint16_t color){
	ST7735_FillRectCanvas(can, 0, 0, can->width, can->height, color);
}

void ST7735_FillRectCanvas(LCD_CanvasHandleTypeDef* can, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color){
	uint16_t indx = 0;

	for(uint16_t row = y; row<(y+h); row++){
		for(uint16_t col=x; col<(x+w); col++){
			indx = (row * can->width) + col;
			uint16_t revColor = (color >> 8) | (color << 8);
			can->buffer[indx] = revColor;
		}
	}
}


void ST7735_SetFontCanvas(LCD_CanvasHandleTypeDef* can, const GFXfont *f){
	can->gfxFont = (GFXfont *)f;
}

void ST7735_SetCursorCanvas(LCD_CanvasHandleTypeDef* can, uint16_t x, uint16_t y){
	can->cursor_x = x;
	can->cursor_y = y;
}

void ST7735_SetTextColorCanvas(LCD_CanvasHandleTypeDef* can, uint16_t newTextColor, uint16_t newBackgroundColor){
	can->textColor = newTextColor;
	can->textBgColor = newBackgroundColor;
}

void ST7735_SetTextBGColorCanvas(LCD_CanvasHandleTypeDef* can, uint16_t newColor){
	can->textBgColor = newColor;
}



void ST7735_DrawCharCanvas(LCD_CanvasHandleTypeDef* can, char c){
	// Character is assumed previously filtered by write() to eliminate
	// newlines, returns, non-printable characters, etc.  Calling
	// drawChar() directly with 'bad' characters of font may cause mayhem!

	c -= (uint8_t)pgm_read_byte(&can->gfxFont->first);
	GFXglyph *glyph = can->gfxFont->glyph + c;

	uint8_t *bitmap = can->gfxFont->bitmap;
	uint16_t x = can->cursor_x;
	uint16_t y = can->cursor_y;
	uint16_t bo = pgm_read_word(&glyph->bitmapOffset);
	uint8_t w = pgm_read_byte(&glyph->width), h = pgm_read_byte(&glyph->height);
	int8_t xo = pgm_read_byte(&glyph->xOffset),
		   yo = pgm_read_byte(&glyph->yOffset);
	uint8_t xx, yy, bits = 0, bit = 0;

	for (yy = 0; yy < h; yy++) {
		for (xx = 0; xx < w; xx++) {
			if (!(bit++ & 7)) {
				bits = pgm_read_byte(&bitmap[bo++]);
			}
			if(bits & 0x80){
				ST7735_DrawPixelCanvas(can, x + xo + xx, y + yo + yy, can->textColor);
			}else{
				ST7735_DrawPixelCanvas(can, x + xo + xx, y + yo + yy, can->textBgColor);
			}
			bits <<= 1;
		}
	}
}

uint8_t ST7735_WriteCharCanvas(LCD_CanvasHandleTypeDef* can, char c){
	if(!can->gfxFont) return 0;

	if (c == '\n') {
		can->cursor_x = 0;
		can->cursor_y += (uint8_t)pgm_read_byte(&can->gfxFont->yAdvance);
	} else if (c != '\r') {
		uint8_t first = pgm_read_byte(&can->gfxFont->first);
		if ((c >= first) && (c <= (uint8_t)pgm_read_byte(&can->gfxFont->last))) {
			//GFXglyph *glyph = pgm_read_glyph_ptr(gfxFont, c - first);
			GFXglyph *glyph = can->gfxFont->glyph + (c - first);
			uint8_t w = pgm_read_byte(&glyph->width),
					h = pgm_read_byte(&glyph->height);
			if ((w > 0) && (h > 0)) { // Is there an associated bitmap?
				int16_t xo = (int8_t)pgm_read_byte(&glyph->xOffset); // sic
				if (can->wrapText && ((can->cursor_x + (xo + w)) > can->width)) {
					can->cursor_x = 0;
					can->cursor_y += (uint8_t)pgm_read_byte(&can->gfxFont->yAdvance);
				}
				//ST7567_DrawChar(lcd.cursor_x, lcd.cursor_y, c, SET, lcd.textsize_x, lcd.textsize_y);
				ST7735_DrawCharCanvas(can, c);
			}
			can->cursor_x += (uint8_t)pgm_read_byte(&glyph->xAdvance);
		}
	}
	return 1;
}

size_t ST7735_WriteTextCanvas(LCD_CanvasHandleTypeDef* can, uint8_t *buffer, size_t size){
	size_t n = 0;
	while (size--) {
		if (ST7735_WriteCharCanvas(can, *buffer++)) n++;
		else break;
	}
	return n;
}

size_t ST7735_PrintTextCanvas(LCD_CanvasHandleTypeDef* can, const char str[]){
	size_t n = 0;
	size_t size = strlen(str);
	while (size--) {
		if (ST7735_WriteCharCanvas(can, str[n])) n++;
		else break;
	}
	return n;
}



void ST7735_DrawFastRawVLineCanvas(LCD_CanvasHandleTypeDef* can, int16_t x, int16_t y, int16_t h, uint16_t color){
	// x & y already in raw (rotation 0) coordinates, no need to transform.
	uint16_t *buffer_ptr = can->buffer + y * can->dev->width + x;
	for (int16_t i = 0; i < h; i++) {
		uint16_t revColor = (color >> 8) | (color << 8);
		(*buffer_ptr) = revColor;
		buffer_ptr += can->dev->width;
	}
}

void ST7735_DrawFastVLineCanvas(LCD_CanvasHandleTypeDef* can, int16_t x, int16_t y, int16_t h, uint16_t color){
	if (h < 0) { // Convert negative heights to positive equivalent
		h *= -1;
		y -= h - 1;
		if (y < 0) {
			h += y;
			y = 0;
		}
	}

	// Edge rejection (no-draw if totally off canvas)
	if ((x < 0) || (x >= can->width) || (y >= can->height) || ((y + h - 1) < 0)) {
		return;
	}

	if (y < 0) { // Clip top
		h += y;
		y = 0;
	}
	if (y + h > can->height) { // Clip bottom
		h = can->height - y;
	}

//	int16_t t = x;
//	switch(can->dev->rotation){
//	case 0:
		ST7735_DrawFastRawVLineCanvas(can, x, y, h, color);
//		break;
//	case 1:
//		//int16_t t = x;
//		x = ST7735_WIDTH - 1 - y;
//		y = t;
//		x -= h - 1;
//		ST7735_DrawFastRawHLineCanvas(can, x, y, h, color);
//		break;
//	case 2:
//		x = ST7735_WIDTH - 1 - x;
//		y = ST7735_HEIGHT - 1 - y;
//
//		y -= h - 1;
//		ST7735_DrawFastRawVLineCanvas(can, x, y, h, color);
//		break;
//	case 3:
//	default:
//		//int16_t t = x;
//		x = y;
//		y = can->dev->height - 1 - t;
//		ST7735_DrawFastRawHLineCanvas(can, x, y, h, color);
//		break;
//	}
}

void ST7735_DrawFastRawHLineCanvas(LCD_CanvasHandleTypeDef* can, int16_t x, int16_t y, int16_t w, uint16_t color){
	// x & y already in raw (rotation 0) coordinates, no need to transform.
	uint32_t buffer_index = y * can->dev->width + x;
	for (uint32_t i = buffer_index; i < buffer_index + w; i++) {
		uint16_t revColor = (color >> 8) | (color << 8);
		can->buffer[i] = revColor;

	}
}
void ST7735_DrawFastHLineCanvas(LCD_CanvasHandleTypeDef* can, int16_t x, int16_t y, int16_t w, uint16_t color){
	if (w < 0) { // Convert negative widths to positive equivalent
		w *= -1;
		x -= w - 1;
		if (x < 0) {
			w += x;
			x = 0;
		}
	}

	// Edge rejection (no-draw if totally off canvas)
	if ((y < 0) || (y >= can->height) || (x >= can->width) || ((x + w - 1) < 0)) {
		return;
	}

	if (x < 0) { // Clip left
		w += x;
		x = 0;
	}
	if (x + w >= can->width) { // Clip right
		w = can->width - x;
	}

//	int16_t t = x;
//	switch(can->dev->rotation){
//	case 0:
		ST7735_DrawFastRawHLineCanvas(can, x, y, w, color);
//		break;
//	case 1:
//		//int16_t t = x;
//		x = ST7735_WIDTH - 1 - y;
//		y = t;
//		ST7735_DrawFastRawVLineCanvas(can, x, y, w, color);
//		break;
//	case 2:
//		x = ST7735_WIDTH - 1 - x;
//		y = ST7735_HEIGHT - 1 - y;
//
//		x -= w - 1;
//		ST7735_DrawFastRawHLineCanvas(can, x, y, w, color);
//		break;
//	case 3:
//	default:
//		int16_t t = x;
//		x = y;
//		y = ST7735_HEIGHT - 1 - t;
//		y -= w - 1;
//		ST7735_DrawFastRawVLineCanvas(can, x, y, w, color);
//		break;
//	}
}


void ST7735_WriteLineCanvas(LCD_CanvasHandleTypeDef* can, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color){
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        _swap_int16_t(x0, y0);
        _swap_int16_t(x1, y1);
    }

    if (x0 > x1) {
        _swap_int16_t(x0, x1);
        _swap_int16_t(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    for (; x0<=x1; x0++) {
        if (steep) {
        	//ST7735_WritePixel(dev, y0, x0, color);
        	ST7735_DrawPixelCanvas(can, y0, x0, color);
        } else {
        	//ST7735_WritePixel(dev, x0, y0, color);
        	ST7735_DrawPixelCanvas(can, x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}

void ST7735_DrawLineCanvas(LCD_CanvasHandleTypeDef* can, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color){
	if(x0 == x1){
		if(y0 > y1) _swap_int16_t(y0, y1);
		ST7735_DrawFastVLineCanvas(can, x0, y0, y1 - y0 + 1, color);
	} else if(y0 == y1){
		if(x0 > x1) _swap_int16_t(x0, x1);
		ST7735_DrawFastHLineCanvas(can, x0, y0, x1 - x0 + 1, color);
	} else {
		ST7735_WriteLineCanvas(can, x0, y0, x1, y1, color);
	}
}

void ST7735_DrawCircleCanvas(LCD_CanvasHandleTypeDef* can, int16_t x0, int16_t y0, int16_t r, uint16_t color){
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    ST7735_DrawPixelCanvas(can, x0  , y0+r, color);
    ST7735_DrawPixelCanvas(can, x0  , y0-r, color);
    ST7735_DrawPixelCanvas(can, x0+r, y0  , color);
    ST7735_DrawPixelCanvas(can, x0-r, y0  , color);

    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        ST7735_DrawPixelCanvas(can, x0 + x, y0 + y, color);
        ST7735_DrawPixelCanvas(can, x0 - x, y0 + y, color);
        ST7735_DrawPixelCanvas(can, x0 + x, y0 - y, color);
        ST7735_DrawPixelCanvas(can, x0 - x, y0 - y, color);
        ST7735_DrawPixelCanvas(can, x0 + y, y0 + x, color);
        ST7735_DrawPixelCanvas(can, x0 - y, y0 + x, color);
        ST7735_DrawPixelCanvas(can, x0 + y, y0 - x, color);
        ST7735_DrawPixelCanvas(can, x0 - y, y0 - x, color);
    }
}

void ST7735_DrawCircleHelperCanvas(LCD_CanvasHandleTypeDef* can, int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color){
    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;

    while (x<y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;
        if (cornername & 0x4) {
        	ST7735_DrawPixelCanvas(can, x0 + x, y0 + y, color);
        	ST7735_DrawPixelCanvas(can, x0 + y, y0 + x, color);
        }
        if (cornername & 0x2) {
        	ST7735_DrawPixelCanvas(can, x0 + x, y0 - y, color);
        	ST7735_DrawPixelCanvas(can, x0 + y, y0 - x, color);
        }
        if (cornername & 0x8) {
        	ST7735_DrawPixelCanvas(can, x0 - y, y0 + x, color);
        	ST7735_DrawPixelCanvas(can, x0 - x, y0 + y, color);
        }
        if (cornername & 0x1) {
        	ST7735_DrawPixelCanvas(can, x0 - y, y0 - x, color);
        	ST7735_DrawPixelCanvas(can, x0 - x, y0 - y, color);
        }
    }
}

void ST7735_FillCircleHelperCanvas(LCD_CanvasHandleTypeDef* can, int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color){
    int16_t f     = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x     = 0;
    int16_t y     = r;
    int16_t px    = x;
    int16_t py    = y;

    delta++; // Avoid some +1's in the loop

    while(x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f     += ddF_y;
        }
        x++;
        ddF_x += 2;
        f     += ddF_x;
        // These checks avoid double-drawing certain lines, important
        // for the SSD1306 library which has an INVERT drawing mode.
        if(x < (y + 1)) {
            if(corners & 1) ST7735_DrawFastVLineCanvas(can, x0+x, y0-y, 2*y+delta, color);
            if(corners & 2) ST7735_DrawFastVLineCanvas(can, x0-x, y0-y, 2*y+delta, color);
        }
        if(y != py) {
            if(corners & 1) ST7735_DrawFastVLineCanvas(can, x0+py, y0-px, 2*px+delta, color);
            if(corners & 2) ST7735_DrawFastVLineCanvas(can, x0-py, y0-px, 2*px+delta, color);
            py = y;
        }
        px = x;
    }
}

void ST7735_FillCircleCanvas(LCD_CanvasHandleTypeDef* can, int16_t x0, int16_t y0, int16_t r, uint16_t color){
	ST7735_DrawFastVLineCanvas(can, x0, y0-r, 2*r+1, color);
    ST7735_FillCircleHelperCanvas(can, x0, y0, r, 3, 0, color);
}

void ST7735_DrawRectCanvas(LCD_CanvasHandleTypeDef* can, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color){
	ST7735_DrawFastHLineCanvas(can, x, y, w, color);
	ST7735_DrawFastHLineCanvas(can, x, y+h-1, w, color);
	ST7735_DrawFastVLineCanvas(can, x, y, h, color);
	ST7735_DrawFastVLineCanvas(can, x+w-1, y, h, color);
}

void ST7735_DrawRoundRectCanvas(LCD_CanvasHandleTypeDef* can, int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color){
    int16_t max_radius = ((w < h) ? w : h) / 2; // 1/2 minor axis
    if(r > max_radius) r = max_radius;
    // smarter version
    ST7735_DrawFastHLineCanvas(can, x+r  , y    , w-2*r, color); // Top
    ST7735_DrawFastHLineCanvas(can, x+r  , y+h-1, w-2*r, color); // Bottom
    ST7735_DrawFastVLineCanvas(can, x    , y+r  , h-2*r, color); // Left
    ST7735_DrawFastVLineCanvas(can, x+w-1, y+r  , h-2*r, color); // Right
    // draw four corners
    ST7735_DrawCircleHelperCanvas(can, x+r    , y+r    , r, 1, color);
    ST7735_DrawCircleHelperCanvas(can, x+w-r-1, y+r    , r, 2, color);
    ST7735_DrawCircleHelperCanvas(can, x+w-r-1, y+h-r-1, r, 4, color);
    ST7735_DrawCircleHelperCanvas(can, x+r    , y+h-r-1, r, 8, color);
}

void ST7735_FillRoundRectCanvas(LCD_CanvasHandleTypeDef* can, int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color){
    int16_t max_radius = ((w < h) ? w : h) / 2; // 1/2 minor axis
    if(r > max_radius) r = max_radius;
    // smarter version
    ST7735_FillRectCanvas(can, x+r, y, w-2*r, h, color);
    // draw four corners
    ST7735_FillCircleHelperCanvas(can, x+w-r-1, y+r, r, 1, h-2*r-1, color);
    ST7735_FillCircleHelperCanvas(can, x+r    , y+r, r, 2, h-2*r-1, color);
}

void ST7735_DrawTriangleCanvas(LCD_CanvasHandleTypeDef* can, int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color){
	ST7735_DrawLineCanvas(can, x0, y0, x1, y1, color);
	ST7735_DrawLineCanvas(can, x1, y1, x2, y2, color);
	ST7735_DrawLineCanvas(can, x2, y2, x0, y0, color);
}

void ST7735_FillTriangleCanvas(LCD_CanvasHandleTypeDef* can, int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color){
    int16_t a, b, y, last;
    // Sort coordinates by Y order (y2 >= y1 >= y0)
    if (y0 > y1) {
        _swap_int16_t(y0, y1); _swap_int16_t(x0, x1);
    }
    if (y1 > y2) {
        _swap_int16_t(y2, y1); _swap_int16_t(x2, x1);
    }
    if (y0 > y1) {
        _swap_int16_t(y0, y1); _swap_int16_t(x0, x1);
    }

    if(y0 == y2) { // Handle awkward all-on-same-line case as its own thing
        a = b = x0;
        if(x1 < a)      a = x1;
        else if(x1 > b) b = x1;
        if(x2 < a)      a = x2;
        else if(x2 > b) b = x2;
        ST7735_DrawFastHLineCanvas(can, a, y0, b-a+1, color);
        return;
    }

    int16_t
    dx01 = x1 - x0,
    dy01 = y1 - y0,
    dx02 = x2 - x0,
    dy02 = y2 - y0,
    dx12 = x2 - x1,
    dy12 = y2 - y1;
    int32_t
    sa   = 0,
    sb   = 0;

    // For upper part of triangle, find scanline crossings for segments
    // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
    // is included here (and second loop will be skipped, avoiding a /0
    // error there), otherwise scanline y1 is skipped here and handled
    // in the second loop...which also avoids a /0 error here if y0=y1
    // (flat-topped triangle).
    if(y1 == y2) last = y1;   // Include y1 scanline
    else         last = y1-1; // Skip it

    for(y=y0; y<=last; y++) {
        a   = x0 + sa / dy01;
        b   = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;
        /* longhand:
        a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
        b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
        */
        if(a > b) _swap_int16_t(a,b);
        ST7735_DrawFastHLineCanvas(can, a, y, b-a+1, color);
    }

    // For lower part of triangle, find scanline crossings for segments
    // 0-2 and 1-2.  This loop is skipped if y1=y2.
    sa = (int32_t)dx12 * (y - y1);
    sb = (int32_t)dx02 * (y - y0);
    for(; y<=y2; y++) {
        a   = x1 + sa / dy12;
        b   = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;
        /* longhand:
        a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
        b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
        */
        if(a > b) _swap_int16_t(a,b);
        ST7735_DrawFastHLineCanvas(can, a, y, b-a+1, color);
    }
}

void ST7735_WriteCanvas(LCD_CanvasHandleTypeDef* can){
	ST7735_DrawImage(can->dev, can->left, can->top, can->width, can->height, can->buffer);
}

void ST7735_WriteCanvasDMA(LCD_CanvasHandleTypeDef* can){
	ST7735_DrawImageDMA(can->dev, can->left, can->top, can->width, can->height, can->buffer);
}
