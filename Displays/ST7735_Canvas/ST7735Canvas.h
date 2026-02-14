

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __ST7735_H__
#define __ST7735_H__

#include "gfxfont.h"
#include "fonts.h"
#include "main.h"
#include <stdbool.h>
#include "stdlib.h"


// some flags for initR() :(
#define INITB_NOOPTIONS 0xFF
#define INITR_GREENTAB 0x00
#define INITR_REDTAB 0x01
#define INITR_BLACKTAB 0x02
#define INITR_18GREENTAB INITR_GREENTAB
#define INITR_18REDTAB INITR_REDTAB
#define INITR_18BLACKTAB INITR_BLACKTAB
#define INITR_144GREENTAB 0x01
#define INITR_MINI160x80 0x04
#define INITR_HALLOWING 0x05
#define INITR_MINI160x80_PLUGIN 0x06

#define ST7735_TFTWIDTH_128 128  // for 1.44 and mini
#define ST7735_TFTWIDTH_80 80    // for mini
#define ST7735_TFTHEIGHT_128 128 // for 1.44" display
#define ST7735_TFTHEIGHT_160 160 // for 1.8" and mini display


#define ST7735_WIDTH  80
#define ST7735_HEIGHT 160

#define ST_CMD_DELAY 0x80

#define ST7735_MADCTL_MY  0x80
#define ST7735_MADCTL_MX  0x40
#define ST7735_MADCTL_MV  0x20
#define ST7735_MADCTL_ML  0x10
#define ST7735_MADCTL_RGB 0x00
#define ST7735_MADCTL_BGR 0x08
#define ST7735_MADCTL_MH  0x04

#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09

#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13

#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E

#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5

#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD

#define ST7735_PWCTR6  0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

// Color definitions
#define	BLACK   	0x0000
#define LIGHTGRAY	0xCCD5
#define GRAY		0X8410
#define DARKGRAY	0x2104
#define	BLUE    	0x001F
#define	RED     	0xF800
#define	GREEN   	0x07E0
#define CYAN    	0x07FF
#define MAGENTA 	0xF81F
#define YELLOW  	0xFFE0
#define WHITE   	0xFFFF
#define color565(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))

typedef struct {
	SPI_HandleTypeDef* 	hspi;
	volatile uint8_t	dmaWriteActive;
	GPIO_TypeDef*		csPort;
	uint16_t			csPin;
	GPIO_TypeDef*		dcPort;
	uint16_t			dcPin;
	GPIO_TypeDef*		rstPort;
	uint16_t			rstPin;

	uint16_t			width;			// Display width as modified by current rotation
	uint16_t			height;			// Display height as modified by current rotation
	uint8_t				rotation;		// Display rotation (0 thru 3)
	int16_t 			cursor_x;     	// x location to start print()ing text
	int16_t 			cursor_y;     	// y location to start print()ing text
	uint8_t 			colstart;		// Some displays need this changed to offset
	uint8_t 			rowstart;		// Some displays need this changed to offset
	uint8_t 			xstart;
	uint8_t 			ystart;

	uint8_t				tabColor;		// Also called "options"
	uint8_t				invertOnCmd;
	uint8_t 			invertOffCmd;

	GFXfont 			*gfxFont;     		// Main text font to use
	uint16_t			textColor;
	uint16_t			textBgColor;
	bool				wrapText;

	//uint16_t			scr[ST7735_WIDTH * ST7735_HEIGHT];

} LCD_HandleTypeDef;

typedef struct {
	LCD_HandleTypeDef*	dev;
	uint16_t			width;
	uint16_t			height;
	uint16_t			top;
	uint16_t			left;

	int16_t 			cursor_x;     	// x location to start print()ing text
	int16_t 			cursor_y;     	// y location to start print()ing text
	GFXfont 			*gfxFont;     		// Main text font to use
	uint16_t			textColor;
	uint16_t			textBgColor;
	bool				wrapText;

	uint16_t				*buffer;	//buffer for pixel data
} LCD_CanvasHandleTypeDef;

void ST7735_Init(LCD_HandleTypeDef* dev, uint16_t w, uint16_t h, SPI_HandleTypeDef* spi, GPIO_TypeDef* csPort, uint16_t csPin,
			GPIO_TypeDef* dcPort, uint16_t dcPin, GPIO_TypeDef* rstPort, uint16_t rstPin, uint8_t rotation,
			uint8_t options, bool finishInit);  //INITR_MINI160x80_PLUGIN
void ST7735_Start(LCD_HandleTypeDef* dev, uint32_t preDelay);

void ST7735_InitB(LCD_HandleTypeDef* dev);
void ST7735_InitR(LCD_HandleTypeDef* dev);
void ST7735_DisplayInit(LCD_HandleTypeDef* dev, const uint8_t *addr);

void ST7735_DisplayOn(LCD_HandleTypeDef* dev);
void ST7735_DisplayOff(LCD_HandleTypeDef* dev);

void ST7735_Select(LCD_HandleTypeDef* dev);
void ST7735_Unselect(LCD_HandleTypeDef* dev);
void ST7735_Reset(LCD_HandleTypeDef* dev);
void ST7735_WriteCommand(LCD_HandleTypeDef* dev, uint8_t cmd);
void ST7735_WriteData(LCD_HandleTypeDef* dev, uint8_t* buff, size_t buff_size);
void ST7735_WriteDataDMA(LCD_HandleTypeDef* dev, uint8_t* buff, size_t buff_size);
void ST7735_SetAddressWindow(LCD_HandleTypeDef* dev, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void ST7735_SetRotation(LCD_HandleTypeDef* dev, uint8_t m);

void ST7735_DrawPixel(LCD_HandleTypeDef* dev, uint16_t x, uint16_t y, uint16_t color);


void ST7735_DrawChar(LCD_HandleTypeDef* dev, char c);
uint8_t ST7735_WriteChar(LCD_HandleTypeDef* dev, char c);
//void ST7735_WriteString(LCD_HandleTypeDef* dev, uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor);
size_t ST7735_Write(LCD_HandleTypeDef* dev, uint8_t *buffer, size_t size);
size_t ST7735_Print(LCD_HandleTypeDef* dev, const char str[]);

void ST7735_FillRect(LCD_HandleTypeDef* dev, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ST7735_FillScreen(LCD_HandleTypeDef* dev, uint16_t color);
void ST7735_DrawImage(LCD_HandleTypeDef* dev, uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data);
void ST7735_DrawImageDMA(LCD_HandleTypeDef* dev, uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data);
void ST7735_InvertColors(LCD_HandleTypeDef* dev, bool invert);

void ST7735_SetFont(LCD_HandleTypeDef* dev, const GFXfont *f);
void ST7735_SetCursor(LCD_HandleTypeDef* dev, uint16_t x, uint16_t y);
void ST7735_SetTextColor(LCD_HandleTypeDef* dev, uint16_t newTextColor, uint16_t newBackgroundColor);
void ST7735_SetTextBGColor(LCD_HandleTypeDef* dev, uint16_t newColor);
uint8_t ST7735_GetTextWidth(LCD_HandleTypeDef* dev, const char str[]);
uint8_t ST7735_GetTextHeight(LCD_HandleTypeDef* dev, const char str[], bool includeBelowTheLine);

void ST7735_WriteLine(LCD_HandleTypeDef* dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void ST7735_DrawFastVLine(LCD_HandleTypeDef* dev, int16_t x, int16_t y, int16_t h, uint16_t color);
void ST7735_DrawFastHLine(LCD_HandleTypeDef* dev, int16_t x, int16_t y, int16_t w, uint16_t color);
//void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void ST7735_DrawLine(LCD_HandleTypeDef* dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void ST7735_DrawCircle(LCD_HandleTypeDef* dev, int16_t x0, int16_t y0, int16_t r, uint16_t color);
void ST7735_DrawCircleHelper(LCD_HandleTypeDef* dev, int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color);
void ST7735_FillCircleHelper(LCD_HandleTypeDef* dev, int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color);
void ST7735_FillCircle(LCD_HandleTypeDef* dev, int16_t x0, int16_t y0, int16_t r, uint16_t color);
void ST7735_DrawRect(LCD_HandleTypeDef* dev, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void ST7735_DrawRoundRect(LCD_HandleTypeDef* dev, int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color);
void ST7735_FillRoundRect(LCD_HandleTypeDef* dev, int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color);
void ST7735_DrawTriangle(LCD_HandleTypeDef* dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void ST7735_FillTriangle(LCD_HandleTypeDef* dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);




//====Canvas====

void ST7735_InitCanvas(LCD_HandleTypeDef* dev, LCD_CanvasHandleTypeDef* newCanvas, uint16_t w, uint16_t h);
void ST7735_DeleteCanvas(LCD_CanvasHandleTypeDef* can);

void ST7735_DrawPixelCanvas(LCD_CanvasHandleTypeDef* can, int16_t x, int16_t y, uint16_t color);
uint16_t ST7735_GetPixelCanvas(LCD_CanvasHandleTypeDef* can, int16_t x, int16_t y);
uint16_t ST7735_GetRawPixelCanvas(LCD_CanvasHandleTypeDef* can, int16_t x, int16_t y);

void ST7735_FillScreenCanvas(LCD_CanvasHandleTypeDef* can, uint16_t color);
void ST7735_FillRectCanvas(LCD_CanvasHandleTypeDef* can, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

void ST7735_SetFontCanvas(LCD_CanvasHandleTypeDef* can, const GFXfont *f);
void ST7735_SetCursorCanvas(LCD_CanvasHandleTypeDef* can, uint16_t x, uint16_t y);
void ST7735_SetTextColorCanvas(LCD_CanvasHandleTypeDef* can, uint16_t newTextColor, uint16_t newBackgroundColor);
void ST7735_SetTextBGColorCanvas(LCD_CanvasHandleTypeDef* can, uint16_t newColor);

void ST7735_DrawCharCanvas(LCD_CanvasHandleTypeDef* can, char c);
uint8_t ST7735_WriteCharCanvas(LCD_CanvasHandleTypeDef* can, char c);
size_t ST7735_WriteTextCanvas(LCD_CanvasHandleTypeDef* can, uint8_t *buffer, size_t size);
size_t ST7735_PrintTextCanvas(LCD_CanvasHandleTypeDef* can, const char str[]);

void ST7735_DrawFastRawVLineCanvas(LCD_CanvasHandleTypeDef* can, int16_t x, int16_t y, int16_t h, uint16_t color);
void ST7735_DrawFastVLineCanvas(LCD_CanvasHandleTypeDef* can, int16_t x, int16_t y, int16_t h, uint16_t color);
void ST7735_DrawFastRawHLineCanvas(LCD_CanvasHandleTypeDef* can, int16_t x, int16_t y, int16_t w, uint16_t color);
void ST7735_DrawFastHLineCanvas(LCD_CanvasHandleTypeDef* can, int16_t x, int16_t y, int16_t w, uint16_t color);
void ST7735_WriteLineCanvas(LCD_CanvasHandleTypeDef* can, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void ST7735_DrawLineCanvas(LCD_CanvasHandleTypeDef* can, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void ST7735_DrawCircleCanvas(LCD_CanvasHandleTypeDef* can, int16_t x0, int16_t y0, int16_t r, uint16_t color);
void ST7735_DrawCircleHelperCanvas(LCD_CanvasHandleTypeDef* can, int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color);
void ST7735_FillCircleHelperCanvas(LCD_CanvasHandleTypeDef* can, int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color);
void ST7735_FillCircleCanvas(LCD_CanvasHandleTypeDef* can, int16_t x0, int16_t y0, int16_t r, uint16_t color);
void ST7735_DrawRectCanvas(LCD_CanvasHandleTypeDef* can, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void ST7735_DrawRoundRectCanvas(LCD_CanvasHandleTypeDef* can, int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color);
void ST7735_FillRoundRectCanvas(LCD_CanvasHandleTypeDef* can, int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color);
void ST7735_DrawTriangleCanvas(LCD_CanvasHandleTypeDef* can, int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void ST7735_FillTriangleCanvas(LCD_CanvasHandleTypeDef* can, int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);

void ST7735_WriteCanvas(LCD_CanvasHandleTypeDef* can);
void ST7735_WriteCanvasDMA(LCD_CanvasHandleTypeDef* can);

#endif // __ST7735_H__
#ifdef __cplusplus
}
#endif












/*
#ifndef ST7735_H
#define ST7735_H

#include "main.h"
//#include <math.h>
#include <algorithm>
#include <cstring>
#include "gfxfont.h"



// some flags for initR() :(
#define INITB_NOOPTIONS 0xFF
#define INITR_GREENTAB 0x00
#define INITR_REDTAB 0x01
#define INITR_BLACKTAB 0x02
#define INITR_18GREENTAB INITR_GREENTAB
#define INITR_18REDTAB INITR_REDTAB
#define INITR_18BLACKTAB INITR_BLACKTAB
#define INITR_144GREENTAB 0x01
#define INITR_MINI160x80 0x04
#define INITR_HALLOWING 0x05
#define INITR_MINI160x80_PLUGIN 0x06


#define ST7735_TFTWIDTH_128 128  // for 1.44 and mini
#define ST7735_TFTWIDTH_80 80    // for mini
#define ST7735_TFTHEIGHT_128 128 // for 1.44" display
#define ST7735_TFTHEIGHT_160 160 // for 1.8" and mini display

#define ST_CMD_DELAY 0x80 // special signifier for command lists

#define ST77XX_NOP 0x00
#define ST77XX_SWRESET 0x01
#define ST77XX_RDDID 0x04
#define ST77XX_RDDST 0x09

#define ST77XX_SLPIN 0x10
#define ST77XX_SLPOUT 0x11
#define ST77XX_PTLON 0x12
#define ST77XX_NORON 0x13

#define ST77XX_INVOFF 0x20
#define ST77XX_INVON 0x21
#define ST77XX_DISPOFF 0x28
#define ST77XX_DISPON 0x29
#define ST77XX_CASET 0x2A
#define ST77XX_RASET 0x2B
#define ST77XX_RAMWR 0x2C
#define ST77XX_RAMRD 0x2E

#define ST77XX_PTLAR 0x30
#define ST77XX_TEOFF 0x34
#define ST77XX_TEON 0x35
#define ST77XX_MADCTL 0x36
#define ST77XX_COLMOD 0x3A

#define ST77XX_MADCTL_MY 0x80
#define ST77XX_MADCTL_MX 0x40
#define ST77XX_MADCTL_MV 0x20
#define ST77XX_MADCTL_ML 0x10
#define ST77XX_MADCTL_RGB 0x00

#define ST77XX_RDID1 0xDA
#define ST77XX_RDID2 0xDB
#define ST77XX_RDID3 0xDC
#define ST77XX_RDID4 0xDD

// Some ready-made 16-bit ('565') color settings:
#define ST77XX_BLACK 0x0000
#define ST77XX_WHITE 0xFFFF
#define ST77XX_RED 0xF800
#define ST77XX_GREEN 0x07E0
#define ST77XX_BLUE 0x001F
#define ST77XX_CYAN 0x07FF
#define ST77XX_MAGENTA 0xF81F
#define ST77XX_YELLOW 0xFFE0
#define ST77XX_ORANGE 0xFC00


// Some register settings
#define ST7735_MADCTL_BGR 0x08
#define ST7735_MADCTL_MH 0x04

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR 0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1 0xC0
#define ST7735_PWCTR2 0xC1
#define ST7735_PWCTR3 0xC2
#define ST7735_PWCTR4 0xC3
#define ST7735_PWCTR5 0xC4
#define ST7735_VMCTR1 0xC5

#define ST7735_PWCTR6 0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

// Some ready-made 16-bit ('565') color settings:
#define ST7735_BLACK ST77XX_BLACK
#define ST7735_WHITE ST77XX_WHITE
#define ST7735_RED ST77XX_RED
#define ST7735_GREEN ST77XX_GREEN
#define ST7735_BLUE ST77XX_BLUE
#define ST7735_CYAN ST77XX_CYAN
#define ST7735_MAGENTA ST77XX_MAGENTA
#define ST7735_YELLOW ST77XX_YELLOW
#define ST7735_ORANGE ST77XX_ORANGE


class ST7735 {
public:
	ST7735();
	~ST7735();
	void init(uint16_t w, uint16_t h, SPI_HandleTypeDef* spi, GPIO_TypeDef* csPort, uint16_t csPin, GPIO_TypeDef* dcPort, uint16_t dcPin,
			GPIO_TypeDef* rstPort, uint16_t rstPin, uint8_t rotation, uint8_t options = INITB_NOOPTIONS);
	void reset();
	void start();

	void setRotation(uint8_t r);
	void enableDisplay(bool enable);
	void invertColors(bool invert);

	int16_t getScreenWidth();
	int16_t getScreenHeight();

	void drawPixel(int16_t x, int16_t y, uint16_t color);
	void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

	void setFont(const GFXfont *f);
	void setTextColor(uint16_t color);
	size_t getTextWidth(const char str[]);
	void setCursor(int16_t x, int16_t y);
	void wrapText(bool val);
	virtual size_t write(uint8_t);
	size_t write(uint8_t *buffer, size_t size);
	size_t print(const char str[]);



private:
	void initB();
	void initR(uint8_t options = INITR_MINI160x80_PLUGIN);
	void displayInit(const uint8_t *addr);
	void select();
	void unselect();
	void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);

	void writeCommand(uint8_t cmd);
	void writeData(uint8_t* buff, size_t buff_size);

	void writeColor(uint16_t color, uint32_t len);
	void writeFillRectPreclipped(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

	void drawChar(int16_t x, int16_t y, uint8_t c, uint16_t color);

	uint8_t	_tabColor;

	SPI_HandleTypeDef* 	_hspi;
	GPIO_TypeDef*		_csPort;
	uint16_t			_csPin;
	GPIO_TypeDef*		_dcPort;
	uint16_t			_dcPin;
	GPIO_TypeDef*		_rstPort;
	uint16_t			_rstPin;

	uint8_t				_displayType;

	int16_t 	_width;       ///< Display width as modified by current rotation
	int16_t		_normwidth;		//Display width in normal orientation
	int16_t 	_height;      ///< Display height as modified by current rotation
	int16_t		_normheight;	//Display height in normal orientation
	int16_t 	cursor_x;     ///< x location to start print()ing text
	int16_t 	cursor_y;     ///< y location to start print()ing text
	uint8_t 	_rotation;     ///< Display rotation (0 thru 3)
	uint8_t 	_colstart;   ///< Some displays need this changed to offset
	uint8_t 	_rowstart;       ///< Some displays need this changed to offset
	uint8_t 	_xstart;
	uint8_t 	_ystart;

	uint8_t		_invertOnCommand = ST77XX_INVON;
	uint8_t 	_invertOffCommand = ST77XX_INVOFF;

	GFXfont* 	_curFont;
	uint16_t	_textColor;
	int16_t		_cursorX;
	int16_t		_cursorY;
	bool		_wrapText;

};


#endif // __ST7735_H__


*/
//===============================================






