#ifdef __cplusplus
extern "C" {
#endif

//********************************************************************
//
//				General Function Definitions
//
//********************************************************************

void xLCDInit(void);

void xLCDClear(int16_t color);
void xLCDContrast(uint8_t setting);

/*-----------------------------------------------------------------*/

// xLCDPrintf_P( x, y, font, foregroundColor, backgroundColor, PSTR("\r\nMessage %u %u %u"), var1, var2, var2);

void xLCDPrintf(  int16_t x, int16_t y, uint8_t font, int16_t fColor, int16_t bColor, char * format, ...); // from SRAM
void xLCDPrintf_P(int16_t x, int16_t y, uint8_t font, int16_t fColor, int16_t bColor, PGM_P format, ...); // from PRGMEM

void xLCDPrint(   int16_t x, int16_t y, uint8_t font, int16_t fColor, int16_t bColor, char * str); // from SRAM
void xLCDPrint_P( int16_t x, int16_t y, uint8_t font, int16_t fColor, int16_t bColor, PGM_P str); // from PRGMEM

void xLCDDrawPixel(int16_t color, uint8_t x, uint8_t y);
void xLCDDrawLine( int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t color);
void xLCDDrawRect( int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t fill, int16_t color);
void xLCDDrawCircle (int16_t xCenter, int16_t yCenter, int16_t radius, int16_t circleType, int16_t color);

//********************************************************************
//
//					Data Structure Definitions
//********************************************************************
typedef struct
{
	unsigned char red;
	unsigned char green;
	unsigned char blue;
} RGBColor;


//********************************************************************
//
//					Controller Type Definition
//
//********************************************************************

//#define	PHILLIPS
#define	EPSON

//********************************************************************
//
//					LCD Dimension Definitions
//
//********************************************************************
#define ROW_LENGTH	132
#define COL_HEIGHT	132
#define ENDCOL      130

//********************************************************************
//
//					EPSON Controller Definitions
//
//********************************************************************

#define DISON   0xAF      // Display on
#define DISOFF  0xAE      // Display off
#define DISNOR  0xA6      // Normal display
#define DISINV  0xA7      // Inverse display
#define COMSCN  0xBB      // Common scan direction
#define DISCTL  0xCA      // Display control
#define SLPIN   0x95      // Sleep in
#define SLPOUT  0x94      // Sleep out
#define PASET   0x75      // Page address set
#define CASET   0x15      // Column address set
#define DATCTL  0xBC      // Data scan direction, etc.
#define RGBSET8 0xCE      // 256-color position set
#define RAMWR   0x5C      // Writing to memory
#define RAMRD   0x5D      // Reading from memory
#define PTLIN   0xA8      // Partial display in
#define PTLOUT  0xA9      // Partial display out
#define RMWIN   0xE0      // Read and modify write
#define RMWOUT  0xEE      // End
#define ASCSET  0xAA      // Area scroll set
#define SCSTART 0xAB      // Scroll start set
#define OSCON   0xD1      // Internal oscillation on
#define OSCOFF  0xD2      // Internal oscillation off
#define PWRCTR  0x20      // Power control
#define VOLCTR  0x81      // Electronic volume control
#define VOLUP   0xD6      // Increment electronic control by 1
#define VOLDOWN 0xD7      // Decrement electronic control by 1
#define TMPGRD  0x82      // Temperature gradient set
#define EPCTIN  0xCD      // Control EEPROM
#define EPCOUT  0xCC      // Cancel EEPROM control
#define EPMWR   0xFC      // Write into EEPROM
#define EPMRD   0xFD      // Read from EEPROM
#define EPSRRD1 0x7C      // Read register 1
#define EPSRRD2 0x7D      // Read register 2
#define NOP     0x25      // NOP instruction


//*************************************************************************************
//	LCD Include File for Philips PCF8833 STN RGB- 132x132x3 Driver
//
//		Taken from Philips data sheet Feb 14, 2003
//*************************************************************************************
//*	I changed them to P_ for Philips
//*	many of these commands are not used but I wanted them all listed in case
//*	anyone wants to write more features
//	Philips PCF8833 LCD controller command codes
#define	P_NOP			0x00	// nop
#define	P_SWRESET		0x01	// software reset
#define	P_BSTROFF		0x02	// booster voltage OFF
#define	P_BSTRON		0x03	// booster voltage ON
#define	P_RDDIDIF		0x04	// read display identification
#define	P_RDDST			0x09	// read display status
#define	P_SLEEPIN		0x10	// sleep in
#define	P_SLEEPOUT		0x11	// sleep out
#define	P_PTLON			0x12	// partial display mode
#define	P_NORON			0x13	// display normal mode
#define	P_INVOFF		0x20	// inversion OFF
#define	P_INVON			0x21	// inversion ON
#define	P_DALO			0x22	// all pixel OFF
#define	P_DAL			0x23	// all pixel ON
#define	P_SETCON		0x25	// write contrast
#define	P_DISPOFF		0x28	// display OFF
#define	P_DISPON		0x29	// display ON
#define	P_CASET			0x2A	// column address set
#define	P_PASET			0x2B	// page address set
#define	P_RAMWR			0x2C	// memory write
#define	P_RGBSET		0x2D	// colour set
#define	P_PTLAR			0x30	// partial area
#define	P_VSCRDEF		0x33	// vertical scrolling definition
#define	P_TEOFF			0x34	// test mode
#define	P_TEON			0x35	// test mode
#define	P_MADCTL		0x36	// memory access control
#define	P_SEP			0x37	// vertical scrolling start address
#define	P_IDMOFF		0x38	// idle mode OFF
#define	P_IDMON			0x39	// idle mode ON
#define	P_COLMOD		0x3A	// interface pixel format
#define	P_SETVOP		0xB0	// set Vop
#define	P_BRS			0xB4	// bottom row swap
#define	P_TRS			0xB6	// top row swap
#define	P_DISCTR		0xB9	// display control
#define	P_DOR			0xBA	// data order
#define	P_TCDFE			0xBD	// enable/disable DF temperature compensation
#define	P_TCVOPE		0xBF	// enable/disable Vop temp comp
#define	P_EC			0xC0	// internal or external oscillator
#define	P_SETMUL		0xC2	// set multiplication factor
#define	P_TCVOPAB		0xC3	// set TCVOP slopes A and B
#define	P_TCVOPCD		0xC4	// set TCVOP slopes c and d
#define	P_TCDF			0xC5	// set divider frequency
#define	P_DF8COLOR		0xC6	// set divider frequency 8-color mode
#define	P_SETBS			0xC7	// set bias system
#define	P_RDTEMP		0xC8	// temperature read back
#define	P_NLI			0xC9	// n-line inversion
#define	P_RDID1			0xDA	// read ID1
#define	P_RDID2			0xDB	// read ID2
#define	P_RDID3			0xDC	// read ID3


//*******************************************************
//				12-Bit Color Definitions
//*******************************************************
#define WHITE	0xFFF
#define BLACK	0x000
#define RED	    0xF00
#define	GREEN	0x0F0
#define BLUE	0x00F
#define CYAN	0x0FF
#define MAGENTA	0xF0F
#define YELLOW	0xFF0
#define BROWN	0xB22
#define ORANGE	0xFA0
#define PINK	0xF6A


//*******************************************************
//                Circle Definitions
//*******************************************************
#define FULLCIRCLE    1
#define OPENSOUTH     2
#define OPENNORTH     3
#define OPENEAST      4
#define OPENWEST      5
#define OPENNORTHEAST 6
#define OPENNORTHWEST 7
#define OPENSOUTHEAST 8
#define OPENSOUTHWEST 9


//*******************************************************
//				Button Pin Definitions
//*******************************************************
#define	kSwitch1_PIN	PD3
#define	kSwitch2_PIN	PD4
#define	kSwitch3_PIN	PD5


//*******************************************************
//				Font Definitions
//*******************************************************
#define	BOLD	1
#define	NORMAL	0

#ifdef __cplusplus
}
#endif
