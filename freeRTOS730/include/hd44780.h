/*-----------------------------------------------------------------------*/
/* EZ-LCD - Generic control module include/configuration file            */
/* http://elm-chan.org/fsw/ezlcd/00index_e.html                          */
/*-----------------------------------------------------------------------*/

#ifndef _EZLCD_DEFINED
#define _EZLCD_DEFINED

/*--------------------------------------------------*/
/* Configuration Options                            */
/*--------------------------------------------------*/

//#define FreetronicsLCD		// Define whether we're using (modified) Freetronics LCD (or PololuLCD) connections.
#define PololuLCD				// Define for Pololu (modified) display.

								/* Adjust the size of the display to suit hardware */
#define portLCD_BUFFER	80		// Define the size of the LCD buffer. Lines x Columns. -> Retrograde Project needs 80 else 32

#define _LCD_ROWS		4		/* Number of Rows (1,2 or 4). -> Retrograde Project uses  4 else use  2 */
#define _LCD_COLS		20		/* Number of Columns (8..40). -> Retrograde Project uses 20 else use 16 */

#define IF_BUS			4		/* define whether the interface bus is 4 bit or 8 bit */

#define _USE_CURSOR		0		/* 1:Enable lcd_Cursor function */
#define _USE_CGRAM		0		/* 1:Enable lcd_SetCG function */

#define	_USE_FUEL		0		/* 1:Enable lcd_PutFuel function (_USE_CGRAM must be 1) */

#define	_USE_BAR		0		/* 1:Enable lcd_PutBar function (_USE_CGRAM must be 1) */
#define	_MAX_BAR		255		/* Maximum value for lcd_PutBar function */

#define	_USE_POINT		0		/* 1:Enable lcd_PutPoint function (_USE_CGRAM must be 1) */
#define	_MAX_POINT		255		/* Maximum value for lcd_PutPoint function */

#define	_BASE_GRAPH		0		/* Common user character used by lcd_PutBar/lcd_PutPoint function (2 chars from this) */


/*--------------------------------------------------*/
/* API declarations                                */
/*--------------------------------------------------*/

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void lcd_Init(void);
void lcd_Close(void);
void lcd_Locate (uint8_t, uint8_t);
void lcd_Putc (uint8_t); 						// '\f'  Clear Screen and Return Home to (0,0)

void lcd_Printf( const uint8_t * format, ...);
void lcd_Printf_P(PGM_P format, ...);
void lcd_Print( uint8_t * str);
void lcd_Print_P(PGM_P str);

#if _USE_CURSOR
void lcd_Cursor (uint8_t);
#endif

#if _USE_CGRAM
void lcd_SetCG (uint8_t, uint8_t, const uint8_t*);
#endif

#if _USE_FUEL
void lcd_PutFuel (int8_t, uint8_t);
#endif

#if _USE_BAR
void lcd_PutBar (uint16_t, uint8_t, uint8_t);
#endif

#if _USE_POINT
void lcd_PutPoint (uint16_t, uint8_t, uint8_t);
#endif

#ifdef __cplusplus
}
#endif

#define CSR_OFF		0
#define CSR_BLOCK	1
#define CSR_UNDER	2


#endif	/* #ifndef _EZLCD */
