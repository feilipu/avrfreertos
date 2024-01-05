/*------------------------------------------------------------------------/
/  EZ-LCD - Generic control module for HD44780 LCDC
/-------------------------------------------------------------------------/
/
/  Copyright (C) 2010, ChaN, all right reserved.
/
/  http://elm-chan.org/fsw/ezlcd/00index_e.html
/
/ * This software is a free software and there is NO WARRANTY.
/ * No restriction on use. You can use, modify and redistribute it for
/   personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
/ * Redistributions of source code must retain the above copyright notice.
/
/-------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "FreeRTOS.h"

#include "hd44780.h"

/*---------------------------------------------------------------------------------*/
/* Platform dependent macros and functions needed to be modified to suit hardware  */
/*---------------------------------------------------------------------------------*/

#ifdef portHD44780_LCD

/* Characteristics of LCD module  */
#define	LCD_ETIME_1	1530		/* Execution time of Clear Display command [us] */
#define	LCD_ETIME_2	43			/* Execution time of other command and data write [us] */
#define	LCD_DLF		2.0			/* Delay factor (>=2.0) */

/* Bus controls */

#if defined(_MEGA_) && defined(_FreetronicsLCD)
// assume we're using an Arduino Mega2560 with ATmega2560
/*
 * THIS SETUP IS FOR THE FREETRONICS 16x2 LCD that has been modified
 * to move D4 pin to D2, to avoid conflict with the SD _SS_ Pin for EtherMega.
 */

//  E  - PH6
//  RS - PH5

//#define	IF_INIT()		{ DDRH |= 0x78; DDRE |= _BV(DDE3); DDRG |= _BV(DDG5); E1_HIGH(); RS_HIGH(); OUT_DATA(0xFF); }	/* (Normal) Initialise control port */
#define	IF_INIT()		{ DDRH |= 0x78; DDRE |= (_BV(DDE4) | _BV(DDE3)); E1_HIGH(); RS_HIGH(); OUT_DATA(0xFF); }	/* (D2 Mod) Initialise control port */
#define	IF_DLY60nS()	{__asm__ __volatile__("nop\n\t"::);}	/* Delay >=60ns */
#define	IF_DLY450nS()	{_delay_us(1);}							/* Delay >=450ns@3V, >=250ns@5V */

#define E1_HIGH() 	PORTH|= _BV(PH6)/* Set E/E1 high */
#define E1_LOW()  	PORTH&=~_BV(PH6)/* Set E/E1 low */
#define	RS_HIGH() 	PORTH|= _BV(PH5)/* Set RS high */
#define	RS_LOW()  	PORTH&=~_BV(PH5)/* Set RS low */

//  D7 & D3 - PH4
//  D6 & D2 - PH3
//  D5 & D1 - PE3
//  D4 & D0 - PE4  Pin D2 Rather than pin D4 which is PG5.

#define	DATA7(d) { if (d & _BV(7)) PORTH |= _BV(PH4); else PORTH &= ~_BV(PH4); }
#define	DATA6(d) { if (d & _BV(6)) PORTH |= _BV(PH3); else PORTH &= ~_BV(PH3); }
#define	DATA5(d) { if (d & _BV(5)) PORTE |= _BV(PE3); else PORTE &= ~_BV(PE3); }
//#define	DATA4(d) { if (d & _BV(4)) PORTG |= _BV(PG5); else PORTG &= ~_BV(PG5); } // normal Arduino Mega
#define	DATA4(d) { if (d & _BV(4)) PORTE |= _BV(PE4); else PORTE &= ~_BV(PE4); }

#define	OUT_DATA(d)  { DATA7(d); DATA6(d); DATA5(d); DATA4(d); }


#elif defined(_GOLDILOCKS_) && defined(_FreetronicsLCD)// assume we're using a Goldilocks with ATmega1284p
#define	IF_BUS			4										/* Bus width (4 or 8) */

//  E  - PB3
//  RS - PB2

//#define	IF_INIT()		{ DDRB |= 0x0C; DDRD |= 0xF0; E1_HIGH(); RS_HIGH(); OUT_DATA(0xF0); }	/* Normal Initialise control port */
#define	IF_INIT()		{ DDRB |= 0x0C; DDRD |= 0xE4; E1_HIGH(); RS_HIGH(); OUT_DATA(0xF0); }	/* (D2 Mod) Initialise control port */
#define	IF_DLY60nS()	{__asm__ __volatile__("nop\n\t"::);}	/* Delay >=60ns */
#define	IF_DLY450nS()	{_delay_us(1);}							/* Delay >=450ns@3V, >=250ns@5V */

#define E1_HIGH() 	PORTB|= _BV(PB3)/* Set E/E1 high */
#define E1_LOW()  	PORTB&=~_BV(PB3)/* Set E/E1 low */
#define	RS_HIGH() 	PORTB|= _BV(PB2)/* Set RS high */
#define	RS_LOW()  	PORTB&=~_BV(PB2)/* Set RS low */

//  D7 & D3 - PD7
//  D6 & D2 - PD6
//  D5 & D1 - PD5
//  D4 & D0 - PD2  Pin D2 Rather than Pin D4.

#define	DATA7(d) { if (d & _BV(7)) PORTD |= _BV(PD7); else PORTD &= ~_BV(PD7); }
#define	DATA6(d) { if (d & _BV(6)) PORTD |= _BV(PD6); else PORTD &= ~_BV(PD6); }
#define	DATA5(d) { if (d & _BV(5)) PORTD |= _BV(PD5); else PORTD &= ~_BV(PD5); }
//#define	DATA4(d) { if (d & _BV(4)) PORTD |= _BV(PD4); else PORTD &= ~_BV(PD4); } // normal unmodded Arduino Uno
#define	DATA4(d) { if (d & _BV(4)) PORTD |= _BV(PD2); else PORTD &= ~_BV(PD2); }

#define	OUT_DATA(d)  { DATA7(d); DATA6(d); DATA5(d); DATA4(d); }

// OR simply do this below if using Pin D4 (with normal Goldilocks)
//#define	OUT_DATA(d)  { PORTD = (PORTD & 0x0F) | (d & 0xF0); }


#elif defined(_UNO_) && defined(_FreetronicsLCD)// assume we're using an Arduino with ATmega328p
#define	IF_BUS			4										/* Bus width (4 or 8) */

//  E  - PB1
//  RS - PB0

//#define	IF_INIT()		{ DDRB |= 0x03; DDRD |= 0xF0; E1_HIGH(); RS_HIGH(); OUT_DATA(0xF0); }	/* Normal Initialise control port */
#define	IF_INIT()		{ DDRB |= 0x03; DDRD |= 0xE4; E1_HIGH(); RS_HIGH(); OUT_DATA(0xF0); }	/* (D2 Mod) Initialise control port */
#define	IF_DLY60nS()	{__asm__ __volatile__("nop\n\t"::);}	/* Delay >=60ns */
#define	IF_DLY450nS()	{_delay_us(1);}							/* Delay >=450ns@3V, >=250ns@5V */

#define E1_HIGH() 	PORTB|= _BV(PB1)/* Set E/E1 high */
#define E1_LOW()  	PORTB&=~_BV(PB1)/* Set E/E1 low */
#define	RS_HIGH() 	PORTB|= _BV(PB0)/* Set RS high */
#define	RS_LOW()  	PORTB&=~_BV(PB0)/* Set RS low */

//  D7 & D3 - PD7
//  D6 & D2 - PD6
//  D5 & D1 - PD5
//  D4 & D0 - PD2  Pin D2 Rather than Pin D4.

#define	DATA7(d) { if (d & _BV(7)) PORTD |= _BV(PD7); else PORTD &= ~_BV(PD7); }
#define	DATA6(d) { if (d & _BV(6)) PORTD |= _BV(PD6); else PORTD &= ~_BV(PD6); }
#define	DATA5(d) { if (d & _BV(5)) PORTD |= _BV(PD5); else PORTD &= ~_BV(PD5); }
//#define	DATA4(d) { if (d & _BV(4)) PORTD |= _BV(PD4); else PORTD &= ~_BV(PD4); } // normal unmodded Arduino Uno
#define	DATA4(d) { if (d & _BV(4)) PORTD |= _BV(PD2); else PORTD &= ~_BV(PD2); }

#define	OUT_DATA(d)  { DATA7(d); DATA6(d); DATA5(d); DATA4(d); }

// OR simply do this below if using Pin D4 (on normal Arduino Uno)
//#define	OUT_DATA(d)  { PORTD = (PORTD & 0x0F) | (d & 0xF0); }


#elif defined(_UNO_) && defined(_PololuLCD) // assume we're using an Arduino with ATmega328p

// On the MODIFIED Orangutan LV-168 and 3pi robot, the LCD control lines are split between ports B and D.
// See posts on modifying to avoid the Timer1 16 bit PWM channel for the Retrograde clock.

// Note that the EZ-LCD module does not support R/W signal so that set the R/W signal low if it is wired to the port.

// LCD_RW				PORTB0		// PB0 Set to LOW
// LCD_E				PORTD4		// PD4
// LCD_RS				PORTD2		// PD2

#define	IF_INIT()		{ DDRB |= 0x39; DDRD |= 0x94; E1_HIGH(); RS_HIGH(); RW_LOW(); OUT_DATA(0xF0); }	/* Initialise control port */
#define	IF_DLY60nS()	{__asm__ __volatile__("nop\n\t"::);}	/* Delay >=60ns */
#define	IF_DLY450nS()	{_delay_us(1);}							/* Delay >=450ns@3V, >=250ns@5V */

#define RW_HIGH() 	PORTB|= _BV(PB0)/* Set RW high */
#define RW_LOW()  	PORTB&=~_BV(PB0)/* Set RW low */
#define E1_HIGH() 	PORTD|= _BV(PD4)/* Set E/E1 high */
#define E1_LOW()  	PORTD&=~_BV(PD4)/* Set E/E1 low */
#define	RS_HIGH() 	PORTD|= _BV(PD2)/* Set RS high */
#define	RS_LOW()  	PORTD&=~_BV(PD2)/* Set RS low */

// LCD_DB7				PORTD7		// PD7
// LCD_DB6				PORTB5		// PB5
// LCD_DB5				PORTB4		// PB4
// LCD_DB4				PORTB3		// PB3 : Pololu was PB1, but I modified to avoid the Timer1 PWM pin(s).

#define	DATA7(d) { if (d & _BV(7)) PORTD |= _BV(PD7); else PORTD &= ~_BV(PD7); }
#define	DATA6(d) { if (d & _BV(6)) PORTB |= _BV(PB5); else PORTB &= ~_BV(PB5); }
#define	DATA5(d) { if (d & _BV(5)) PORTB |= _BV(PB4); else PORTB &= ~_BV(PB4); }
#define	DATA4(d) { if (d & _BV(4)) PORTB |= _BV(PB3); else PORTB &= ~_BV(PB3); }

#define	OUT_DATA(d)  { DATA7(d); DATA6(d); DATA5(d); DATA4(d); }

#endif


/*-------------------------------------------------------------------------*/

static uint8_t *LCDWorkBuffer; // create a working buffer pointer, to later be pvPortMalloc() on the heap.

/*-------------------------------------------------------------------------*/

#if _LCD_ROWS >= 2 || _LCD_COLS > 8
 #define LCD_IF_2ROW 8		/* 2-row cfg. */
 #if _LCD_ROWS == 1
  #define LCD_IF_SPLIT 1	/* Half split row */
 #else
  #define LCD_IF_SPLIT 0	/* Direct row */
 #endif
#else
 #define LCD_IF_2ROW 0		/* 1-row cfg. */
 #define LCD_IF_SPLIT 0		/* Direct row */
#endif

#if _LCD_ROWS == 4 && _LCD_COLS <= 20
 #define LCD_IF_ALTROW	1	/* Alternate row layout */
#else
 #define LCD_IF_ALTROW	0	/* Incremental row layout */
#endif

#if _LCD_ROWS == 4 && _LCD_COLS > 20
 #define LCD_IF_DUAL 	1	/* Dual controller */
#else
 #define LCD_IF_DUAL 	0	/* Single controller */
#endif

#define	LCD_DT1		((uint16_t)(LCD_ETIME_1 * LCD_DLF))
#define	LCD_DT2		((uint16_t)(LCD_ETIME_2 * LCD_DLF))



static uint8_t Row, Column;	/* Current cursor position */

#if _USE_CURSOR
static uint8_t Csr;	/* Current cursor state */
#endif


/*----------------------------------------------*/
/* Write a byte to the LCD controller           */
/*----------------------------------------------*/

static void lcd_Write ( uint8_t reg, uint8_t data );

static void lcd_Write (
	uint8_t reg,	/*	b0:command(0)/data(1), b2-b1:E1(2)/E2(1)/both(0)(don't care on single controller),
						b3:write high nibble only(don't care on 8-bit I/F) */
	uint8_t data	/*	Byte to be written */
)
{
	if (reg & 1)	/* Select register */
		RS_HIGH();
	else
		RS_LOW();
	IF_DLY60nS();

#if IF_BUS == 4
	if (!(reg & 8)) { // write both nibbles
		OUT_DATA(data);
#if LCD_IF_DUAL
		if (!(reg & 2)) E1_HIGH();
		if (!(reg & 4)) E2_HIGH();
		IF_DLY450nS();
		E1_LOW();
		E2_LOW();
#else
		E1_HIGH();
		IF_DLY450nS();
		E1_LOW();
#endif
		data <<= 4;
	}
#endif

	OUT_DATA(data);

#if LCD_IF_DUAL
	if (!(reg & 2)) E1_HIGH();
	if (!(reg & 4)) E2_HIGH();
	IF_DLY450nS();
	E1_LOW();
	E2_LOW();
#else
	E1_HIGH();
	IF_DLY450nS();
	E1_LOW();
#endif

	_delay_us(LCD_DT2);	/* Always use timer */

	OUT_DATA(0xF0);
}


/*-----------------------------------------------------------------------*/
/* Initialise LCD module                                                 */
/*-----------------------------------------------------------------------*/

void lcd_Init(void)
{
	uint8_t d;
	// create a working buffer for vsnprintf on the heap (so we can use extended RAM, if available).
	if(LCDWorkBuffer == NULL) // if there is no LCDWorkBuffer allocated (pointer is NULL), then allocate buffer.
		if( !(LCDWorkBuffer = (uint8_t *)pvPortMalloc(sizeof(uint8_t) * portLCD_BUFFER)))
			return;

	IF_INIT();
	_delay_ms(40);
	lcd_Write(8, 0x30);
	_delay_us(4100);
	lcd_Write(8, 0x30);
	_delay_us(4100);
	lcd_Write(8, 0x30); // put in an extra (undocumented) initialise cycle, as it seems to work the LCD reliably.
	_delay_us(100);
	lcd_Write(8, 0x30);

#if IF_BUS == 4
	d = 0x20 + LCD_IF_2ROW;
	lcd_Write(0, d);
#else
	d = 0x30 + LCD_IF_2ROW;
	lcd_Write(8, d);
#endif

	lcd_Write(0, 0x08);
	lcd_Write(0, 0x01);
	_delay_us(LCD_DT1);
	lcd_Write(0, 0x06);
	lcd_Write(0, 0x0C);

	Row = Column = 0;
#if _USE_CURSOR
	Csr = 0;
#endif
	return;
}

void lcd_Close(void)
{
	vPortFree (LCDWorkBuffer);
	return;
}

/*-----------------------------------------------------------------------*/
/* Set cursor position                                                   */
/*-----------------------------------------------------------------------*/

void lcd_Locate (
	uint8_t row,	/* Cursor row position (0.._LCD_ROWS-1) */
	uint8_t col		/* Cursor column position (0.._LCD_COLS-1) */
)
{
	Row = row; Column = col;

	if (row < _LCD_ROWS && col < _LCD_COLS) {
		if (_LCD_COLS >= 2 && (row & 1)) col += 0x40;
		if (LCD_IF_SPLIT && col >= _LCD_COLS / 2) col += 0x40 - _LCD_COLS / 2;
		if (LCD_IF_ALTROW && (row & 2)) col += _LCD_COLS;
		col |= 0x80;
	} else {
		col = 0x0C;
	}

#if LCD_IF_DUAL
	if (_USE_CURSOR && !(row &= 2)) row |= 4;
	lcd_Write(row, col);
#if _USE_CURSOR
	if (col != 0x0C) lcd_Write(row, Csr | 0x0C);
	row ^= 6;
	lcd_Write(row, 0x0C);
#endif
#else
	lcd_Write(0, col);
#if _USE_CURSOR
	if (col != 0x0C) lcd_Write(0, Csr | 0x0C);
#endif
#endif
}



/*-----------------------------------------------------------------------*/
/* Put a character                                                       */
/*-----------------------------------------------------------------------*/

void lcd_Putc (
	uint8_t chr
)
{
	if (Row >= _LCD_ROWS) return;

	if (chr == '\r') {	/* Carriage Return */
		lcd_Locate(Row, 0);
		return;
	}
	if (chr == '\n') {	/* Line Feed */
		lcd_Locate(Row + 1, 0);
		return;
	}
	if (chr == '\f') {	/* Clear Screen and Return Home */
		lcd_Write(0, 0x01);
		_delay_us(LCD_DT1);
		lcd_Locate(0, 0);
		return;
	}

	if (Column >= _LCD_COLS) return;

	lcd_Write((LCD_IF_DUAL && Row >= 2) ? 3 : 5, chr);
	Column++;

	if (LCD_IF_SPLIT && Column == _LCD_COLS / 2)
		lcd_Write(0, 0x40);

	if (Column >= _LCD_COLS)
		lcd_Locate(Row + 1, 0);
}



/*-----------------------------------------------------------------------*/
/* Set cursor form                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_CURSOR
void lcd_Cursor (
	uint8_t stat	/* 0:off, 1:blinking block, 2:under-line */
)
{
	Csr = stat & 3;
	lcd_Locate(Row, Column);
}
#endif



/*-----------------------------------------------------------------------*/
/* Register user character pattern                                       */
/*-----------------------------------------------------------------------*/

#if _USE_CGRAM
void lcd_SetCG (
	uint8_t chr,		/* Character code to be registered (0..7) */
	uint8_t n,			/* Number of characters to register */
	const uint8_t* p	/* Pointer to the character pattern (8 * n bytes) */
)
{
	lcd_Write(0, 0x40 | chr * 8);
	n *= 8;
	do
		lcd_Write(1, *p++);
	while (--n);

	lcd_Locate(Row, Column);
}
#endif



/*-----------------------------------------------------------------------*/
/* Put a fuel indicator                                                  */
/*-----------------------------------------------------------------------*/

#if _USE_FUEL
void lcd_PutFuel (
	int8_t val,		/* Fuel level (-1:plugged, 0:empty cell, ..., 5:full cell) */
	uint8_t chr		/* User character to use */
)
{
	static const uint8_t plg[8] = {10,10,31,31,14,4,7,0};
	uint8_t gfx[8], d, *p;
	int8_t i;


	if (val >= 0) {		/* Cell (0..5) */
		p = &gfx[8];
		*(--p) = 0; *(--p) = 0x1F;
		for (i = 1; i <= 5; i++) {
			d = 0x1F;
			if (val < i) d = (i == 5) ? 0x1B : 0x11;
			*(--p) = d;
		}
		*(--p) = 0x0E;
	} else {			/* Plug (-1) */
		p = (uint8_t*)plg;
	}
	lcd_SetCG(chr, 1, p);
	lcd_Putc(chr);
}


#endif



/*-----------------------------------------------------------------------*/
/* Draw bargraph                                                         */
/*-----------------------------------------------------------------------*/

#if _USE_BAR
void lcd_PutBar (
	uint16_t val,	/* Bar length (0 to _MAX_BAR represents bar length from left end) */
	uint8_t width,	/* Display area (number of chars from cursor position) */
	uint8_t chr		/* User character code (2 chars used from this) */
)
{
	static const uint8_t ptn[] = {
		0xE0, 0xE0, 0xE0, 0xC0, 0xC0, 0xC0, 0x80, 0,
		0xF0, 0xE0, 0xE0, 0xE0, 0xC0, 0xC0, 0xC0, 0,
		0xF0, 0xF0, 0xE0, 0xE0, 0xE0, 0xC0, 0xC0, 0
	};
	const uint8_t *pp;
	uint16_t n, m, s, gi;
	uint8_t gfx[16];


	for (n = 0; n < 16; n++)		/* Register common pattern (space/fill) */
		gfx[n] = n < 7 ? 0 : 0xFF;
	lcd_SetCG(_BASE_GRAPH, 2, gfx);

	/* Draw edge pattern into gfx[] */
	val = (uint32_t)val * (width * 18) / (_MAX_BAR + 1);
	pp = &ptn[(val % 3) * 8];		/* Get edge pattern */
	s = val / 3 % 6;				/* Bit shift */
	for (n = 0; n < 7; n++) {		/* Draw edge pattern into the pattern buffer */
		m = (*pp++ | 0xFF00) >> s;
		gfx[n] = m;
		gfx[n + 8] = m >> 6;
	}

	/* Put graphic pattern into the LCD module */
	gi = val / 18;						/* Indicator start position */
	for (n = 1; n <= width; n++) {		/* Draw each location in the bargraph */
		if (n == gi) {					/* When edge pattern is exist at the location */
			m = chr + 1;				/* A edge pattern */
		} else {
			if (n == gi + 1) {
				lcd_SetCG(chr, 2, gfx);	/* Register edge pattern */
				m = chr;
			} else {
				m = (n >= gi) ? _BASE_GRAPH : _BASE_GRAPH + 1;	/* A space or fill */
			}
		}
		lcd_Putc(m);					/* Put the character into the LCD */
	}
}
#endif



/*-----------------------------------------------------------------------*/
/* Draw point indicator                                                  */
/*-----------------------------------------------------------------------*/

#if _USE_POINT
void lcd_PutPoint (
	uint16_t val,	/* Dot position (0 to _MAX_POINT represents left end to write end) */
	uint8_t width,	/* Display area (number of chars from cursor position) */
	uint8_t chr		/* User character code (2 chars used from this) */
)
{
	static const uint8_t ptn[] = {
		0x06, 0x0C, 0x0C, 0x0C, 0x18, 0x18, 0x18, 0,
		0x06, 0x06, 0x0C, 0x0C, 0x0C, 0x18, 0x18, 0,
		0x06, 0x06, 0x06, 0x0C, 0x0C, 0x0C, 0x18, 0
	};
	const uint8_t *pp;
	uint16_t n, m, s, gi;
	uint8_t gfx[16];


	for (n = 0; n < 16; n++)		/* Register common pattern (space) */
		gfx[n] = n < 7 ? 0 : 0xFF;
	lcd_SetCG(_BASE_GRAPH, 1, gfx);

	/* Draw edge pattern into gfx[] */
	val = (uint32_t)val * (width * 18 - 12) / (_MAX_BAR + 1);
	pp = &ptn[(val % 3) * 8];		/* Get edge pattern */
	s = val / 3 % 6;				/* Bit shift */
	for (n = 0; n < 7; n++) {		/* Draw edge pattern into the pattern buffer */
		m = *pp++; m <<= 6; m >>= s;
		gfx[n] = m;
		gfx[n + 8] = m >> 6;
	}
	lcd_SetCG(chr, 2, gfx);				/* Register dot pattern */

	/* Put graphic pattern into the LCD module */
	gi = val / 18;						/* Indicator start position */
	for (n = 0; n < width; n++) {		/* Draw each location in the bargraph */
		if (n == gi) {					/* When edge pattern is exist at the location */
			m = chr + 1;				/* A edge pattern */
		} else {
			if (n == gi + 1)
				m = chr;
			else
				m = _BASE_GRAPH;		/* A space */
		}
		lcd_Putc(m);					/* Put the character into the LCD */
	}
}
#endif


/*-----------------------------------------------------------------------*/
/* Formatted Print Commands                                              */
/*-----------------------------------------------------------------------*/

// e.g. lcd_Printf_P(PSTR("\fMessage %u\r\n%u %u"), var1, var2, var2);

void lcd_Printf( const uint8_t * format, ...)
{
	va_list arg;

	va_start(arg, format);
	vsnprintf((char *)LCDWorkBuffer, portLCD_BUFFER, (const char *)format, arg);
	lcd_Print((uint8_t *)LCDWorkBuffer);
	va_end(arg);
}

void lcd_Printf_P(PGM_P format, ...)
{
	va_list arg;

	va_start(arg, format);
	vsnprintf_P((char *)LCDWorkBuffer, portLCD_BUFFER, format, arg);
	lcd_Print((uint8_t *)LCDWorkBuffer);
	va_end(arg);
}

void lcd_Print( uint8_t * str)
{
	int16_t i = 0;
	size_t stringlength;

	if( strlen((char *)str) < portLCD_BUFFER )
		stringlength = strlen((char *)str);
	else
		stringlength = portLCD_BUFFER-1;

	while(i < stringlength)
		lcd_Putc ( str[i++] );
}

void lcd_Print_P(PGM_P str)
{
	uint16_t i = 0;
	size_t stringlength;

	if( strlen_P(str) < portLCD_BUFFER )
		stringlength = strlen_P(str);
	else
		stringlength = portLCD_BUFFER-1;

	while(i < stringlength)
		lcd_Putc ( pgm_read_byte(&str[i++]) );
}

#endif // portHD44780_LCD
