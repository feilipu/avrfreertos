/*
    FreeRTOS V7.0.1 - Copyright (C) 2011 Real Time Engineers Ltd.


    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

#ifndef LIB_SERIAL_H
#define LIB_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <avr/pgmspace.h>

/* Constant for zero block time on xQueue */
#define xNoBlock						( ( uint8_t ) 0x00 )

typedef enum
{
	serCOM1,
	serCOM2,
	serCOM3,
	serCOM4,
	serCOM5,
	serCOM6,
	serCOM7,
	serCOM8
} eCOMPort;

typedef enum
{
	serNO_PARITY,
	serODD_PARITY,
	serEVEN_PARITY,
	serMARK_PARITY,
	serSPACE_PARITY
} eParity;

typedef enum
{
	serSTOP_1,
	serSTOP_2
} eStopBits;

typedef enum
{
	serBITS_5,
	serBITS_6,
	serBITS_7,
	serBITS_8
} eDataBits;

typedef enum
{
	ser50,
	ser75,
	ser110,
	ser134,
	ser150,
	ser200,
	ser300,
	ser600,
	ser1200,
	ser1800,
	ser2400,
	ser4800,
	ser9600,
	ser19200,
	ser38400,
	ser57600,
	ser115200
} eBaud;


typedef void * xComPortHandle;

/*
 * Create a handle reference for the (only) serial port.
 * */
extern xComPortHandle xSerialPort;

/*-----------------------------------------------------------*/

// xSerialPrintf_P(PSTR("\r\nMessage %u %u %u"), var1, var2, var2);

/* If you are worried about race situations on the serial port, then you should implement
 * a semaphore to limit this case.
 * Since I just use the serial port for debugging mainly, there seems to be too much
 * overhead to build this semaphore into the print functions themselves.
 */

/**
 * Serial printf.
 * @param format printf format string.
 */
void xSerialPrintf( const char * format, ...);

/**
 * Serial printf from PROGMEM to the serial port.
 * @param format printf format string.
 */
void xSerialPrintf_P(PGM_P format, ...);

/**
 * Print a character string to the serial port.
 * @param str string to print.
 */
void xSerialPrint( uint8_t * str);

/**
 * Print a character string from PROGMEM to the serial port.
 * @param str string to print.
 */
void xSerialPrint_P(PGM_P str);


/*-----------------------------------------------------------*/

/* Create a handle for the serial port. */
extern xComPortHandle xSerialPort;

/*-----------------------------------------------------------*/

xComPortHandle xSerialPortInitMinimal( uint32_t ulWantedBaud, unsigned portBASE_TYPE uxTxQueueLength, unsigned portBASE_TYPE uxRxQueueLength );

void vSerialClose( xComPortHandle xPort );

// xComPortHandle xSerialPortInit( eCOMPort ePort, eBaud eWantedBaud, eParity eWantedParity, eDataBits eWantedDataBits, eStopBits eWantedStopBits, unsigned portBASE_TYPE uxBufferLength );

/*-----------------------------------------------------------*/

/**
 * Interrupt driven routines to interface to ISR serial port IO.
 */
portBASE_TYPE xSerialGetChar( xComPortHandle pxPort, unsigned portBASE_TYPE *pcRxedChar, portTickType xBlockTime );
portBASE_TYPE xSerialPutChar( xComPortHandle pxPort, unsigned portBASE_TYPE cOutChar, portTickType xBlockTime );

/*-----------------------------------------------------------*/

// polling write and read routines, for use before freeRTOS vTaskStartScheduler
// same as arguments and function as above, but don't use the interrupts.

void avrSerialPrintf(const char * format, ...);
void avrSerialPrintf_P(PGM_P format, ...);

void avrSerialPrint(uint8_t * str);
void avrSerialPrint_P(PGM_P str);

void avrSerialWrite(int8_t DataOut);
int8_t avrSerialRead(void);

int8_t avrSerialCheckRxComplete(void);
int8_t avrSerialCheckTxReady(void);

/*-----------------------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif
