/*
 * Copyright (c) 2015 by Phillip Stevens
 * Serial library for AVR.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef LIB_SERIAL_H
#define LIB_SERIAL_H

#include <avr/pgmspace.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "portable.h"

#include "ringBuffer.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
	USART0,
	MSPIM0,
	USART1,
	MSPIM1,
	USART2,
	MSPIM2,
	USART3,
	MSPIM3
} eCOMPort;

typedef enum
{
	VACANT,
	ENGAGED
} binary;

typedef struct
{
	eCOMPort usart;
	ringBuffer_t xRxedChars;
	ringBuffer_t xCharsForTx;
	uint8_t *serialWorkBuffer;		// create a working buffer pointer, to later be malloc() on the heap.
	uint16_t serialWorkBufferSize;	// size of working buffer as created on the heap.
	binary	serialWorkBufferInUse;	// flag to prevent overwriting by multiple tasks using the same USART.
	uint32_t baudRate;				// configured baud rate.
} xComPortHandle, * xComPortHandlePtr;

/* Create reference to the handle for the serial port, USART0. */
/* This variable is special, as it is used in the interrupt */
extern xComPortHandle xSerialPort;

#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)

/* Create reference to the handle for the other serial port, USART1. */
/* This variable is special, as it is used in the interrupt */
extern xComPortHandle xSerial1Port;

#endif

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)

/* Create reference to the handle for the other serial port, USART2. */
/* This variable is special, as it is used in the interrupt */
extern xComPortHandle xSerial2Port;

/* Create reference to the handle for the other serial port, USART3. */
/* This variable is special, as it is used in the interrupt */
extern xComPortHandle xSerial3Port;

#endif

/*----------------------------------------------------------*/

xComPortHandle xSerialPortInitMinimal( eCOMPort ePort, uint32_t ulWantedBaud, uint16_t uxTxQueueLength, uint16_t uxRxQueueLength );
// xComPortHandle xSerialPortInit( eCOMPort ePort, eBaud eWantedBaud, eParity eWantedParity, eDataBits eWantedDataBits, eStopBits eWantedStopBits, unsigned portBASE_TYPE uxBufferLength );

void xSerialPortReInit( const xComPortHandlePtr oldComPortPtr );

void vSerialClose( const xComPortHandlePtr oldComPortPtr );

/*-----------------------------------------------------------*/

// xSerialPrintf_P(PSTR("\r\nMessage %u %u %u"), var1, var2, var2);

/* If you are worried about race situations on the serial port, then you should implement
 * a semaphore to limit this case.
 * Since I just use the serial port for debugging mainly, there seems to be too much
 * overhead to build this semaphore into the print functions themselves.
 *
 * This is now alleviated by using xSerialPort.serialWorkBufferInUse in xSerialPrintf(_P)();
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
void xSerialPrint( const uint8_t * str) __attribute__ ((flatten));

/**
 * Print a character string from PROGMEM to the serial port.
 * @param str string to print.
 */
void xSerialPrint_P(PGM_P str) __attribute__ ((flatten));

// These can be set to use any USART (but only four implemented for now).
void xSerialxPrintf( const xComPortHandlePtr pxPort, const char * format, ...);
void xSerialxPrintf_P( const xComPortHandlePtr pxPort, PGM_P format, ...);
void xSerialxPrint( const xComPortHandlePtr pxPort, const uint8_t * str) __attribute__ ((flatten));
void xSerialxPrint_P( const xComPortHandlePtr pxPort, PGM_P str) __attribute__ ((flatten));

/**
 * Interrupt driven routines to interface to ISR serial port IO.
 */
void xSerialRxFlush( const xComPortHandlePtr pxPort ) __attribute__ ((hot, flatten));
void xSerialTxFlush( const xComPortHandlePtr pxPort ) __attribute__ ((hot, flatten));
uint16_t xSerialAvailableChar( const xComPortHandlePtr pxPort ) __attribute__ ((hot, flatten));

UBaseType_t xSerialGetChar( const xComPortHandlePtr pxPort, UBaseType_t *pcRxedChar ) __attribute__ ((hot, flatten));
UBaseType_t xSerialPutChar( const xComPortHandlePtr pxPort, const UBaseType_t cOutChar ) __attribute__ ((hot, flatten));
/*-----------------------------------------------------------*/

// Polling write and read routines, for use before freeRTOS vTaskStartScheduler
// Same as arguments and function as above, but don't use the interrupts.

void avrSerialPrintf(const char * format, ...);
void avrSerialPrintf_P(PGM_P format, ...);

void avrSerialPrint(const uint8_t * str) __attribute__ ((flatten));
void avrSerialPrint_P(PGM_P str) __attribute__ ((flatten));

void avrSerialWrite(const uint8_t DataOut) __attribute__ ((flatten));
int8_t avrSerialRead(void) __attribute__ ((flatten));

uint8_t avrSerialCheckRxComplete(void) __attribute__ ((flatten));
uint8_t avrSerialCheckTxReady(void) __attribute__ ((flatten));

// These can be set to use any USART (but only four implemented for now).

void avrSerialxPrintf(const xComPortHandlePtr pxPort, const char * format, ...);
void avrSerialxPrintf_P(const xComPortHandlePtr pxPort, PGM_P format, ...);

void avrSerialxPrint(const xComPortHandlePtr pxPort, const uint8_t * str) __attribute__ ((flatten));
void avrSerialxPrint_P(const xComPortHandlePtr pxPort, PGM_P str) __attribute__ ((flatten));

void avrSerialxWrite(const xComPortHandlePtr pxPort, uint8_t DataOut) __attribute__ ((flatten));
int8_t avrSerialxRead(const xComPortHandlePtr pxPort) __attribute__ ((flatten));

uint8_t avrSerialxCheckRxComplete(const xComPortHandlePtr pxPort) __attribute__ ((flatten));
uint8_t avrSerialxCheckTxReady(const xComPortHandlePtr pxPort) __attribute__ ((flatten));


/*-----------------------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif
