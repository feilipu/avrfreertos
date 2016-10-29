/*
 * Copyright (c) 2015 by Phillip Stevens
 * Serial library for AVR.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

/* BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER. */
/* Also with polling serial functions, for use before scheduler is enabled */

/* NOTE: The MSPIM functionality is not tested, is incomplete, and parts of it actually won't work.
 * It is just part of a solution (that has low priority) currently. */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include <util/delay.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "ringBuffer.h"

#include "serial.h"


/*-----------------------------------------------------------*/

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)

#define vInterrupt0_On()									\
{															\
	UCSR0B |= _BV(UDRIE0);									\
}

#define vInterrupt0_Off()									\
{															\
	UCSR0B &= ~_BV(UDRIE0);									\
}


#elif defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__)

#define vInterrupt0_On()									\
{															\
	UCSR0B |= _BV(UDRIE0);									\
}

#define vInterrupt0_Off()									\
{															\
	UCSR0B &= ~_BV(UDRIE0);									\
}

#define vInterrupt1_On()									\
{															\
	UCSR1B |= _BV(UDRIE1);									\
}

#define vInterrupt1_Off()									\
{															\
	UCSR1B &= ~_BV(UDRIE1);									\
}

#elif  defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)

#define vInterrupt0_On()									\
{															\
	UCSR0B |= _BV(UDRIE0);									\
}

#define vInterrupt0_Off()									\
{															\
	UCSR0B &= ~_BV(UDRIE0);									\
}

#define vInterrupt1_On()									\
{															\
	UCSR1B |= _BV(UDRIE1);									\
}

#define vInterrupt1_Off()									\
{															\
	UCSR1B &= ~_BV(UDRIE1);									\
}

#define vInterrupt2_On()									\
{															\
	UCSR2B |= _BV(UDRIE2);									\
}

#define vInterrupt2_Off()									\
{															\
	UCSR2B &= ~_BV(UDRIE2);									\
}

#define vInterrupt3_On()									\
{															\
	UCSR3B |= _BV(UDRIE3);									\
}

#define vInterrupt3_Off()									\
{															\
	UCSR3B &= ~_BV(UDRIE3);									\
}

#else

#define vInterrupt0_On()
#define vInterrupt0_Off()
#define vInterrupt1_On()
#define vInterrupt1_Off()
#define vInterrupt2_On()
#define vInterrupt2_Off()
#define vInterrupt3_On()
#define vInterrupt3_Off()

#endif

/*-----------------------------------------------------------*/

/* Create a handle for the serial port, USART0. */
/* This variable is special, as it is used in the interrupt */
xComPortHandle xSerialPort;

#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)

/* Create a handle for the other serial port, USART1. */
/* This variable is special, as it is used in the interrupt */
xComPortHandle xSerial1Port;

#endif

#if  defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)

/* Create a handle for the other serial port, USART2. */
/* This variable is special, as it is used in the interrupt */
xComPortHandle xSerial2Port;

/* Create a handle for the other serial port, USART3. */
/* This variable is special, as it is used in the interrupt */
xComPortHandle xSerial3Port;

#endif

/*-----------------------------------------------------------------*/

// xSerialPrintf_P(PSTR("\r\nMessage %u %u %u"), var1, var2, var2);

void xSerialPrintf( const char * format, ...)
{
	va_list arg;

	va_start(arg, format);

	while(xSerialPort.serialWorkBufferInUse == ENGAGED ) taskYIELD();
	xSerialPort.serialWorkBufferInUse = ENGAGED;

	vsnprintf((char *)(xSerialPort.serialWorkBuffer), xSerialPort.serialWorkBufferSize, (const char *)format, arg);
	xSerialPrint((uint8_t *)(xSerialPort.serialWorkBuffer));

	xSerialPort.serialWorkBufferInUse = VACANT;

	va_end(arg);
}

void xSerialPrintf_P(PGM_P format, ...)
{
	va_list arg;

	va_start(arg, format);

	while(xSerialPort.serialWorkBufferInUse == ENGAGED ) taskYIELD();
	xSerialPort.serialWorkBufferInUse = ENGAGED;

	vsnprintf_P((char *)(xSerialPort.serialWorkBuffer), xSerialPort.serialWorkBufferSize, format, arg);
	xSerialPrint((uint8_t *)(xSerialPort.serialWorkBuffer));

	xSerialPort.serialWorkBufferInUse = VACANT;

	va_end(arg);
}

void xSerialPrint( const uint8_t * str)
{
	int16_t i = 0;
	size_t stringlength;

	stringlength = strlen((char *)str);

	while(i < stringlength)
		xSerialPutChar( &xSerialPort, str[i++] );
}

void xSerialPrint_P(PGM_P str)
{
	uint16_t i = 0;
	size_t stringlength;

	stringlength = strlen_P(str);

	while(i < stringlength)
		xSerialPutChar( &xSerialPort, pgm_read_byte(&str[i++]) );
}
/*-----------------------------------------------------------*/

void xSerialxPrintf( const xComPortHandlePtr pxPort, const char * format, ...)
{
	va_list arg;

	va_start(arg, format);

	while(pxPort->serialWorkBufferInUse == ENGAGED ) taskYIELD();
	pxPort->serialWorkBufferInUse = ENGAGED;

	vsnprintf((char *)(pxPort->serialWorkBuffer), pxPort->serialWorkBufferSize, (const char *)format, arg);
	xSerialxPrint(pxPort, (uint8_t *)(pxPort->serialWorkBuffer));

	pxPort->serialWorkBufferInUse = VACANT;

	va_end(arg);
}

void xSerialxPrintf_P( const xComPortHandlePtr pxPort, PGM_P format, ...)
{
	va_list arg;

	va_start(arg, format);

	while(pxPort->serialWorkBufferInUse == ENGAGED ) taskYIELD();
	pxPort->serialWorkBufferInUse = ENGAGED;

	vsnprintf_P((char *)(pxPort->serialWorkBuffer), pxPort->serialWorkBufferSize, format, arg);
	xSerialxPrint(pxPort, (uint8_t *)(pxPort->serialWorkBuffer));

	pxPort->serialWorkBufferInUse = VACANT;

	va_end(arg);
}

void xSerialxPrint( const xComPortHandlePtr pxPort, const uint8_t * str)
{
	int16_t i = 0;
	size_t stringlength;

	stringlength = strlen((char *)str);

	while(i < stringlength)
		xSerialPutChar( pxPort, str[i++]);
}

void xSerialxPrint_P( const xComPortHandlePtr pxPort, PGM_P str)
{
	uint16_t i = 0;
	size_t stringlength;

	stringlength = strlen_P(str);

	while(i < stringlength)
		xSerialPutChar( pxPort, pgm_read_byte(&str[i++]) );
}
/*-----------------------------------------------------------*/

void xSerialRxFlush( const xComPortHandlePtr pxPort )
{
	/* Flush received characters from the serial port RX buffer.*/

	uint8_t byte __attribute__ ((unused));

	switch (pxPort->usart)
	{
	case USART0:
	case MSPIM0:
		while ( UCSR0A & _BV(RXC0) ) // flush received bytes
			byte = UDR0;
		break;

	case USART1:
	case MSPIM1:
#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		while ( UCSR1A & _BV(RXC1) )
			byte = UDR1;
		break;
#endif

	case USART2:
	case MSPIM2:
#if  defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		while ( UCSR2A & _BV(RXC2) )
			byte = UDR2;
		break;
#endif

	case USART3:
	case MSPIM3:
#if  defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		while ( UCSR3A & _BV(RXC3) )
			byte = UDR3;
		break;
#endif

	default:
		break;
	}

	ringBuffer_Flush( (ringBuffer_t*) &(pxPort->xRxedChars) );	// flush received characters
}
/*-----------------------------------------------------------*/

void xSerialTxFlush( const xComPortHandlePtr pxPort )
{
	/* Flush transmitted characters from the serial port buffer.*/

	switch (pxPort->usart)
	{
	case USART0:
	case MSPIM0:
		vInterrupt0_Off(); // stop transmitting
		break;

	case USART1:
	case MSPIM1:
#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		vInterrupt1_Off();
		break;
#endif

	case USART2:
	case MSPIM2:
#if  defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		vInterrupt2_Off();
		break;
#endif

	case USART3:
	case MSPIM3:
#if  defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		vInterrupt3_Off();
		break;
#endif

	default:
		break;
	}

	ringBuffer_Flush( (ringBuffer_t*) &(pxPort->xCharsForTx) ); // flush not yet transmitted characters.
}

uint16_t xSerialAvailableChar( const xComPortHandlePtr pxPort )
{
	/* Are characters available in the serial port buffer.*/

	return ringBuffer_GetCount( (ringBuffer_t*) &(pxPort->xRxedChars) );
}

UBaseType_t xSerialGetChar( const xComPortHandlePtr pxPort, UBaseType_t *pcRxedChar )
{
	/* Get the next character from the ring buffer.  Return false if no characters are available */

	if( ringBuffer_IsEmpty( (ringBuffer_t*) &(pxPort->xRxedChars) ) )
	{
		return pdFALSE;
	}
	else
	{
		* pcRxedChar = ringBuffer_Pop( (ringBuffer_t*) &(pxPort->xRxedChars) );
		return pdTRUE;
	}
}

UBaseType_t xSerialPutChar( const xComPortHandlePtr pxPort, const UBaseType_t cOutChar )
{
	/* Return false if there remains no room on the Tx ring buffer */

	if( ! ringBuffer_IsFull( (ringBuffer_t*) &(pxPort->xCharsForTx) ) )
		ringBuffer_Poke( (ringBuffer_t*) &(pxPort->xCharsForTx), cOutChar ); // poke in a fast byte
	else
	{
		 // go slower, per character rate for 38400 is 28ms
		_delay_ms(32); // delay for about one character

		if( ! ringBuffer_IsFull( (ringBuffer_t*) &(pxPort->xCharsForTx) ) )
			ringBuffer_Poke( (ringBuffer_t*) &(pxPort->xCharsForTx), cOutChar ); // poke in a byte slowly
		else
			return pdFAIL; // if the Tx ring buffer remains full
	}

	switch (pxPort->usart)
	{
	case USART0:
	case MSPIM0:
		vInterrupt0_On();
		break;

	case USART1:
	case MSPIM1:
#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		vInterrupt1_On();
		break;
#endif

	case USART2:
	case MSPIM2:
#if  defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		vInterrupt2_On();
		break;
#endif

	case USART3:
	case MSPIM3:
#if  defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		vInterrupt3_On();
		break;
#endif

	default:
		break;
	}

	return pdPASS;
}
/*-----------------------------------------------------------*/

xComPortHandle xSerialPortInitMinimal( eCOMPort ePort, uint32_t ulWantedBaud, uint16_t uxTxQueueLength, uint16_t uxRxQueueLength )
{
	uint8_t * dataPtr;

	xComPortHandle newComPort;

	/* Create the ring-buffers used by the serial communications task. */
	if( (dataPtr = (uint8_t *)pvPortMalloc( sizeof(uint8_t) * uxRxQueueLength )))
		ringBuffer_InitBuffer( (ringBuffer_t*) &(newComPort.xRxedChars), dataPtr, uxRxQueueLength);

	if( (dataPtr = (uint8_t *)pvPortMalloc( sizeof(uint8_t) * uxTxQueueLength )))
		ringBuffer_InitBuffer( (ringBuffer_t*) &(newComPort.xCharsForTx), dataPtr, uxTxQueueLength);

	// create a working buffer for vsnprintf on the heap (so we can use extended RAM, if available).
	// create the structures on the heap (so they can be moved later).
	if( !(newComPort.serialWorkBuffer = (uint8_t *)pvPortMalloc( sizeof(uint8_t) * uxTxQueueLength )))
		newComPort.serialWorkBuffer = NULL;

	newComPort.usart = ePort; // containing eCOMPort
	newComPort.serialWorkBufferSize = uxTxQueueLength; // size of the working buffer for vsnprintf
	newComPort.serialWorkBufferInUse = VACANT;  // clear the occupation flag.
	newComPort.baudRate = ulWantedBaud; // containing the desired baud rate.

	portENTER_CRITICAL();

	switch (newComPort.usart)
	{
	case USART0:
		/*
		 * Calculate the baud rate register value from the equation in the data sheet. */

		/* As the 16MHz Arduino boards have bad karma for serial port, we're using the 2x clock U2X0 */
		// for Arduino at 16MHz; above data sheet calculation is wrong. Need below from <util/setbaud.h>
		// This provides correct rounding truncation to get closest to correct speed.
		// Normal mode gives 3.7% error, which is too much. Use 2x mode gives 2.1% error.

		// Or, use 22.1184 MHz over clock which gives 0.00% error, for all rates.
		// Or Goldilocks Analogue uses 24.576MHz which gives 0.00% error for 9600, 19200, 38400, and 76800 baud.

#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__)
		UBRR0 = (uint16_t)((configCPU_CLOCK_HZ + ulWantedBaud * 8UL) / (ulWantedBaud * 16UL) - 1); // for 1x mode, using 16 bit avr-gcc capability.
		UCSR0A = 0x00; // 1x mode.
#else
		UBRR0 = (uint16_t)((configCPU_CLOCK_HZ + ulWantedBaud * 4UL) / (ulWantedBaud * 8UL) - 1);  // for 2x mode, using 16 bit avr-gcc capability.
		UCSR0A = _BV(U2X0); // 2x mode.		// 2x speed mode bit
#endif

		/* Enable the Rx and Tx. Also enable the Rx interrupt. The Tx interrupt will get enabled later. */
		UCSR0B = ( _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0) );

		/* Set the data bit register to 8n2. Two Stop Bits only affects transmitter.*/
		UCSR0C = ( _BV(USBS0) | _BV(UCSZ01) | _BV(UCSZ00) );

		break;

	case MSPIM0:

		/* Set the baud rate to highest to get the XCK0 to be set fastest */
		UBRR0 = 0x0000;

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
		DDRD |= _BV(PD4);							// Setting the XCK0 port pin as output, enables USART master SPI mode.
#elif defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__)
		DDRB |= _BV(PB0);							// Setting the XCK0 port pin as output, enables USART master SPI mode.
#elif  defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		DDRE |= _BV(PE2);							// Setting the XCK0 port pin as output, enables USART master SPI mode.
#else
#warning MSPIM XCK Pin not defined - Check MCU type
#endif

		/* Set USART MSPI mode of operation and SPI data mode 0,0. UCPHA0 = UCSZ00 */
		UCSR0C = _BV(UMSEL01) | _BV(UMSEL00);

		/* Enable the Rx and Tx. Also enable the Rx interrupt. The Tx interrupt will get enabled later. */
		UCSR0B = ( _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0));

		/* Calculate the baud rate register value from the equation in the data sheet.
		 * ulBaudRateCounter = ((configCPU_CLOCK_HZ + ulWantedBaud * 8UL) / (ulWantedBaud * 16UL) - 1); for normal mode.
		 * Set baud rate.  IMPORTANT: The Baud Rate must be set after the transmitter is enabled. */
		UBRR0 = (uint16_t)((configCPU_CLOCK_HZ + ulWantedBaud * 8UL) / (ulWantedBaud * 16UL) - 1);  // using 16 bit avr-gcc capability.

		break;

	case USART1:
#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__)
		UBRR1 = (uint16_t)((configCPU_CLOCK_HZ + ulWantedBaud * 8UL) / (ulWantedBaud * 16UL) - 1); // for 1x mode, using 16 bit avr-gcc capability.
		UCSR1A = 0x00; // 1x mode.
		UCSR1B = ( _BV(RXCIE1) | _BV(RXEN1) | _BV(TXEN1));
		UCSR1C = ( _BV(USBS1) | _BV(UCSZ11) | _BV(UCSZ10) );
#else
		UBRR1 = (uint16_t)((configCPU_CLOCK_HZ + ulWantedBaud * 4UL) / (ulWantedBaud * 8UL) - 1);  // for 2x mode, using 16 bit avr-gcc capability.
		UCSR1A = _BV(U2X0); // 2x mode.		// 2x speed mode bit
		UCSR1B = ( _BV(RXCIE1) | _BV(RXEN1) | _BV(TXEN1));
		UCSR1C = ( _BV(USBS1) | _BV(UCSZ11) | _BV(UCSZ10) );
#endif
#endif
		break;

	case MSPIM1:
#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		UBRR1 = 0x0000;
#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__)
		DDRD |= _BV(PD4);
#elif  defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		DDRD |= _BV(PD5);
#else
#warning MSPIM XCK Pin not defined - Check MCU type
#endif
		UCSR1C = _BV(UMSEL11) | _BV(UMSEL10);
		UCSR1B = ( _BV(RXCIE1) | _BV(RXEN1) | _BV(TXEN1));
		UBRR1 = (uint16_t)((configCPU_CLOCK_HZ + ulWantedBaud * 8UL) / (ulWantedBaud * 16UL) - 1);
#endif
		break;

	case USART2:
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		UBRR2 = (uint16_t)((configCPU_CLOCK_HZ + ulWantedBaud * 4UL) / (ulWantedBaud * 8UL) - 1);  // for 2x mode, using 16 bit avr-gcc capability.
		UCSR2A = _BV(U2X0); // 2x mode.		// 2x speed mode bit
		UCSR2B = ( _BV(RXCIE2) | _BV(RXEN2) | _BV(TXEN2));
		UCSR2C = ( _BV(USBS2) | _BV(UCSZ21) | _BV(UCSZ20) );
#endif
		break;

	case MSPIM2:
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		UBRR2 = 0x0000;
		DDRH |= _BV(PH2);
		UCSR2C = _BV(UMSEL21) | _BV(UMSEL20);
		UCSR2B = ( _BV(RXCIE2) | _BV(RXEN2) | _BV(TXEN2));
		UBRR2 = (uint16_t)((configCPU_CLOCK_HZ + ulWantedBaud * 8UL) / (ulWantedBaud * 16UL) - 1);
#endif
		break;

	case USART3:
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		UBRR3 = (uint16_t)((configCPU_CLOCK_HZ + ulWantedBaud * 4UL) / (ulWantedBaud * 8UL) - 1);  // for 2x mode, using 16 bit avr-gcc capability.
		UCSR3A = _BV(U2X0); // 2x mode.		// 2x speed mode bit
		UCSR3B = ( _BV(RXCIE3) | _BV(RXEN3) | _BV(TXEN3));
		UCSR3C = ( _BV(USBS3) | _BV(UCSZ31) | _BV(UCSZ30) );
#endif
		break;

	case MSPIM3:
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		UBRR3 = 0x0000;
		DDRJ |= _BV(PJ2);
		UCSR3C = _BV(UMSEL31) | _BV(UMSEL30);
		UCSR3B = ( _BV(RXCIE3) | _BV(RXEN3) | _BV(TXEN3));
		UBRR3 = (uint16_t)((configCPU_CLOCK_HZ + ulWantedBaud * 8UL) / (ulWantedBaud * 16UL) - 1);
#endif
		break;

	default:
		break;
	}

	portEXIT_CRITICAL();

	return newComPort;
}

void xSerialPortReInit( const xComPortHandlePtr oldComPortPtr )
{

	uint8_t byte __attribute__ ((unused));

	portENTER_CRITICAL();

	switch (oldComPortPtr->usart)
	{
	case USART0:
	case MSPIM0:
		vInterrupt0_Off();				// stop transmitting
		while ( UCSR0A & _BV(RXC0) )	// flush received bytes
			byte = UDR0;
		break;

	case USART1:
	case MSPIM1:
#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		vInterrupt1_Off();
		while ( UCSR1A & _BV(RXC1) )
			byte = UDR1;
		break;
#endif

	case USART2:
	case MSPIM2:
#if  defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		vInterrupt2_Off();
		while ( UCSR2A & _BV(RXC2) )
			byte = UDR2;
		break;
#endif

	case USART3:
	case MSPIM3:
#if  defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		vInterrupt3_Off();
		while ( UCSR3A & _BV(RXC3) )
			byte = UDR3;
		break;
#endif

	default:
		break;
	}

	/* Flush both the ring-buffers used by the serial communications task. */
	if( &oldComPortPtr->xCharsForTx )
		ringBuffer_Flush( (ringBuffer_t*) &(oldComPortPtr->xCharsForTx) );	// flush not yet transmitted characters.
	if( &oldComPortPtr->xRxedChars )
		ringBuffer_Flush( (ringBuffer_t*) &(oldComPortPtr->xRxedChars) );	// flush received characters

	oldComPortPtr->serialWorkBufferInUse = VACANT; 							// clear the occupation flag.

	switch (oldComPortPtr->usart)
	{
	case USART0:
		/*
		 * Calculate the baud rate register value from the equation in the data sheet. */

		/* As the 16MHz Arduino boards have bad karma for serial port, we're using the 2x clock U2X0 */
		// for Arduino at 16MHz; above data sheet calculation is wrong. Need below from <util/setbaud.h>
		// This provides correct rounding truncation to get closest to correct speed.
		// Normal mode gives 3.7% error, which is too much. Use 2x mode gives 2.1% error.

		// Or, use 22.1184 MHz over clock which gives 0.00% error, for all rates.
		// Or Goldilocks Analogue uses 24.576MHz which gives 0.00% error for 9600, 19200, 38400, and 76800 baud.

#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__)
		UBRR0 = (uint16_t)((configCPU_CLOCK_HZ + oldComPortPtr->baudRate * 8UL) / (oldComPortPtr->baudRate * 16UL) - 1); // for 1x mode, using 16 bit avr-gcc capability.
		UCSR0A = 0x00; // 1x mode.
#else
		UBRR0 = (uint16_t)((configCPU_CLOCK_HZ + oldComPortPtr->baudRate * 4UL) / (oldComPortPtr->baudRate * 8UL) - 1);  // for 2x mode, using 16 bit avr-gcc capability.
		UCSR0A = _BV(U2X0); // 2x mode.		// 2x speed mode bit
#endif

		/* Enable the Rx and Tx. Also enable the Rx interrupt. The Tx interrupt will get enabled later. */
		UCSR0B = ( _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0) );

		/* Set the data bit register to 8n2. Two Stop Bits only affects transmitter.*/
		UCSR0C = ( _BV(USBS0) | _BV(UCSZ01) | _BV(UCSZ00) );

		break;

	case MSPIM0:

		/* Set the baud rate to highest to get the XCK0 to be set fastest */
		UBRR0 = 0x0000;

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
		DDRD |= _BV(PD4);							// Setting the XCK0 port pin as output, enables USART master SPI mode.
#elif defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__)
		DDRB |= _BV(PB0);							// Setting the XCK0 port pin as output, enables USART master SPI mode.
#elif  defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		DDRE |= _BV(PE2);							// Setting the XCK0 port pin as output, enables USART master SPI mode.
#else
#warning MSPIM XCK Pin not defined - Check MCU type
#endif

		/* Set USART MSPI mode of operation and SPI data mode 0,0. UCPHA0 = UCSZ00 */
		UCSR0C = _BV(UMSEL01) | _BV(UMSEL00);

		/* Enable the Rx and Tx. Also enable the Rx interrupt. The Tx interrupt will get enabled later. */
		UCSR0B = ( _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0));


		/* Calculate the baud rate register value from the equation in the data sheet.
		 * ulBaudRateCounter = ((configCPU_CLOCK_HZ + ulWantedBaud * 8UL) / (ulWantedBaud * 16UL) - 1); for normal mode.
		 * Set baud rate.  IMPORTANT: The Baud Rate must be set after the transmitter is enabled. */
		UBRR0 = (uint16_t)((configCPU_CLOCK_HZ + oldComPortPtr->baudRate * 8UL) / (oldComPortPtr->baudRate * 16UL) - 1);  // using 16 bit avr-gcc capability.

		break;

	case USART1:
#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__)
		UBRR1 = (uint16_t)((configCPU_CLOCK_HZ + oldComPortPtr->baudRate * 8UL) / (oldComPortPtr->baudRate * 16UL) - 1); // for 1x mode, using 16 bit avr-gcc capability.
		UCSR1A = 0x00; // 1x mode.
		UCSR1B = ( _BV(RXCIE1) | _BV(RXEN1) | _BV(TXEN1));
		UCSR1C = ( _BV(USBS1) | _BV(UCSZ11) | _BV(UCSZ10) );
#else
		UBRR1 = (uint16_t)((configCPU_CLOCK_HZ + oldComPortPtr->baudRate * 4UL) / (oldComPortPtr->baudRate * 8UL) - 1);  // for 2x mode, using 16 bit avr-gcc capability.
		UCSR1A = _BV(U2X0); // 2x mode.		// 2x speed mode bit
		UCSR1B = ( _BV(RXCIE1) | _BV(RXEN1) | _BV(TXEN1));
		UCSR1C = ( _BV(USBS1) | _BV(UCSZ11) | _BV(UCSZ10) );
#endif
#endif
		break;

	case MSPIM1:
#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		UBRR1 = 0x0000;
#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__)
		DDRD |= _BV(PD4);
#elif  defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		DDRD |= _BV(PD5);
#else
#warning MSPIM XCK Pin not defined - Check MCU type
#endif
		UCSR1C = _BV(UMSEL11) | _BV(UMSEL10);
		UCSR1B = ( _BV(RXCIE1) | _BV(RXEN1) | _BV(TXEN1));
		UBRR1 = (uint16_t)((configCPU_CLOCK_HZ + oldComPortPtr->baudRate * 8UL) / (oldComPortPtr->baudRate * 16UL) - 1);
#endif
		break;

	case USART2:
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		UBRR2 = (uint16_t)((configCPU_CLOCK_HZ + oldComPortPtr->baudRate * 4UL) / (oldComPortPtr->baudRate * 8UL) - 1);  // for 2x mode, using 16 bit avr-gcc capability.
		UCSR2A = _BV(U2X0); // 2x mode.		// 2x speed mode bit
		UCSR2B = ( _BV(RXCIE2) | _BV(RXEN2) | _BV(TXEN2));
		UCSR2C = ( _BV(USBS2) | _BV(UCSZ21) | _BV(UCSZ20) );
#endif
		break;

	case MSPIM2:
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		UBRR2 = 0x0000;
		DDRH |= _BV(PH2);
		UCSR2C = _BV(UMSEL21) | _BV(UMSEL20);
		UCSR2B = ( _BV(RXCIE2) | _BV(RXEN2) | _BV(TXEN2));
		UBRR2 = (uint16_t)((configCPU_CLOCK_HZ + oldComPortPtr->baudRate * 8UL) / (oldComPortPtr->baudRate * 16UL) - 1);
#endif
		break;

	case USART3:
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		UBRR3 = (uint16_t)((configCPU_CLOCK_HZ + oldComPortPtr->baudRate * 4UL) / (oldComPortPtr->baudRate * 8UL) - 1);  // for 2x mode, using 16 bit avr-gcc capability.
		UCSR3A = _BV(U2X0); // 2x mode.		// 2x speed mode bit
		UCSR3B = ( _BV(RXCIE3) | _BV(RXEN3) | _BV(TXEN3));
		UCSR3C = ( _BV(USBS3) | _BV(UCSZ31) | _BV(UCSZ30) );
#endif
		break;

	case MSPIM3:
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		UBRR3 = 0x0000;
		DDRJ |= _BV(PJ2);
		UCSR3C = _BV(UMSEL31) | _BV(UMSEL30);
		UCSR3B = ( _BV(RXCIE3) | _BV(RXEN3) | _BV(TXEN3));
		UBRR3 = (uint16_t)((configCPU_CLOCK_HZ + oldComPortPtr->baudRate * 8UL) / (oldComPortPtr->baudRate * 16UL) - 1);
#endif
		break;

	default:
		break;
	}

	portEXIT_CRITICAL();
}

void vSerialClose( const xComPortHandlePtr oldComPortPtr )
{
	/* Turn off the interrupts.  We may also want to delete the queues and/or
	re-install the original ISR. */

	vPortFree( oldComPortPtr->serialWorkBuffer );
	vPortFree( oldComPortPtr->xRxedChars.start );
	vPortFree( oldComPortPtr->xCharsForTx.start );

	portENTER_CRITICAL();
	{
		switch (oldComPortPtr->usart)
		{
		case USART0:
		case MSPIM0:
			vInterrupt0_Off();
			UCSR0B &= ~( _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0));
			break;

		case USART1:
		case MSPIM1:
#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
			vInterrupt1_Off();
			UCSR1B &= ~( _BV(RXCIE1) | _BV(RXEN1) | _BV(TXEN1));
			break;
#endif

		case USART2:
		case MSPIM2:
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
			vInterrupt2_Off();
			UCSR2B &= ~( _BV(RXCIE2) | _BV(RXEN2) | _BV(TXEN2));
			break;
#endif

		case USART3:
		case MSPIM3:
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
			vInterrupt3_Off();
			UCSR3B &= ~( _BV(RXCIE3) | _BV(RXEN3) | _BV(TXEN3));
			break;
#endif

		default:
			break;
		}
	}
	portEXIT_CRITICAL();
}

/*-----------------------------------------------------------*/

// polling read and write routines, for use before freeRTOS vTaskStartScheduler (interrupts enabled).
// same as above, but doesn't use interrupts.

void avrSerialPrintf(const char * format, ...)
{
	va_list arg;

	va_start(arg, format);

	while(xSerialPort.serialWorkBufferInUse == ENGAGED ) _delay_us(25);
	xSerialPort.serialWorkBufferInUse = ENGAGED;

	vsnprintf((char *)(xSerialPort.serialWorkBuffer), xSerialPort.serialWorkBufferSize, (const char *)format, arg);
	avrSerialPrint((xSerialPort.serialWorkBuffer));

	xSerialPort.serialWorkBufferInUse = VACANT;

	va_end(arg);
}

void avrSerialPrintf_P(PGM_P format, ...)
{
	va_list arg;

	va_start(arg, format);

	while(xSerialPort.serialWorkBufferInUse == ENGAGED ) _delay_us(25);
	xSerialPort.serialWorkBufferInUse = ENGAGED;

	vsnprintf_P((char *)(xSerialPort.serialWorkBuffer), xSerialPort.serialWorkBufferSize, format, arg);
	avrSerialPrint((xSerialPort.serialWorkBuffer));

	xSerialPort.serialWorkBufferInUse = VACANT;

	va_end(arg);
}

void avrSerialPrint(const uint8_t * str)
{
	uint16_t i = 0;
	size_t stringlength;

	stringlength = strlen((char *)str);

	while(i < stringlength)
		avrSerialWrite(str[i++]);
}

void avrSerialPrint_P(PGM_P str)
{
	uint16_t i = 0;
	size_t stringlength;

	stringlength = strlen_P(str);

	while(i < stringlength)
		avrSerialWrite(pgm_read_byte(&str[i++]));
}

void avrSerialWrite(const uint8_t DataOut)
{
	while (!avrSerialCheckTxReady())		// while NOT ready to transmit
        _delay_us(25);     					// delay
	UDR0 = DataOut;
}

int8_t avrSerialRead(void)
{

	while (!avrSerialCheckRxComplete())		// While data is NOT available to read
		_delay_us(25);     					// delay
	/* Get status and data */
	/* from buffer */

	/* If error, return 0xFF */
	if ( UCSR0A & (_BV(FE0)|_BV(DOR0)|_BV(UPE0)) )
	return 0xFF;
	else
	return UDR0;
}

uint8_t avrSerialCheckRxComplete(void)
{
	return( UCSR0A & (_BV(RXC0)) );			// nonzero if serial data is available to read.
}

uint8_t avrSerialCheckTxReady(void)
{
	return( UCSR0A & (_BV(UDRE0)) );		// nonzero if transmit register is ready to receive new data.
}

/*-----------------------------------------------------------*/

// Polling read and write routines, for use before freeRTOS vTaskStartScheduler (interrupts enabled).
// Same as above, but doesn't use interrupts.
// These can be set to use any USART (but only two implemented for now).

void avrSerialxPrintf(const xComPortHandlePtr pxPort, const char * format, ...)
{
	va_list arg;

	va_start(arg, format);

	while(pxPort->serialWorkBufferInUse == ENGAGED ) _delay_us(25);
	pxPort->serialWorkBufferInUse = ENGAGED;

	vsnprintf((char *)(pxPort->serialWorkBuffer), pxPort->serialWorkBufferSize, (const char *)format, arg);
	avrSerialxPrint(pxPort, pxPort->serialWorkBuffer);

	pxPort->serialWorkBufferInUse = VACANT;

	va_end(arg);
}

void avrSerialxPrintf_P(const xComPortHandlePtr pxPort, PGM_P format, ...)
{
	va_list arg;

	va_start(arg, format);

	while(pxPort->serialWorkBufferInUse == ENGAGED ) _delay_us(25);
	pxPort->serialWorkBufferInUse = ENGAGED;

	vsnprintf_P((char *)(pxPort->serialWorkBuffer), pxPort->serialWorkBufferSize, format, arg);
	avrSerialxPrint(pxPort, pxPort->serialWorkBuffer);

	pxPort->serialWorkBufferInUse = VACANT;

	va_end(arg);
}

void avrSerialxPrint(const xComPortHandlePtr pxPort, const uint8_t * str)
{
	uint16_t i = 0;
	size_t stringlength;

	stringlength = strlen((char *)str);

	while(i < stringlength)
		avrSerialxWrite(pxPort, str[i++]);
}

void avrSerialxPrint_P(const xComPortHandlePtr pxPort, PGM_P str)
{
	uint16_t i = 0;
	size_t stringlength;

	stringlength = strlen_P(str);

	while(i < stringlength)
		avrSerialxWrite(pxPort, pgm_read_byte(&str[i++]));
}

void avrSerialxWrite(const xComPortHandlePtr pxPort, const uint8_t DataOut)
{
	while (!avrSerialxCheckTxReady(pxPort))		// while NOT ready to transmit
        _delay_us(25);     						// delay

	switch (pxPort->usart)
	{
		case USART0:
		case MSPIM0:
			UDR0 = DataOut;
			break;

		case USART1:
		case MSPIM1:
#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
			UDR1 = DataOut;
#endif
			break;

		case USART2:
		case MSPIM2:
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
			UDR2 = DataOut;
#endif
			break;

		case USART3:
		case MSPIM3:
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
			UDR3 = DataOut;
#endif
			break;

		default:
			break;
	}
}

int8_t avrSerialxRead(const xComPortHandlePtr pxPort)
{
	while (!avrSerialxCheckRxComplete(pxPort))	// While data is NOT available to read
		_delay_us(25);     						// delay
	/* Get status and data */
	/* from buffer */

	switch (pxPort->usart)
	{
		case USART0:
			/* If error, return 0xFF */
			if ( UCSR0A & (_BV(FE0)|_BV(DOR0)|_BV(UPE0)) )
				return 0xFF;
			else
				return UDR0;
			break;

		case MSPIM0:
			break;

		case USART1:
#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
			/* If error, return 0xFF */
			if ( UCSR1A & (_BV(FE1)|_BV(DOR1)|_BV(UPE1)) )
				return 0xFF;
			else
				return UDR1;
#endif
			break;

		case MSPIM1:
#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
#endif
			break;


		case USART2:
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
			/* If error, return 0xFF */
			if ( UCSR2A & (_BV(FE2)|_BV(DOR2)|_BV(UPE2)) )
				return 0xFF;
			else
				return UDR1;
#endif
			break;

		case MSPIM2:
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
#endif
			break;

		case USART3:
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
			/* If error, return 0xFF */
			if ( UCSR3A & (_BV(FE3)|_BV(DOR3)|_BV(UPE3)) )
				return 0xFF;
			else
				return UDR1;
#endif
			break;

		case MSPIM3:
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
#endif
			break;

		default:
			break;
	}
	/* If error, return 0xFF */
	return 0xFF;
}

uint8_t avrSerialxCheckRxComplete(const xComPortHandlePtr pxPort )
{
	switch (pxPort->usart)
	{
		case USART0:
		case MSPIM0:
			return( UCSR0A & (_BV(RXC0)) );			// nonzero if serial data is available to read.
			break;

		case USART1:
		case MSPIM1:
#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
			return( UCSR1A & (_BV(RXC1)) );			// nonzero if serial data is available to read.
#endif
			break;

		case USART2:
		case MSPIM2:
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
			return( UCSR2A & (_BV(RXC2)) );			// nonzero if serial data is available to read.
#endif
			break;

		case USART3:
		case MSPIM3:
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
			return( UCSR3A & (_BV(RXC3)) );			// nonzero if serial data is available to read.
#endif
			break;

		default:
			break;
	}
	return 0;
}

uint8_t avrSerialxCheckTxReady(const xComPortHandlePtr pxPort )
{
	switch (pxPort->usart)
	{
		case USART0:
		case MSPIM0:
			return( UCSR0A & (_BV(UDRE0)) );		// nonzero if transmit register is ready to receive new data.
			break;

		case USART1:
		case MSPIM1:
#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
			return( UCSR1A & (_BV(UDRE1)) );		// nonzero if transmit register is ready to receive new data.
#endif
			break;

		case USART2:
		case MSPIM2:
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
			return( UCSR1A & (_BV(UDRE1)) );		// nonzero if transmit register is ready to receive new data.
#endif
			break;

		case USART3:
		case MSPIM3:
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
			return( UCSR1A & (_BV(UDRE1)) );		// nonzero if transmit register is ready to receive new data.
#endif
			break;

		default:
			break;
	}
	return 0;
}

/*-----------------------------------------------------------*/
/* -------------------- INTERRUPTS --------------------------*/
/*-----------------------------------------------------------*/

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
ISR( USART_RX_vect ) __attribute__ ((hot, flatten));
ISR( USART_RX_vect )

#elif defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
ISR( USART0_RX_vect ) __attribute__ ((hot, flatten));
ISR( USART0_RX_vect )

#endif
{
	/* Get status and data from buffer */

	/* If error bit set (Frame Error, Data Over Run, Parity), flush and return nothing */
	if ( (UCSR0A & (_BV(FE0)|_BV(DOR0)|_BV(UPE0)) ) )
	{
		register uint8_t erroredByte __attribute__ ((unused));
		while ( UCSR0A & (1<<RXC0) ) erroredByte = UDR0;
	}
	else
	{
		/* If no error, get the character and post it on the buffer of Rxed characters.*/
		register uint8_t cChar = UDR0;

		if( ! ringBuffer_IsFull( (ringBuffer_t*) &(xSerialPort.xRxedChars) ) )
			ringBuffer_Poke( (ringBuffer_t*) &(xSerialPort.xRxedChars), cChar);
	}
}
/*-----------------------------------------------------------*/

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
ISR( USART_UDRE_vect ) __attribute__ ((hot, flatten));
ISR( USART_UDRE_vect )

#elif defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
ISR( USART0_UDRE_vect ) __attribute__ ((hot, flatten));
ISR( USART0_UDRE_vect )

#endif
{
	if( ringBuffer_IsEmpty( (ringBuffer_t*) &(xSerialPort.xCharsForTx) ) )
	{
		// Queue empty, nothing to send.
		vInterrupt0_Off();
	}
	else
	{
		UDR0 = ringBuffer_Pop( (ringBuffer_t*) &(xSerialPort.xCharsForTx) );
	}
}
/*-----------------------------------------------------------*/

#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)

ISR( USART1_RX_vect ) __attribute__ ((hot, flatten));
ISR( USART1_RX_vect )
{
	/* Get status and data from buffer */

	/* If error bit set (Frame Error, Data Over Run, Parity), flush and return nothing */
	if ( (UCSR1A &  (_BV(FE1)|_BV(DOR1)|_BV(UPE1)) ) )
	{
		register uint8_t erroredByte __attribute__ ((unused));
		while ( UCSR1A & (1<<RXC1) ) erroredByte = UDR1;
	}
	else
	{
		/* If no error, get the character and post it on the buffer of Rxed characters.*/
		register uint8_t cChar = UDR1;

		if( ! ringBuffer_IsFull( (ringBuffer_t*) &(xSerial1Port.xRxedChars) ) )
			ringBuffer_Poke( (ringBuffer_t*) &(xSerial1Port.xRxedChars), cChar);
	}
}
/*-----------------------------------------------------------*/

ISR( USART1_UDRE_vect ) __attribute__ ((hot, flatten));
ISR( USART1_UDRE_vect )
{
	if( ringBuffer_IsEmpty( (ringBuffer_t*) &(xSerial1Port.xCharsForTx) ) )
	{
		// Queue empty, nothing to send.
		vInterrupt1_Off();
	}
	else
	{
		UDR1 = ringBuffer_Pop( (ringBuffer_t*) &(xSerial1Port.xCharsForTx) );
	}
}
/*-----------------------------------------------------------*/

#endif


#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)

ISR( USART2_RX_vect ) __attribute__ ((hot, flatten));
ISR( USART2_RX_vect )
{
	/* Get status and data from buffer */

	/* If error bit set (Frame Error, Data Over Run, Parity), flush and return nothing */
	if ( (UCSR2A & (_BV(FE2)|_BV(DOR2)|_BV(UPE2)) ) )
	{
		register uint8_t erroredByte __attribute__ ((unused));
		while ( UCSR2A & (1<<RXC2) ) erroredByte = UDR2;
	}
	else
	{
		/* If no error, get the character and post it on the buffer of Rxed characters.*/
		register uint8_t cChar = UDR2;

		if( ! ringBuffer_IsFull( (ringBuffer_t*) &(xSerial2Port.xRxedChars) ) )
			ringBuffer_Poke( (ringBuffer_t*) &(xSerial2Port.xRxedChars), cChar);
	}
}
/*-----------------------------------------------------------*/

ISR( USART2_UDRE_vect ) __attribute__ ((hot, flatten));
ISR( USART2_UDRE_vect )
{
	if( ringBuffer_IsEmpty( (ringBuffer_t*) &(xSerial2Port.xCharsForTx) ) )
	{
		// Queue empty, nothing to send.
		vInterrupt2_Off();
	}
	else
	{
		UDR2 = ringBuffer_Pop( (ringBuffer_t*) &(xSerial2Port.xCharsForTx) );
	}
}
/*-----------------------------------------------------------*/

ISR( USART3_RX_vect ) __attribute__ ((hot, flatten));
ISR( USART3_RX_vect )
{
	/* Get status and data from buffer */

	/* If error bit set (Frame Error, Data Over Run, Parity), flush and return nothing */
	if ( (UCSR3A & (_BV(FE3)|_BV(DOR3)|_BV(UPE3)) ) )
	{
		register uint8_t erroredByte __attribute__ ((unused));
		while ( UCSR3A & (1<<RXC3) ) erroredByte = UDR3;
	}
	else
	{
		/* If no error, get the character and post it on the buffer of Rxed characters.*/
		register uint8_t cChar = UDR3;

		if( ! ringBuffer_IsFull( (ringBuffer_t*) &(xSerial3Port.xRxedChars) ) )
			ringBuffer_Poke( (ringBuffer_t*) &(xSerial3Port.xRxedChars), cChar);
	}
}
/*-----------------------------------------------------------*/

ISR( USART3_UDRE_vect ) __attribute__ ((hot, flatten));
ISR( USART3_UDRE_vect )
{
	if( ringBuffer_IsEmpty( (ringBuffer_t*) &(xSerial3Port.xCharsForTx) ) )
	{
		// Queue empty, nothing to send.
		vInterrupt3_Off();
	}
	else
	{
		UDR3 = ringBuffer_Pop( (ringBuffer_t*) &(xSerial3Port.xCharsForTx) );
	}
}
/*-----------------------------------------------------------*/

#endif
