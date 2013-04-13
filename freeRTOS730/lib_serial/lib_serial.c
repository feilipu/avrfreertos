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

/* BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER. */
/* Also with polling serial functions, for use before scheduler is enabled */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include <util/delay.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <lib_serial.h>


/*-----------------------------------------------------------*/

#define vInterruptOn()										\
{															\
	uint8_t ucByte;								    		\
															\
	ucByte  = UCSR0B;										\
	ucByte |= _BV(UDRIE0);									\
	UCSR0B  = ucByte;										\
}


#define vInterruptOff()										\
{															\
	uint8_t ucByte;								   			\
															\
	ucByte  = UCSR0B;										\
	ucByte &= ~(_BV(UDRIE0));								\
	UCSR0B  = ucByte;										\
}

/*-----------------------------------------------------------*/
static xQueueHandle xRxedChars;
static xQueueHandle xCharsForTx;
static unsigned portBASE_TYPE *serialWorkBuffer; // create a working buffer pointer, to later be malloc() on the heap.

/* Create a handle for the serial port. */
xComPortHandle xSerialPort;

/*-----------------------------------------------------------------*/

// xSerialPrintf_P(PSTR("\r\nMessage %u %u %u"), var1, var2, var2);

void xSerialPrintf( const char * format, ...)
{
	va_list arg;

	va_start(arg, format);
	vsnprintf((char *)serialWorkBuffer, portSERIAL_BUFFER, (const char *)format, arg);
	xSerialPrint((uint8_t *)serialWorkBuffer);
	va_end(arg);
}

void xSerialPrintf_P(PGM_P format, ...)
{
	va_list arg;

	va_start(arg, format);
	vsnprintf_P((char *)serialWorkBuffer, portSERIAL_BUFFER, format, arg);
	xSerialPrint((uint8_t *)serialWorkBuffer);
	va_end(arg);
}

void xSerialPrint( uint8_t * str)
{
	int16_t i = 0;
	size_t stringlength;

	stringlength = strlen((char *)str);

	while(i < stringlength)
		xSerialPutChar( xSerialPort, str[i++], xNoBlock );
}

void xSerialPrint_P(PGM_P str)
{
	uint16_t i = 0;
	size_t stringlength;

	stringlength = strlen_P(str);

	while(i < stringlength)
		xSerialPutChar( xSerialPort, pgm_read_byte(&str[i++]), xNoBlock );
}

/*-----------------------------------------------------------*/

inline portBASE_TYPE xSerialGetChar( xComPortHandle pxPort, unsigned portBASE_TYPE *pcRxedChar, portTickType xBlockTime )
{
	/* Only one port is supported. */
	( void ) pxPort;

	/* Get the next character from the buffer.  Return false if no characters
	are available, or arrive before xBlockTime expires. */
	if( xQueueReceive( xRxedChars, pcRxedChar, xBlockTime ) )
	{
		return pdTRUE;
	}
	else
	{
		return pdFALSE;
	}
}


inline portBASE_TYPE xSerialPutChar( xComPortHandle pxPort, unsigned portBASE_TYPE cOutChar, portTickType xBlockTime )
{
	/* Only one port is supported. */
	( void ) pxPort;

	/* Return false if after the block time there is no room on the Tx queue. */
	if( xQueueSendToBack( xCharsForTx, &cOutChar, xBlockTime ) != pdPASS )
	{
		return pdFAIL;
	}

	vInterruptOn();

	return pdPASS;
}

/*-----------------------------------------------------------*/

xComPortHandle xSerialPortInitMinimal( uint32_t ulWantedBaud, unsigned portBASE_TYPE uxTxQueueLength, unsigned portBASE_TYPE uxRxQueueLength )
{
uint32_t ulBaudRateCounter;
uint8_t ucByte;

	portENTER_CRITICAL();
	{
		/* Create the queues used by the serial communications task. */
		xRxedChars = xQueueCreate( uxRxQueueLength, (unsigned portBASE_TYPE) sizeof(unsigned portBASE_TYPE) );
		xCharsForTx = xQueueCreate( uxTxQueueLength, (unsigned portBASE_TYPE) sizeof(unsigned portBASE_TYPE) );

		// create a working buffer for vsnprintf on the heap (so we can use extended RAM, if available).
		// create the structures on the heap (so they can be moved later).
		if(serialWorkBuffer == NULL) // if there is no Line buffer allocated (pointer is NULL), then allocate buffer.
			if( !(serialWorkBuffer = (unsigned portBASE_TYPE *)pvPortMalloc(sizeof(unsigned portBASE_TYPE) * portSERIAL_BUFFER)))
				return NULL;

		/* Calculate the baud rate register value from the equation in the
		data sheet. */

		/* As the 16MHz Arduino boards have bad karma for serial port, we're using the 2x clock U2X0
		 */
		// for Arduino at 16MHz; above data sheet calculation is wrong. Need below from <util/setbaud.h>
		// This provides correct rounding truncation to get closest to correct speed.
		// Normal mode gives 3.7% error, which is too much. Use 2x mode gives 2.1% error.
		// Or, use 22.1184 MHz overclock which gives 0.00% error, for all rates.

		//ulBaudRateCounter = ((configCPU_CLOCK_HZ + ulWantedBaud * 8UL) / (ulWantedBaud * 16UL) - 1); // for normal mode
		ulBaudRateCounter = ((configCPU_CLOCK_HZ + ulWantedBaud * 4UL) / (ulWantedBaud * 8UL) - 1);  // for 2x mode

		/* Set the baud rate. */
		ucByte = ( uint8_t ) ( ulBaudRateCounter & ( uint32_t ) 0xff );
		UBRR0L = ucByte;

		ulBaudRateCounter >>= ( uint32_t ) 8;
		ucByte = ( uint8_t ) ( ulBaudRateCounter & ( uint32_t ) 0xff );
		UBRR0H = ucByte;

		/* Set the 2x speed mode bit */
		UCSR0A = _BV(U2X0);

		/* Enable the Rx and Tx. Also enable the Rx interrupt. The Tx interrupt will get enabled later. */
		UCSR0B = ( _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0));

		/* Set the data bit register to 8n1. */
		UCSR0C = ( _BV(UCSZ00) | _BV(UCSZ01) );

	}
	portEXIT_CRITICAL();

	/* Unlike other ports, this serial code does not allow for more than one
	com port.  We therefore don't return a pointer to a port structure and can
	instead just return NULL. */
	return NULL;
}


void vSerialClose( xComPortHandle xPort )
{
	uint8_t ucByte;

	/* The parameter is not used. */
	( void ) xPort;

	/* Turn off the interrupts.  We may also want to delete the queues and/or
	re-install the original ISR. */

	vPortFree (serialWorkBuffer);
	vQueueDelete(xRxedChars);
	vQueueDelete(xCharsForTx);

	portENTER_CRITICAL();
	{
		vInterruptOff();
		ucByte = UCSR0B;
		ucByte &= ~(_BV(RXCIE0));
		UCSR0B = ucByte;
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
	vsnprintf((char *)serialWorkBuffer, portSERIAL_BUFFER, (const char *)format, arg);
	avrSerialPrint(serialWorkBuffer);
	va_end(arg);
}

void avrSerialPrintf_P(PGM_P format, ...)
{
	va_list arg;

	va_start(arg, format);
	vsnprintf_P((char *)serialWorkBuffer, portSERIAL_BUFFER, format, arg);
	avrSerialPrint(serialWorkBuffer);
	va_end(arg);
}

void avrSerialPrint(uint8_t * str)
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

inline void avrSerialWrite(int8_t DataOut)
{
	while (!avrSerialCheckTxReady())		// while NOT ready to transmit
        _delay_ms(1);     					// delay
	UDR0 = DataOut;
}

inline int8_t avrSerialRead(void)
{

	while (!avrSerialCheckRxComplete())		// While data is NOT available to read
		_delay_ms(1);     					// delay
	/* Get status and data */
	/* from buffer */

	/* If error, return 0xFF */
	if ( UCSR0A & ((1<<FE0)|(1<<DOR0)|(1<<UPE0)) )
	return 0xFF;
	else
	return UDR0;
}

inline int8_t avrSerialCheckRxComplete(void)
{
	return( UCSR0A & (1 << RXC0) );			// nonzero if serial data is available to read.
}

inline int8_t avrSerialCheckTxReady(void)
{
	return( UCSR0A & (1 << UDRE0) );		// nonzero if transmit register is ready to receive new data.
}


/*-----------------------------------------------------------*/
#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
ISR( USART0_RX_vect )
#elif defined(__AVR_ATmega324P__)  || defined(__AVR_ATmega644P__)|| defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__)
ISR( USART0_RX_vect )
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
ISR( USART_RX_vect )
#endif
{
	signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	uint8_t cChar;

	/* Get status and data */
	/* from buffer */

	portENTER_CRITICAL();

	/* If error it set (Frame Error, Data Over Run, Parity), return 0xFF */
	if ( UCSR0A & ((1<<FE0)|(1<<DOR0)|(1<<UPE0)) )
		cChar = 0xFF;
	else
		/* Get the character and post it on the queue of Rxed characters.
		If the post causes a task to wake force a context switch as the awoken task
		may have a higher priority than the task we have interrupted. */
		cChar = UDR0;

	xQueueSendToBackFromISR( xRxedChars, &cChar, &xHigherPriorityTaskWoken );

	portEXIT_CRITICAL();


	if( xHigherPriorityTaskWoken != pdFALSE )
	{
		taskYIELD();
	}
}
/*-----------------------------------------------------------*/

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
ISR( USART0_UDRE_vect )
#elif defined(__AVR_ATmega324P__)  || defined(__AVR_ATmega644P__)|| defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__)
ISR( USART0_UDRE_vect )
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
ISR( USART_UDRE_vect )
#endif
{
	uint8_t cChar;
	signed portBASE_TYPE cTaskWoken;

	portENTER_CRITICAL();

	if( xQueueReceiveFromISR( xCharsForTx, &cChar, &cTaskWoken ) == pdTRUE )
	{
		/* Send the next character queued for Tx. */
		UDR0 = cChar;
	}
	else
	{
		/* Queue empty, nothing to send. */
		vInterruptOff();
	}

	portEXIT_CRITICAL();

}
