/*
 * eefs_avrspi.c
 *
 *  Created on: 04/01/2015
 *      Author: phillip
 */
/*
 * Includes
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <avr/io.h>
#include <util/delay.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "semphr.h"

/* SPI interface include file. */
#include "spi.h"

#include "common_types.h"

#include "eefs_avrspi.h"


/* Declare a binary Semaphore flag for the EEFS. To ensure only single access to the EEFS. */
SemaphoreHandle_t xEEFSSemaphore = NULL;

/* Use the previously declared a binary Semaphore flag for the SPI Bus. To ensure only single access to SPI Bus. */
extern SemaphoreHandle_t xSPISemaphore;

int8_t eefs_avrspi_begin(void)
{
	spiSetDataMode(SPI_MODE0);			// Enable SPI function in mode 0
	spiSetClockDivider(SPI_CLOCK_DIV2);	// SPI at maximum speed
	spiBegin(Default);

#if defined (RAM0)
	RAM_DDR |= _BV(RAM0_SS);		// Set the RAM0 SS to Output
	RAM_PORT |= _BV(RAM0_SS);	// Set the RAM0 SS to High
#endif

#if defined (RAM1)
	RAM_DDR |= _BV(RAM1_SS); 	// Set the RAM1 SS to Output
	RAM_PORT |= _BV(RAM1_SS);	// Set the RAM1 SS to High
#endif

    if( xEEFSSemaphore == NULL ) 					/* Check to see if the semaphore has not been created. */
    {
    	xEEFSSemaphore = xSemaphoreCreateBinary();	/* Then create the EEFS Chip binary semaphore */
		if( ( xEEFSSemaphore ) != NULL )
		{
			xSemaphoreGive( xEEFSSemaphore );		/* make it available */
		}
		else
		{
			return EEFS_BUS_IS_NOT_READY;}
    	}

    return EEFS_SUCCESS;
}

void eefs_avrspi_lock(void)
{
	xSemaphoreTake( xEEFSSemaphore, (SPI_TIMEOUT / portTICK_PERIOD_MS ) );
}

void eefs_avrspi_unlock(void)
{
	xSemaphoreGive( xEEFSSemaphore );
}

int8_t eefs_avrspi_read(uint8_t * Dest, const addr_farptr_t Src, size_t Length)
{
	size_t index = 0;
	register uint8_t TxRxByte;
	int8_t ReturnCode = EEFS_SUCCESS;

	// Length is 0 so just return
	if( Length == 0 ) return EEFS_SUCCESS;

	// If the SPI module has not been enabled yet, then return with nothing.
	if( !(SPCR & _BV(SPE)) ) return EEFS_BUS_IS_NOT_READY;

	// The SPI module is enabled, but it is in slave mode, so we can not
	// transmit the byte. This can happen if SSbar is an input and it went low.
	// We will try to recover by setting the MSTR bit.
	if( !(SPCR & _BV(MSTR)) )
		{
			SPCR |= _BV(MSTR);
			if( !(SPCR & _BV(MSTR)) ) return EEFS_BUS_IS_NOT_READY;
		}

	if( (xSemaphoreTake( xSPISemaphore, (SPI_TIMEOUT / portTICK_PERIOD_MS )) == pdTRUE ) )
	{

		switch( Src.bAddr.device_byte )
		{
		case RAM0_DEVICE_ADDR:
#if defined (RAM0)

#if defined (RAM0_BUSY_MODE)
			TxRxByte = MODE_BUSY;
			while((TxRxByte & MODE_BUSY) != 0x00)
			{
				RAM_PORT &= ~_BV(RAM0_SS);	// Set the RAM0 SS to Low
				SPDR = RDSR; 			// Read the Status Register
				while ( !(SPSR & _BV(SPIF)) );
				SPDR = 0xFF;			// Transmit dummy Byte
				while ( !(SPSR & _BV(SPIF)) );
				TxRxByte = SPDR;
				RAM_PORT |= _BV(RAM0_SS);	// Set the RAM0 SS to High
				if((TxRxByte & MODE_BUSY) != 0x00)
					_delay_ms(2);
			}
#endif

			RAM_PORT &= ~_BV(RAM0_SS);	// Set the RAM0 SS to Low

			// Transmit the mode command
			SPDR = READ;

#if (RAM0_ADDR_BITS > 16)
			// Prepare & transmit the Address High Byte if warranted
			TxRxByte = Src.bAddr.high_byte;
			while ( !(SPSR & _BV(SPIF)) );
			SPDR = TxRxByte;
#endif

			// Prepare & transmit the Address Mid Byte
			TxRxByte = Src.bAddr.mid_byte;
			while ( !(SPSR & _BV(SPIF)) );
			SPDR = TxRxByte;

			// Prepare & transmit the Address Low Byte
			TxRxByte = Src.bAddr.low_byte;
			while ( !(SPSR & _BV(SPIF)) );
			SPDR = TxRxByte;

			while ( !(SPSR & _BV(SPIF)) );
			SPDR = 0xFF; // Begin dummy transmission
			while( index < Length - 1 )
			{
				while ( !(SPSR & _BV(SPIF)) );
				TxRxByte = SPDR; // copy received byte
				SPDR = 0xFF;   // Continue dummy transmission
				Dest[ index++ ] = TxRxByte;
			}
			while( !(SPSR & _BV(SPIF)) );
			Dest[ index ] = SPDR;	// store the last byte that was read

			RAM_PORT |= _BV(RAM0_SS);	// Set the RAM0 SS to High

			ReturnCode = EEFS_SUCCESS;
#else
			ReturnCode = EEFS_NO_SUCH_DEVICE;
#endif
			break;

		case RAM1_DEVICE_ADDR:
#if defined (RAM1)

#if defined (RAM1_BUSY_MODE)
			TxRxByte = MODE_BUSY;
			while((TxRxByte & MODE_BUSY) != 0x00)
			{
				RAM_PORT &= ~_BV(RAM1_SS);	// Set the RAM1 SS to Low
				SPDR = RDSR; 			// Read the Status Register
				while ( !(SPSR & _BV(SPIF)) );
				SPDR = 0xFF;			// Transmit dummy Byte
				while ( !(SPSR & _BV(SPIF)) );
				TxRxByte = SPDR;
				RAM_PORT |= _BV(RAM1_SS);	// Set the RAM1 SS to High
				if((TxRxByte & MODE_BUSY) != 0x00) _delay_ms(2);
			}
#endif

			RAM_PORT &= ~_BV(RAM1_SS);	// Set the RAM1 SS to Low

			// Transmit the mode command
			SPDR = READ;

#if (RAM1_ADDR_BITS > 16)
			// Prepare & transmit the Address High Byte if warranted
			TxRxByte = Src.bAddr.high_byte;
			while ( !(SPSR & _BV(SPIF)) );
			SPDR = TxRxByte;
#endif

			// Prepare & transmit the Address Mid Byte
			TxRxByte = Src.bAddr.mid_byte;
			while ( !(SPSR & _BV(SPIF)) );
			SPDR = TxRxByte;

			// Prepare & transmit the Address Low Byte
			TxRxByte = Src.bAddr.low_byte;
			while ( !(SPSR & _BV(SPIF)) );
			SPDR = TxRxByte;

			while ( !(SPSR & _BV(SPIF)) );
			SPDR = 0xFF; // Begin dummy transmission
			while( index < Length - 1 )
			{
				while ( !(SPSR & _BV(SPIF)) );
				TxRxByte = SPDR; // copy received byte
				SPDR = 0xFF;   // Continue dummy transmission
				Dest[ index++ ] = TxRxByte;
			}
			while( !(SPSR & _BV(SPIF)) );
			Dest[ index ] = SPDR;	// store the last byte that was read

			RAM_PORT |= _BV(RAM1_SS);	// Set the RAM1 SS to High

			ReturnCode = EEFS_SUCCESS;
#else
			ReturnCode = EEFS_NO_SUCH_DEVICE;
#endif
			break;

		default:
			ReturnCode = EEFS_NO_SUCH_DEVICE;
			break;
		}
		xSemaphoreGive( xSPISemaphore );	/* Free SPI semaphore to allow other SPI access */
	}
	else
		ReturnCode = EEFS_BUS_IS_NOT_READY;

	return ReturnCode;
}


int8_t eefs_avrspi_write(const addr_farptr_t Dest, const uint8_t * Src, size_t Length)
{
	size_t index = 0;
	register uint8_t TxRxByte;
	int8_t ReturnCode = EEFS_SUCCESS;

	// Length is 0 so just return
	if( Length == 0 ) return EEFS_SUCCESS;

	// If the SPI module has not been enabled yet, then return.
	if( !(SPCR & _BV(SPE)) ) return EEFS_BUS_IS_NOT_READY;

	// The SPI module is enabled, but it is in slave mode, so we can not
	// transmit the byte. This can happen if SSbar is an input and it went low.
	// We will try to recover by setting the MSTR bit.
	if( !(SPCR & _BV(MSTR)) )
		{
			SPCR |= _BV(MSTR);
			if( !(SPCR & _BV(MSTR)) ) return EEFS_BUS_IS_NOT_READY;
		}

	if( (xSemaphoreTake( xSPISemaphore, (SPI_TIMEOUT / portTICK_PERIOD_MS )) == pdTRUE ) )
	{
		switch( Dest.bAddr.device_byte )
		{
		case RAM0_DEVICE_ADDR:
#if defined (RAM0)

#if defined (RAM0_BUSY_MODE)
			TxRxByte = MODE_BUSY;
			while((TxRxByte & MODE_BUSY) != 0x00)
			{
				RAM_PORT &= ~_BV(RAM0_SS);	// Set the RAM0 SS to Low
				SPDR = RDSR; 			// Read the Status Register
				while ( !(SPSR & _BV(SPIF)) );
				SPDR = 0xFF;			// Transmit dummy Byte
				while ( !(SPSR & _BV(SPIF)) );
				TxRxByte = SPDR;
				RAM_PORT |= _BV(RAM0_SS);	// Set the RAM0 SS to High
				if((TxRxByte & MODE_BUSY) != 0x00) _delay_ms(2);
			}
#endif

#if defined (RAM0_WRITE_LATCH)
			RAM_PORT &= ~_BV(RAM0_SS);	// Set the RAM0 SS to Low
			// Transmit the Write Latch command
			SPDR = WREN;
			while ( !(SPSR & _BV(SPIF)) );
			RAM_PORT |= _BV(RAM0_SS);	// Set the RAM0 SS to High
#endif

			RAM_PORT &= ~_BV(RAM0_SS);	// Set the RAM0 SS to Low

			// Transmit the mode command
			SPDR = WRITE;

#if (RAM0_ADDR_BITS > 16)
			// Prepare & transmit the Address High Byte if warranted
			TxRxByte = Dest.bAddr.high_byte;
			while ( !(SPSR & _BV(SPIF)) );
			SPDR = TxRxByte;
#endif

			// Prepare & transmit the Address Mid Byte
			TxRxByte = Dest.bAddr.mid_byte;
			while ( !(SPSR & _BV(SPIF)) );
			SPDR = TxRxByte;

			// Prepare & transmit the Address Low Byte
			TxRxByte = Dest.bAddr.low_byte;
			while ( !(SPSR & _BV(SPIF)) );
			SPDR = TxRxByte;

			while ( !(SPSR & _BV(SPIF)) );
			SPDR = Src[ index++ ]; // Begin transmission from the Src to the Dest
			while( index < Length )
			{
				TxRxByte = Src[ index++ ]; // pre-load the byte to be transmitted
				while ( !(SPSR & _BV(SPIF)) );
				SPDR = TxRxByte; // Continue transmission
			}
			while( !(SPSR & _BV(SPIF)) );

			RAM_PORT |= _BV(RAM0_SS);	// Set the RAM0 SS to High

			ReturnCode = EEFS_SUCCESS;
#else
			ReturnCode = EEFS_NO_SUCH_DEVICE;
#endif
			break;

		case RAM1_DEVICE_ADDR:
#if defined (RAM1)

#if defined (RAM1_BUSY_MODE)
			TxRxByte = MODE_BUSY;
			while((TxRxByte & MODE_BUSY) != 0x00)
			{
				RAM_PORT &= ~_BV(RAM1_SS);	// Set the RAM1 SS to Low
				SPDR = RDSR; 			// Read the Status Register
				while ( !(SPSR & _BV(SPIF)) );
				SPDR = 0xFF;			// Transmit dummy Byte
				while ( !(SPSR & _BV(SPIF)) );
				TxRxByte = SPDR;
				RAM_PORT |= _BV(RAM1_SS);	// Set the RAM1 SS to High
				if((TxRxByte & MODE_BUSY) != 0x00) _delay_ms(2);
			}
#endif

#if defined (RAM1_WRITE_LATCH)
			RAM_PORT &= ~_BV(RAM1_SS);	// Set the RAM1 SS to Low
			// Transmit the Write Latch command
			SPDR = WREN;
			while ( !(SPSR & _BV(SPIF)) );
			RAM_PORT |= _BV(RAM1_SS);	// Set the RAM1 SS to High
#endif

			RAM_PORT &= ~_BV(RAM1_SS);	// Set the RAM1 SS to Low

			// Transmit the mode command
			SPDR = WRITE;

#if (RAM1_ADDR_BITS > 16)
			// Prepare & transmit the Address High Byte if warranted
			TxRxByte = Dest.bAddr.high_byte;
			while ( !(SPSR & _BV(SPIF)) );
			SPDR = TxRxByte;
#endif

			// Prepare & transmit the Address Mid Byte
			TxRxByte = Dest.bAddr.mid_byte;
			while ( !(SPSR & _BV(SPIF)) );
			SPDR = TxRxByte;

			// Prepare & transmit the Address Low Byte
			TxRxByte = Dest.bAddr.low_byte;
			while ( !(SPSR & _BV(SPIF)) );
			SPDR = TxRxByte;

			while ( !(SPSR & _BV(SPIF)) );
			SPDR = Src[ index++ ]; // Begin transmission from the Src to the Dest
			while( index < Length )
			{
				TxRxByte = Src[ index++ ]; // pre-load the byte to be transmitted
				while ( !(SPSR & _BV(SPIF)) );
				SPDR = TxRxByte; // Continue transmission
			}
			while( !(SPSR & _BV(SPIF)) );

			RAM_PORT |= _BV(RAM1_SS);	// Set the RAM1 SS to High

			ReturnCode = EEFS_SUCCESS;
#else
			ReturnCode = EEFS_NO_SUCH_DEVICE;
#endif
			break;

		default:
			ReturnCode = EEFS_NO_SUCH_DEVICE;
			break;
		}
		xSemaphoreGive( xSPISemaphore );	/* Free SPI semaphore to allow other SPI access */
	}
	else
		ReturnCode = EEFS_BUS_IS_NOT_READY;

	return ReturnCode;
}



