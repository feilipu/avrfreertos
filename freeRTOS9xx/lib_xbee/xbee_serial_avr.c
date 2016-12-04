/**
	@addtogroup hal_avr
	@{
	@file xbee_serial_avr.c

	Serial Interface for XBee Module (AVR Platform)

*/
// NOTE: Documentation for these functions can be found in xbee/serial.h.
/*** BeginHeader */

#include <limits.h>
#include <errno.h>
#include <stdio.h>

#include "serial.h"

#include "xbee/xbee_platform.h"
#include "xbee/xbee_device.h"
#include "xbee/xbee_serial.h"

// Could change XBEE_SER_CHECK to an assert, or even ignore it if not in debug.
#define XBEE_SER_CHECK(ptr)   \
   do { if (xbee_ser_invalid(ptr)) return -EINVAL; } while (0)
/*** EndHeader */

/*** BeginHeader xbee_ser_invalid */
/*** EndHeader */
bool_t xbee_ser_invalid( xbee_serial_t *serial)
{
	bool_t disabled;

	if (serial)
	{
		disabled = 1;
		switch (serial->usart)
		{
			case USART0:
				disabled = FALSE;
				break;

#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
			case USART1:
				disabled = FALSE;
				break;
#endif

			default:
				disabled = TRUE;
				break;
		}

		#ifdef XBEE_SERIAL_VERBOSE
	      if (disabled)
	      {
	    	  xSerialxPrintf_P( &xSerial1Port, PSTR("ERROR: Support for port %c not compiled in.  " \
					"Define XBEE_ON_USART%c and recompile.\n"), serial->usart + '0',
					serial->usart + '0');
	      }
	   #endif
		return disabled;
	}
	return TRUE;
}

/*** BeginHeader xbee_ser_portname */
/*** EndHeader */
const char *xbee_ser_portname( xbee_serial_t *serial)
{
	if (serial)
	{
		switch (serial->usart)
		{
			case USART0:
				return "USART0";
				break;

#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
			case USART1:
				return "USART1";
				break;
#endif

			default:
				return "(invalid)";
				break;
		}
	}
	return "(invalid)";
}

/*** BeginHeader xbee_ser_write */
/*** EndHeader */
int xbee_ser_write( xbee_serial_t *serial, const void FAR *buffer, int length)
{
	int16_t written;
	UBaseType_t *buff = (UBaseType_t *)buffer;

	XBEE_SER_CHECK( serial );

	if (! buffer || length < 0)
	{
		return -EINVAL;
	}

	for( written = 0; written < length; ++written )
	{
		if( !(xSerialPutChar( serial, buff[written] )))	// if there's an error
				return -EIO;							// return an error
	}

	return written;										// otherwise return bytes written.
}

/*** BeginHeader xbee_ser_read*/
/*** EndHeader */
int xbee_ser_read( xbee_serial_t *serial, void FAR *buffer, int bufsize)
{
	int16_t read;
	UBaseType_t *buff = (UBaseType_t *)buffer;

	XBEE_SER_CHECK( serial );

	if (! buffer || bufsize < 0)
	{
		return -EINVAL;
	}

	for( read = 0; read < bufsize; ++read )
	{
		if( !(xSerialGetChar( serial, &buff[read] )))	// if there's an error, because we have no bytes
				return -EIO;							// return an error
	}

	return read;										// otherwise return bytes read.
}

/*** BeginHeader xbee_ser_putchar */
/*** EndHeader */
int xbee_ser_putchar( xbee_serial_t *serial, uint8_t ch)
{
	bool_t error;

	XBEE_SER_CHECK( serial);

	error = xSerialPutChar( serial, (UBaseType_t)ch);

	if (error == FALSE)
		return 0;
	else
		return -ENOSPC;
}

/*** BeginHeader xbee_ser_getchar */
/*** EndHeader */
int xbee_ser_getchar( xbee_serial_t *serial)
{
	UBaseType_t ch;
	bool_t error;

	XBEE_SER_CHECK( serial);

	error = xSerialGetChar( serial, &ch);

	if (error == FALSE)
		return (int)ch;
	else
		return -ENODATA;
}

/*** BeginHeader xbee_ser_tx_free */
/*** EndHeader */
int xbee_ser_tx_free( xbee_serial_t *serial)
{
	XBEE_SER_CHECK( serial);

	return (int)(serial->xCharsForTx.size - serial->xCharsForTx.count );
}

/*** BeginHeader xbee_ser_tx_used */
/*** EndHeader */
int xbee_ser_tx_used( xbee_serial_t *serial)
{
	XBEE_SER_CHECK( serial );

	return (int)( serial->xCharsForTx.count );
}

/*** BeginHeader xbee_ser_tx_flush */
/*** EndHeader */
int xbee_ser_tx_flush( xbee_serial_t *serial)
{
	XBEE_SER_CHECK( serial );

	xSerialTxFlush( serial );
	return 0;
}

/*** BeginHeader xbee_ser_rx_free */
/*** EndHeader */
int xbee_ser_rx_free( xbee_serial_t *serial)
{
	XBEE_SER_CHECK( serial );

	return (int)(serial->xRxedChars.size - serial->xRxedChars.count );
}

/*** BeginHeader xbee_ser_rx_used */
/*** EndHeader */
int xbee_ser_rx_used( xbee_serial_t *serial)
{
	XBEE_SER_CHECK( serial);

// return bytes used in rx buffer -- will this be difficult on some platforms
// (like PC) where we can't peek at the stream?  It may be necessary to create
// a small buffer for the program to buffer up to 512 bytes from COM port.
	return (int)( serial->xRxedChars.count );
}

/*** BeginHeader xbee_ser_rx_flush */
/*** EndHeader */
int xbee_ser_rx_flush( xbee_serial_t *serial)
{
	XBEE_SER_CHECK( serial);

	xSerialRxFlush( serial );
	return 0;
}

/*** BeginHeader xbee_ser_open */
/*** EndHeader */
int xbee_ser_open( xbee_serial_t *serial, uint32_t baudrate)
{
	XBEE_SER_CHECK( serial);

	if (xbee_ser_invalid(serial))
	{
		return -EIO;
	}
	else
	{
		return 0;
	}
}

/*** BeginHeader xbee_ser_close */
/*** EndHeader */
int xbee_ser_close( xbee_serial_t *serial)
{
	XBEE_SER_CHECK( serial);

	vSerialClose (serial);

	return 0;
}

/*** BeginHeader xbee_ser_break */
/*** EndHeader */
int xbee_ser_break( xbee_serial_t *serial, bool_t enabled)
{
	XBEE_SER_CHECK( serial);

	if (enabled)
	{
		switch ( serial->usart )
		{
			case USART0:
				/* Disable the Tx */
				UCSR0B &= ~_BV(UDRIE0);
				UCSR0B &= ~_BV(TXEN0);
				/* Pull it low to send Break */
				DDRD |= _BV(DDD1);
				PORTD &= ~_BV(PIN1);
				break;

	#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
			case USART1:
				/* Disable the Tx */
				UCSR1B &= ~_BV(UDRIE1);
				UCSR1B &= ~ _BV(TXEN1);
				/* Pull it low to send Break */
				DDRD |= _BV(DDD3);
				PORTD &= ~_BV(PIN3);
				break;
	#endif

			default:
				return -EINVAL;
				break;
		}
	}
	else
	{
		switch ( serial->usart )
		{
			case USART0:
				/* Enable the Tx. */
				UCSR0B |=  _BV(TXEN0) ;
				break;

		#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
			case USART1:
				/* Enable the Tx. */
				UCSR1B |= _BV(TXEN1);
				break;
		#endif

			default:
				return -EINVAL;
				break;
		}
	}

	return 0;
}

/*** BeginHeader xbee_ser_flowcontrol */
/*** EndHeader */
int xbee_ser_flowcontrol( xbee_serial_t *serial, bool_t enabled)
{
	XBEE_SER_CHECK( serial);

	if (enabled)
	{
		switch ( serial->usart )
		{
		case USART0:
			return 0;
			break;

#if defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega324PA__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
		case USART1:
			return 0;
			break;
#endif

		default:
			return -EINVAL;
			break;
		}
	}

	return 0;
}

/*** BeginHeader xbee_ser_set_rts */
/*** EndHeader */
int xbee_ser_set_rts( xbee_serial_t *serial, bool_t asserted)
{
	XBEE_SER_CHECK( serial);

	return 0;
}

/*** BeginHeader xbee_ser_get_cts */
int xbee_ser_get_cts( xbee_serial_t *serial)
{
	XBEE_SER_CHECK( serial);

	// the xCheckCTS returns the pin's value, negate it since CTS is active low.
	return 1;
}

//@}
