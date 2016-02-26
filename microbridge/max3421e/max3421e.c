/*
Copyright 2011 Niels Brouwers

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.#include <string.h>
*/

/*
 *
 * Library for the max3421e USB host controller shield produced by circuitsathome and Sparkfun.
 * This is a low-level interface that provides access to the internal registers and polls the
 * controller for state changes.
 *
 * This library is based on work done by Oleg Masurov, but has been ported to C and heavily
 * restructured. Control over the GPIO pins has been stripped.
 *
 * Note that the current incarnation of this library only supports the Arduino Mega with a
 * hardware mod to rewire the MISO, MOSI, and CLK SPI pins.
 *
 * http://www.circuitsathome.com/
 */

// Include freeRTOS Headers & AVR files

#include <stdbool.h>

#include <util/delay.h>

#include "FreeRTOS.h"
#include "task.h"

#include "serial.h"

#include "max3421e_constants.h"
#include "max3421e.h"
#include "max3421e_usb.h"


static vbusState_t vbusState;

/*
 * Initialises the max3421e host shield. Initialises the SPI bus and sets the required pin directions.
 * Must be called before powerOn.
 */
void max3421e_init(void)
{
	spiBegin(Default);
	spiSetClockDivider(SPI_CLOCK_DIV2);
	spiSetDataMode(SPI_MODE0);

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

	// Set MAX_INT and MAX_GPX pins to input mode.
	DDRE &= ~_BV(DDE5);
	DDRJ &= ~_BV(DDJ3);

	// Set MAX RESET pin to output
	DDRJ |= _BV(DDJ2);

	// Hold the MAX RESET high
	PORTJ |= _BV(PJ2);

	// MAX SPI _SS pin is the default _SS

#else

#warning no INT, GPX, RESET defined.

#endif


}

/*
 * Resets the max3412e. Sets the chip reset bit, SPI configuration is not affected.
 * @return true if success.
 */
uint8_t max3421e_reset(void)
{
	uint8_t tmp = 0;

	max3421e_write(MAX_REG_USBCTL, bmCHIPRES);	// Chip reset. This stops the oscillator

	// 200nS is enough pulse width...

	max3421e_write(MAX_REG_USBCTL, 0x00);		// Remove the reset

	while (!(max3421e_read(MAX_REG_USBIRQ) & bmOSCOKIRQ))	// Wait until the PLL is stable
		{
		_delay_ms(1); // needs 1mS in this loop, or the PLL doesn't settle.
		if( ++tmp == 0 ) return( false ); //timeout after 256 attempts
		}

	return (true);	// Success.
}

/*
 * Initialises the max3421e after power-on.
 */
void max3421e_powerOn(void)
{
	// Configure full-duplex SPI, interrupt pulse.
	max3421e_write(MAX_REG_PINCTL, bmFDUPSPI | bmINTLEVEL | bmGPXB ); //Full-duplex SPI, level interrupt, GPX

	// Stop and restart the oscillator.
	if (max3421e_reset() == false)
		xSerialPrint_P(PSTR("\nError: OSCOKIRQ failed to assert!\r\n"));

	// Configure host operation.
	max3421e_write(MAX_REG_MODE, bmDPPULLDN | bmDMPULLDN | bmSEPIRQ | bmHOST ); // set pull-downs, Host, separate GPIN IRQ on GPX
	max3421e_write(MAX_REG_HIEN, bmFRAMEIE | bmCONDETIE );// //connection detection

	// Check if device is connected.
	max3421e_write(MAX_REG_HCTL, bmSAMPLEBUS ); // sample USB bus
	while (!(max3421e_read(MAX_REG_HCTL) & bmSAMPLEBUS)); //wait for sample operation to finish

	max3421e_busprobe(); //check if anything is connected
	max3421e_write(MAX_REG_HIRQ, bmCONDETIRQ ); //clear connection detect interrupt

	max3421e_write(MAX_REG_CPUCTL, bmIE);	// Enable interrupt pin.
}

/*
 * Writes a single register.
 *
 * @param reg register address.
 * @param value value to write.
 */
void max3421e_write(max_registers_t reg, uint8_t value)
{
	// If the SPI module has not been enabled yet, then return with nothing.
	if( !(SPCR & _BV(SPE)) ) return;

	// The SPI module is enabled, but it is in slave mode, so we can not
	// transmit the byte. This can happen if SSbar is an input and it went low.
	// We will try to recover by setting the MSTR bit.
	if( !(SPCR & _BV(MSTR)) )
		{
			SPCR |= _BV(MSTR);
			if( !(SPCR & _BV(MSTR)) ) return;
		}

	// Pull slave select low to indicate start of transfer.
	MAX_SS(0);

	// Transfer command byte, 0x02 indicates write.
	SPDR = ((uint8_t)reg | 0x02);

	while( !(SPSR & _BV(SPIF)) )
	{
		if( !(SPCR & _BV(MSTR)) ) return;
			// The SPI module has left master mode, so return.
			// Otherwise, this will be an infinite loop.
	}

	// Transfer value byte.
	SPDR = value;

	while( !(SPSR & _BV(SPIF)) )
	{
		if( !(SPCR & _BV(MSTR)) ) return;
			// The SPI module has left master mode, so return.
			// Otherwise, this will be an infinite loop.
	}

	// Pull slave select high to indicate end of transfer.
	MAX_SS(1);

	return;
}

/*
 * Writes multiple bytes to a register.
 * @param reg register address.
 * @param count number of bytes to write.
 * @param values input values.
 * @return a pointer to values, incremented by the number of bytes written (values + length).
 */
uint8_t * max3421e_writeMultiple(max_registers_t reg, uint8_t count, uint8_t * values)
{
	// If the SPI module has not been enabled yet, then return with nothing.
	if( !(SPCR & _BV(SPE)) ) return 0;

	// The SPI module is enabled, but it is in slave mode, so we can not
	// transmit the byte. This can happen if SSbar is an input and it went low.
	// We will try to recover by setting the MSTR bit.
	if( !(SPCR & _BV(MSTR)) )
		{
			SPCR |= _BV(MSTR);
			if( !(SPCR & _BV(MSTR)) ) return 0;
		}

	// Pull slave select low to indicate start of transfer.
	MAX_SS(0);

	// Transfer command byte, 0x02 indicates write.
	SPDR = ((uint8_t)reg | 0x02);

	// Transfer values.
	while (count--)
	{
		// Send next value byte.
		while( !(SPSR & _BV(SPIF)) )
		{
			if( !(SPCR & _BV(MSTR)) ) return 0;
				// The SPI module has left master mode, so return.
				// Otherwise, this will be an infinite loop.
		}
		SPDR = (*values);
		values++;
	}

	while( !(SPSR & _BV(SPIF)) )
	{
		if( !(SPCR & _BV(MSTR)) ) return 0;
			// The SPI module has left master mode, so return.
			// Otherwise, this will be an infinite loop.
	}

	// Pull slave select high to indicate end of transfer.
	MAX_SS(1);

	return (values);
}

/*
 * Reads a single register.
 *
 * @param reg register address.
 * @return result value.
 */
uint8_t max3421e_read(max_registers_t reg)
{
	// If the SPI module has not been enabled yet, then return with nothing.
	if( !(SPCR & _BV(SPE)) ) return 0;

	// The SPI module is enabled, but it is in slave mode, so we can not
	// transmit the byte. This can happen if SSbar is an input and it went low.
	// We will try to recover by setting the MSTR bit.
	if( !(SPCR & _BV(MSTR)) )
		{
			SPCR |= _BV(MSTR);
			if( !(SPCR & _BV(MSTR)) ) return 0;
		}

	// Pull slave-select low to initiate transfer.
	MAX_SS(0);

	// Send a command byte containing the register number.
	SPDR = (uint8_t)reg;

	while( !(SPSR & _BV(SPIF)) )
	{
		if( !(SPCR & _BV(MSTR)) ) return 0;
			// The SPI module has left master mode, so return.
			// Otherwise, this will be an infinite loop.
	}

	// Send an empty byte while reading.
	SPDR = 0xFF;

	while( !(SPSR & _BV(SPIF)) )
	{
		if( !(SPCR & _BV(MSTR)) ) return 0;
			// The SPI module has left master mode, so return.
			// Otherwise, this will be an infinite loop.
	}

	// Pull slave-select high to signal transfer complete.
	MAX_SS(1);

	// Return result byte.
	return (SPDR);
}

/*
 * Reads multiple bytes from a register.
 *
 * @param reg register to read from.
 * @param count number of bytes to read.
 * @param values target buffer.
 * @return pointer to the input buffer + count.
 */
uint8_t * max3421e_readMultiple(max_registers_t reg, uint8_t count, uint8_t * values)
{
	// If the SPI module has not been enabled yet, then return with nothing.
	if( !(SPCR & _BV(SPE)) ) return 0;

	// The SPI module is enabled, but it is in slave mode, so we can not
	// transmit the byte. This can happen if SSbar is an input and it went low.
	// We will try to recover by setting the MSTR bit.
	if( !(SPCR & _BV(MSTR)) )
		{
			SPCR |= _BV(MSTR);
			if( !(SPCR & _BV(MSTR)) ) return 0;
		}


	// Pull slave-select high to initiate transfer.
	MAX_SS(0);

	// Send a command byte containing the register number.
	SPDR = (uint8_t)reg;

	while( !(SPSR & _BV(SPIF)) )
	{
		if( !(SPCR & _BV(MSTR)) ) return 0;
			// The SPI module has left master mode, so return.
			// Otherwise, this will be an infinite loop.
	}

	// Read [count] bytes.
	while (count--)
	{
		// Send empty byte while reading.
		SPDR = 0xFF;

		while( !(SPSR & _BV(SPIF)) )
		{
			if( !(SPCR & _BV(MSTR)) ) return 0;
				// The SPI module has left master mode, so return.
				// Otherwise, this will be an infinite loop.
		}

		*values++ = SPDR;
	}

	// Pull slave-select low to signal transfer complete.
	MAX_SS(1);

	// Return the byte array + count.
	return (values);
}

/*
 * @return the status of Vbus.
 */
vbusState_t max3421e_getVbusState(void)
{
	return vbusState;
}

/*
 * Probes the bus to determine device presence and speed, and switches host to this speed.
 */
void max3421e_busprobe(void)
{
	uint8_t bus_sample;
	bus_sample = max3421e_read(MAX_REG_HRSL); //Get J,K status

//	xSerialPrintf_P(PSTR("\r\nmax3421 bus_probe @ 0x%02x"), bus_sample); // FIXME remove this debugging
	// if the bus probe is not 0x80 or 0x40 then there is a Host error in the 4 LSB's in the HRSL register.

	bus_sample &= (bmJSTATUS | bmKSTATUS); //zero the rest of the uint8_t

	switch (bus_sample)
	{
	case (bmSE0): //disconnected state
		max3421e_write(MAX_REG_MODE, bmDPPULLDN|bmDMPULLDN|bmSEPIRQ|bmHOST);
		vbusState = SE0;
		break;

	//start full-speed or low-speed host
	case (bmJSTATUS):
		if ((max3421e_read(MAX_REG_MODE) & bmLOWSPEED) == 0)
		{
			max3421e_write(MAX_REG_MODE, MODE_FS_HOST ); //start full-speed host
			vbusState = FSHOST;
		} else
		{
			max3421e_write(MAX_REG_MODE, MODE_LS_HOST); //start low-speed host
			vbusState = LSHOST;
		}
		break;
	case (bmKSTATUS):
		if ((max3421e_read(MAX_REG_MODE) & bmLOWSPEED) == 0)
		{
			max3421e_write(MAX_REG_MODE, MODE_LS_HOST ); //start low-speed host
			vbusState = LSHOST;
		} else
		{
			max3421e_write(MAX_REG_MODE, MODE_FS_HOST ); //start full-speed host
			vbusState = FSHOST;
		}
		break;

	case (bmSE1): //illegal state
		vbusState = SE1;
		break;
	}
}

/*
 * MAX3421 state change task and interrupt handler.
 * @return error code or 0 if successful.
 */
uint8_t max3421e_poll(void)
{
	uint8_t rcode = 0;

	// Check INT interrupt, interrupt is low signal.
	if (MAX_INT() == 0)
		rcode = max3421e_interruptHandler();

	// Check GPX interrupt, interrupt is low signal.
	if (MAX_GPX() == 0)
		max3421e_gpxInterruptHandler();

	return (rcode);
}

/*
 * Interrupt handler.
 */
uint8_t max3421e_interruptHandler(void)
{
	uint8_t interruptStatus;
	uint8_t HIRQ_sendback = 0x00;

	// Determine interrupt source.
	interruptStatus = max3421e_read(MAX_REG_HIRQ);

/*	if (interruptStatus & bmFRAMEIRQ)	//->1ms SOF interrupt handler - have removed this from powerOn
	{
		HIRQ_sendback |= bmFRAMEIRQ;
	}
*/

	if (interruptStatus & bmCONDETIRQ)
	{
		max3421e_busprobe();

		HIRQ_sendback |= bmCONDETIRQ;
	}

	// End HIRQ interrupts handling, clear serviced IRQs
	max3421e_write(MAX_REG_HIRQ, HIRQ_sendback);

	return (HIRQ_sendback);
}

/*
 * GPX interrupt handler
 */
uint8_t max3421e_gpxInterruptHandler(void)
{
	//read GPIN IRQ register
	uint8_t interruptStatus = max3421e_read(MAX_REG_GPINIRQ);
/*
	if( GPINIRQ & bmGPINIRQ7 ) {            //vbus overload
		vbusPwr( OFF );                     //attempt powercycle
		delay( 1000 );
		vbusPwr( ON );
		regWr( rGPINIRQ, bmGPINIRQ7 );
	}
*/
	return (interruptStatus);
}

