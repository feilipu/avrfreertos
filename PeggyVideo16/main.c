/*
* Copyright 2008 Jay Clegg.  All rights reserved.
*
*    This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
	Peggy2-i2c interface, Copyright 2008 by Jay Clegg.  All rights reserved.

	This code is designed for an unmodified Peggy 2.0 board sold by evilmadscience.com

	The code configures the Peggy as an TWI (I2C) slave.

	Companion code for an Arduino allows it to act as an TWI master, so that it can transmit
	frames to the peggy.

	Please see http://www.planetclegg.com/projects/Twi2Peggy.html for explanation of how all
	this is supposed to work.

	Credits goes to:
		Windell H Oskay, (http://www.evilmadscientist.com/)
			for creating the Peggy 2.0 kit, and getting 16 shades of gray working
		Geoff Harrison (http://www.solivant.com/peggy2/),
			for proving that interrupt driven display on the Peggy 2.0 was viable.
*/

////////////////////////////////////////////////////////////////////////////////////////////
// FPS must be high enough to not have obvious flicker, low enough that main loop has
// time to process one byte per pass.
// ~140 seems to be about the absolute max for me (with this code on avr-gcc 4.2, -Os),
// but compiler differences might make this maximum value larger or smaller.
// if the value is too high errors start to occur or it will stop receiving altogether
// conversely, any lower than 60 and flicker becomes apparent.
// note: further code optimization might allow this number to
// be a bit higher, but only up to a point...


////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////    main.c
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////


#include <stdlib.h>
#include <stdbool.h>

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

// freeRTOS Scheduler include files.
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// freeRTOS added i2c Interface include file.
// I2C_BUFFER_SIZE Set this to the largest message size that will be sent including address byte.
#include "i2cMultiMaster.h"
#include "spi.h"
#include "lib_crc.h"

// Peggy Video include file.
#include "PeggyVideo16.h"

/////////////// Global DATA structures /////////////////

// Stores pixels in two per byte packed format.
Pixel frameBuffer[DISP_COLUMN_LENGTH][DISP_BYTES_LENGTH];

/**************************************************************************************
 * Set the brightness of the pixel at x, y to 'brightness' (0=off).
 *************************************************************************************/

inline void setPixel(uint8_t x,uint8_t y, uint8_t brightness);

///////////////////// Main program loop ////////////////
int main(void) __attribute__((OS_main));

int main(void)
{

    xTaskCreate(
        TaskReadI2CVideo
        ,  (const portCHAR *)"ReadI2CVideo"
        ,  256				// Tested x free
        ,  NULL
        ,  2
        ,  NULL );


    xTaskCreate(
		TaskWriteLED
		,  (const portCHAR *)"WriteLED"
		,  208				// Tested x free
		,  NULL
		,  1
		,  NULL );


    vTaskStartScheduler();
}

/*-----------------------------------------------------------*/



static void TaskReadI2CVideo(void *pvParameters) // Read i2c Bus for video frames being sent.
{
    (void) pvParameters;;

    TickType_t xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

    // init I2C slave interface, need to do this once only.
    I2C_Slave_Initialise( (PEGGYVIDEO<<I2C_ADR_BITS) | (true<<I2C_GEN_BIT) );


	xVideoRowArray xVideoRow;     // Holds return values from the I2C bus transfer.

    while(1)
    {

        I2C_Slave_Start_Transceiver();

    	while( I2C_statusReg.RxDataInBuf == false )
    		vTaskDelayUntil( &xLastWakeTime, ( 5 / portTICK_PERIOD_MS ) ); // adjust later

		I2C_Slave_Get_Data_From_Transceiver( (uint8_t *)&xVideoRow, sizeof(xVideoRow) );

		// check that we're receiving good data.
		// and move the row data into the frame buffer

		if ( ! crc8( (uint8_t *) &xVideoRow, sizeof(xVideoRow)  ))
			for ( uint8_t i = 0; i < DISP_BYTES_LENGTH; i++)
				frameBuffer[xVideoRow.RowNumber][i].both = xVideoRow.DoublePixels[i];

    }
}

/*-----------------------------------------------------------*/


static void TaskWriteLED(void *pvParameters) // Write to LED
{
    (void) pvParameters;

    TickType_t xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

    uint8_t * currentRowPtr 	= (uint8_t *) frameBuffer;

    uint8_t currentRow			= 0;
    register uint8_t currentBrightness	= 0;


    // turn OFF serial RX/TX, necessary if using arduino bootloader
	UCSR0B = 0;

	// need to set output for SPI clock, MOSI, and SS.
	// Even though SS is not connected it must be set as output to remain in Master mode
	// set the speed to fsk/2
	spiBegin(Default);
	spiSetClockDivider(SPI_CLOCK_DIV2);

	// set the latch pin as output.
	DDRB |= _BV(DDB1);

	// set all PortD pins as output.
	DDRD = 0xff;

	// select no row.
	PORTD=0;


	for (uint8_t i=0; i < 4; i++)
	{
		SPDR = 0;
		while (!bit_is_set(SPSR, SPIF))
			if (!bit_is_set(SPCR, MSTR))
			{
				// The SPI module has left master mode, so return.
				// Otherwise, this will be an infinite loop.
				return;
			}
	}


	// set the display to white/grey/black on launch.

	for ( uint8_t i = 0; i < DISP_COLUMN_LENGTH; i++)
	{
		for ( uint8_t j = 0; j < DISP_ROW_LENGTH; j++)
			setPixel( j, i, MAX_BRIGHTNESS );
	}


    while(1)
    {
    	// there are 15 passes through this interrupt for each row per frame.
    	// ( 15 * 25) = 375 times per frame.
    	// during those 15 passes, a led can be on or off.
    	// if it is off the entire time, the perceived brightness is 0/15
    	// if it is on the entire time, the perceived brightness is 15/15
    	// giving a total of 16 average brightness levels from fully on to fully off.
    	// currentBrightness is a comparison variable, used to determine if a certain
    	// pixel is on or off during one of those 15 cycles.   currentBrightnessShifted
    	// is the same value left shifted 4 bits:  This is just an optimisation for
    	// comparing the high-order bytes.

    	if (++currentBrightness >= MAX_BRIGHTNESS)
    	{
    		currentBrightness = 0;
    		if (++currentRow > 24)
    		{
    	        currentRow = 0;
    			currentRowPtr = (uint8_t *) frameBuffer;
    		}
    		else
    		{
    			currentRowPtr += DISP_BYTES_LENGTH;
    		}
    	}


    	////////////////////  Parse a row of data and write out the bits via spi
    	register uint8_t currentBrightnessShifted = currentBrightness <<4;

    	uint8_t *ptr = currentRowPtr + 12;  // its more convenient to work from right to left
    	register uint8_t p;
    	register uint8_t bits = 0;

    	// optimisation: by using variables for these two masking constants, we can trick gcc into NOT
    	// promoting to 16-bit int (constants are 16 bit by default, causing the
    	// comparisons to get promoted to 16bit otherwise)].  This turns out to be a pretty
    	// substantial optimisation for this handler
    	register uint8_t himask = 0xf0;
    	register uint8_t lomask = 0x0f;

    	// Optimisation: interleave waiting for SPI with other code, so the CPU can do something useful
    	// when waiting for each SPI transmission to complete

    	p = *ptr--;
    	if ((p & lomask) > currentBrightness)  			bits|=1;

    	SPDR = bits;

    	bits=0;
    	p = *ptr--;
    	if ((p & lomask) > currentBrightness)  			bits|=64;
    	if ((p & himask) > currentBrightnessShifted)	bits|=128;
    	p = *ptr--;
    	if ((p & lomask) > currentBrightness)  			bits|=16;
    	if ((p & himask) > currentBrightnessShifted)	bits|=32;
    	p = *ptr--;
    	if ((p & lomask) > currentBrightness)  			bits|=4;
    	if ((p & himask) > currentBrightnessShifted)	bits|=8;
    	p = *ptr--;
    	if ((p & lomask) > currentBrightness)  			bits|=1;
    	if ((p & himask) > currentBrightnessShifted)	bits|=2;

		while (!bit_is_set(SPSR, SPIF)) // wait for prior bitshift to complete
			if (!bit_is_set(SPCR, MSTR))
			{
				// The SPI module has left master mode, so return.
				// Otherwise, this will be an infinite loop.
				return;
			}
    	SPDR = bits;


    	bits=0;
    	p = *ptr--;
    	if ((p & lomask) > currentBrightness)  			bits|=64;
    	if ((p & himask) > currentBrightnessShifted)	bits|=128;
    	p = *ptr--;
    	if ((p & lomask) > currentBrightness)  			bits|=16;
    	if ((p & himask) > currentBrightnessShifted)	bits|=32;
    	p = *ptr--;
    	if ((p & lomask) > currentBrightness)  			bits|=4;
    	if ((p & himask) > currentBrightnessShifted)	bits|=8;
    	p = *ptr--;
    	if ((p & lomask) > currentBrightness)  			bits|=1;
    	if ((p & himask) > currentBrightnessShifted)	bits|=2;

		while (!bit_is_set(SPSR, SPIF)) // wait for prior bitshift to complete
			if (!bit_is_set(SPCR, MSTR))
			{
				// The SPI module has left master mode, so return.
				// Otherwise, this will be an infinite loop.
				return;
			}
    	SPDR = bits;


    	bits=0;
    	p = *ptr--;
    	if ((p & lomask) > currentBrightness)  			bits|=64;
    	if ((p & himask) > currentBrightnessShifted)	bits|=128;
    	p = *ptr--;
    	if ((p & lomask) > currentBrightness)  			bits|=16;
    	if ((p & himask) > currentBrightnessShifted)	bits|=32;
    	p = *ptr--;
    	if ((p & lomask) > currentBrightness)  			bits|=4;
    	if ((p & himask) > currentBrightnessShifted)	bits|=8;
    	p = *ptr--;
    	if ((p & lomask) > currentBrightness)  			bits|=1;
    	if ((p & himask) > currentBrightnessShifted)	bits|=2;

		while (!bit_is_set(SPSR, SPIF)) // wait for prior bitshift to complete
			if (!bit_is_set(SPCR, MSTR))
			{
				// The SPI module has left master mode, so return.
				// Otherwise, this will be an infinite loop.
				return;
			}
    	SPDR = bits;

    	////////////////////  Now set the row and latch the bits

    	uint8_t portD; // hold the row select bits.

    	if (currentRow < 15)
    		portD = currentRow+1;
    	else
    		portD = (currentRow -14)<<4;


		while (!bit_is_set(SPSR, SPIF))  // wait for last bitshift to complete
			if (!bit_is_set(SPCR, MSTR))
			{
				// The SPI module has left master mode, so return.
				// Otherwise, this will be an infinite loop.
				return;
			}


    	PORTD = 0;				// set all rows to off
    	PORTB |= _BV(PB1); //  latch it, values now set

    	// >> Timing below is valid under freeRTOS @ 22.1184MHz <<<
    	// Rough observation from freeRTOS is just under 4ms per frame.
    	// This means we can sustain 250 Hz frame rate.

    	if( currentRow == 0 && currentBrightness == 0)
    		vTaskDelayUntil( &xLastWakeTime, ( 5 / portTICK_PERIOD_MS ) ); // setting 200Hz frame rate


    	PORTD = portD;     // set row
    	PORTB &= ~_BV(PB1); // reset latch for next time



    	// >> Timing below is not valid under freeRTOS & 22.1184MHz <<<

    	// notes to self, calculations from the oscilloscope:
    	// need about minimum of 6us total to clock out all 4 bytes
    	// roughly 1.5ms per byte, although some of that is
    	// idle time taken between bytes.  6=7us therefore is our
    	// absolute minimum time needed to refresh a row, not counting calculation time.
    	// Thats just if we do nothing else when writing out SPI and toggle to another row.
    	//Measured values from this routine
    	// @ 144 fps the latch is toggled every 19us with an actual 4byte clock out time of 12-13us
    	// @ 70 fps the latch is toggle every 39us, with a clock out time of 13-14us
    	// times do not count setup/teardown of stack frame

    	// one byte @ 115k takes 86us (max) 78us (min) , measured time
    	// one byte @ 230k takes 43us (max) 39us (min) , measured time
    	// so 230k serial might barely be possible, but not with a 16mhz crystal (error rate to high)
    	// 250k might just barely be possible

	}
}


/**************************************************************************************
 * Set the brightness of the pixel at x, y to 'brightness' (0=off).
 *************************************************************************************/
inline void setPixel(uint8_t x,uint8_t y, uint8_t brightness)
{
	if (x & 0x01)
		frameBuffer[y][x/2].odd = brightness;
	else
		frameBuffer[y][x/2].even = brightness;
}


/*-----------------------------------------------------------*/


void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    portCHAR *pcTaskName )
{
	DDRB  |= _BV(DDB5);
	PORTB |= _BV(PORTB5);       // main (red PB5) LED on. Arduino LED on and die.
	while(1);
}

/*-----------------------------------------------------------*/
