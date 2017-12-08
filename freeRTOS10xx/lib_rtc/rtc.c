/*
 * rtc.c
 *
 *  Created on: 16/02/2012
 *  Revised on:  8/ 7/2014 - modified to pass variables consistent with time.h time functions.
 *  Author: Phillip Stevens
 */

#include <stdint.h>
#include <stdlib.h>		/* ANSI memory controls */

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#ifdef portRTC_DEFINED		// RTC implemented, therefore define.

/* i2c Interface include file. */
#include "i2cMultiMaster.h"

#include "rtc.h"			/* RTC configurations and declarations */

/* structure to receive the DS1307 RTC parameters */
typedef struct
{
	uint8_t	   I2CAddress;	 // Address and read/write bit for I2C transaction. First Byte.
	uint8_t    Second;       //
	uint8_t    Minute;       //
	uint8_t    Hour;         // 1-12, 0-23 (depending on am pm/24 bit 6)
	uint8_t    Day;          // Sun=1, Mon=2, Tue=3, Wed=4, Thur=5, Fri=6, Sat=7
	uint8_t    Date;         // 1 through 28, 30, or 31
	uint8_t    Month;        // Jan=1,... Dec=12
	uint8_t    Year;         // '00 through '99
} xDS1307Array, * pDS1307Array;

/* structure to hold the DS1307 RTC parameters for transmission*/
// used ONLY for SETTING the time, where the Command byte is required.
typedef struct
{
	uint8_t	   I2CAddress;	  // Address and read/write bit for I2C transaction. First Byte.
	uint8_t    Command;       // Command or Address on the I2C bus
	uint8_t    Second;        //
	uint8_t    Minute;        //
	uint8_t    Hour;          // 1-12, 0-23 (depending on am pm/24 bit 6)
	uint8_t    Day;           // Sun=1, Mon=2, Tue=3, Wed=4, Thur=5, Fri=6, Sat=7
	uint8_t    Date;          // 1 through 28, 30, or 31
	uint8_t    Month;         // Jan=1,... Dec=12
	uint8_t    Year;          // '00 through '99
	uint8_t    Control;       //
} xDS1307ArraySto, * pDS1307ArraySto;

/*----------------   Private Functions   -------------------------*/

static uint8_t decToBcd(uint8_t);  // Convert normal decimal numbers to binary coded decimal
static uint8_t bcdToDec(uint8_t);  // Convert binary coded decimal to normal decimal numbers

/*----------------------------------------------------------------*/

uint8_t getDateTimeDS1307( tm * timeDate)
{
    uint8_t I2C_command_buf[ 2 ];
    xDS1307Array xTimeDate;

	if( xI2CSemaphore != NULL ) // use the xI2CSemaphore if you're sharing the I2C bus.
								// Better to do it anyway.
	{
		// See if we can obtain the semaphore.  If the semaphore is not available
		// wait 10 ticks to see if it becomes free.
		if( xSemaphoreTake( xI2CSemaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
			// We were able to obtain the semaphore and can now access the
			// shared resource.

			/*  Reading from the Slave
			1. Send a start sequence
			2. Send 0xD0 ( I2C address of the DS1307 with the R/W bit low (even address)
			3. Send 0x00 (Internal address of the bearing register)

			4. Send a start sequence again (repeated start)
			5. Send 0xD1 ( I2C address of the DS1307 with the R/W bit high (odd address)
			6. Read data byte from DS1307
			7. Repeat, reading the next data byte from DS1307
			8. Send the stop sequence.
			*/

			I2C_command_buf[0] = DS1307 + I2C_WRITE; 		// set device address and write mode
			I2C_command_buf[1] = 0x00;                      // write address = 0 (Seconds)

			if (I2C_Check_Free_After_Stop() == pdTRUE )
				I2C_Master_Start_Transceiver_With_Data( (uint8_t *)&I2C_command_buf, 2 );

			xTimeDate.I2CAddress = DS1307 + I2C_READ;		// set device address and read mode

			if (I2C_Check_Free_After_Stop() == pdTRUE )
				// all that matters here is to have the right size of the data we're looking to receive.
				I2C_Master_Start_Transceiver_With_Data( (uint8_t *)&xTimeDate, sizeof(xDS1307Array) );

			if( I2C_Master_Get_Data_From_Transceiver( (uint8_t *)&xTimeDate, sizeof(xDS1307Array) ) )
			{
				// We have finished accessing the shared resource. Release the i2c semaphore.
				xSemaphoreGive( xI2CSemaphore );

				timeDate->tm_sec  =  bcdToDec( xTimeDate.Second & 0x7f );		// convert one byte 0-59
				timeDate->tm_min  =  bcdToDec( xTimeDate.Minute & 0x7f );		// convert one byte 0-59
				timeDate->tm_hour =  bcdToDec( xTimeDate.Hour   & 0x3f );		// convert one byte 1-23
				timeDate->tm_wday =  bcdToDec( xTimeDate.Day    & 0x07 ) -1;	// convert one byte to Sun=0, Mon=1, Tue=2, Wed=3, Thur=4, Fri=5, Sat=6
				timeDate->tm_mday =  bcdToDec( xTimeDate.Date   & 0x3f );		// convert one byte to 1 to 28, 30, or 31
				timeDate->tm_mon  =  bcdToDec( xTimeDate.Month  & 0x1f ) -1;	// convert one byte to Jan=0,... Dec=11
				timeDate->tm_year =  (uint16_t)bcdToDec( xTimeDate.Year );		// '00 - '99 year
			}
			else{
				// We have finished accessing the shared resource. Release the i2c semaphore.
				xSemaphoreGive( xI2CSemaphore );

				return  pdFALSE;           // return 0 to signify failure.
			}
		}
	}
	return pdTRUE;
}

// set the date & time to the values given here.

uint8_t setDateTimeDS1307(tm * timeDateSet)
{
	// Holds values for the RTC DS1307
	xDS1307ArraySto xSettings;

	if( xI2CSemaphore != NULL ) // use the xI2CSemaphore if you're sharing the I2C bus.
	{
		// See if we can obtain the semaphore.  If the semaphore is not available
		// wait 10 ticks to see if it becomes free.
		if( xSemaphoreTake( xI2CSemaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
			xSettings.I2CAddress = DS1307 + I2C_WRITE;					// set device address and write mode
			xSettings.Command    = 0x00;								// Write to the first address 0x00 (Seconds)
			xSettings.Second     = decToBcd (timeDateSet->tm_sec);		// 0-59
			xSettings.Minute     = decToBcd (timeDateSet->tm_min);		// 0-59
			xSettings.Hour       = decToBcd (timeDateSet->tm_hour);     // 1-23
			xSettings.Day        = decToBcd (timeDateSet->tm_wday +1);	// convert to Sun=1, Mon=2, Tue=3, Wed=4, Thur=5, Fri=6, Sat=7
			xSettings.Date       = decToBcd (timeDateSet->tm_mday);     // convert one byte to 1 to 28, 30, or 31
			xSettings.Month      = decToBcd (timeDateSet->tm_mon +1);	// convert to Jan=1,... Dec=12
			xSettings.Year       = decToBcd (timeDateSet->tm_year);		// convert '00 - '99 year
			xSettings.Control    = SQWENABLE;							// enable the 1Hz square wave

			if (I2C_Check_Free_After_Stop() == pdTRUE )
			{
				I2C_Master_Start_Transceiver_With_Data( (uint8_t *)&xSettings, sizeof(xDS1307ArraySto));

				// We have finished accessing the shared resource. Release the i2c semaphore.
				xSemaphoreGive( xI2CSemaphore );

			}else{
				xSemaphoreGive( xI2CSemaphore );
				return pdFALSE;
			}
		}
	}
	return pdTRUE;
}

/*----------------------------------------------------------------*/

// Convert normal decimal number byte to binary coded decimal byte
uint8_t decToBcd(uint8_t val)
{
    return ( ((uint16_t)val*16/10) + (val%10) );
}

// Convert binary coded decimal byte to normal decimal number byte
uint8_t bcdToDec(uint8_t val)
{
    return ( ((uint16_t)val*10/16) + (val%16) );
}

/*----------------------------------------------------------------*/

#endif
