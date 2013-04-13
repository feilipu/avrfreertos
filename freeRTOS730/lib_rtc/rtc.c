/*
 * rtc.c
 *
 *  Created on: 16/02/2012
 *      Author: Phillip Stevens
 */



#include <stdint.h>
#include <stdlib.h>		/* ANSI memory controls */

/* Scheduler include files. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#ifdef portRTC_DEFINED		// RTC implemented, therefore define.

/* i2c Interface include file. */
#include <i2cMultiMaster.h>

#include <rtc.h>			/* RTC configurations and declarations */

/*----------------   Private Functions   -------------------------*/

static uint8_t decToBcd(uint8_t);  // Convert normal decimal numbers to binary coded decimal
static uint8_t bcdToDec(uint8_t);  // Convert binary coded decimal to normal decimal numbers

/*----------------------------------------------------------------*/

uint8_t getDateTimeDS1307( pRTCArray xTimeDate)
{
    uint8_t I2C_command_buf[ 2 ];

	if( xI2CSemaphore != NULL ) // use the xI2CSemaphore if you're sharing the I2C bus.
		// Better to do it anyway.
	{
		// See if we can obtain the semaphore.  If the semaphore is not available
		// wait 10 ticks to see if it becomes free.
		if( xSemaphoreTake( xI2CSemaphore, ( portTickType ) 10 ) == pdTRUE )
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

			xTimeDate->I2CAddress = DS1307 + I2C_READ;		// set device address and read mode

			if (I2C_Check_Free_After_Stop() == pdTRUE )
				// all that matters here is to have the right size of the data we're looking to receive.
				I2C_Master_Start_Transceiver_With_Data( (uint8_t *)xTimeDate, sizeof(xRTCArray) );

			if( I2C_Master_Get_Data_From_Transceiver( (uint8_t *)xTimeDate, sizeof(xRTCArray) ) )
			{
				xTimeDate->Second =  bcdToDec( xTimeDate->Second & 0x7f );  // convert one byte
				xTimeDate->Minute =  bcdToDec( xTimeDate->Minute & 0x7f );  // convert one byte
				xTimeDate->Hour   =  bcdToDec( xTimeDate->Hour   & 0x3f );  // convert one byte
				xTimeDate->Day    =  bcdToDec( xTimeDate->Day    & 0x07 );  // convert one byte
				xTimeDate->Date   =  bcdToDec( xTimeDate->Date   & 0x3f );  // convert one byte
				xTimeDate->Month  =  bcdToDec( xTimeDate->Month  & 0x1f );  // convert one byte
				xTimeDate->Year   =  bcdToDec( xTimeDate->Year );           // convert one byte

				// We have finished accessing the shared resource. Release the i2c semaphore.
				xSemaphoreGive( xI2CSemaphore );
			}
			else{
				xSemaphoreGive( xI2CSemaphore );

				return  pdFALSE;           // return 0 to signify failure.
			}
		}

	}

	return pdTRUE;
}


// set the date & time to the values given here.

uint8_t setDateTimeDS1307(pRTCArraySto xSettings)
{
	// Holds values for the RTC DS1307

	if( xI2CSemaphore != NULL ) // use the xI2CSemaphore if you're sharing the I2C bus.
	{
		// See if we can obtain the semaphore.  If the semaphore is not available
		// wait 10 ticks to see if it becomes free.
		if( xSemaphoreTake( xI2CSemaphore, ( portTickType ) 10 ) == pdTRUE )
		{
			xSettings->I2CAddress = DS1307 + I2C_WRITE;				 // set device address and write mode
			xSettings->Command	  = 0x00;							 // Write to the first address 0x00 (Seconds)
			xSettings->Second	  = decToBcd (xSettings->Second);	 // 0-59
			xSettings->Minute	  = decToBcd (xSettings->Minute);    // 0-59
			xSettings->Hour		  = decToBcd (xSettings->Hour);      // 1-23
			xSettings->Day		  = decToBcd (xSettings->Day);       // Sun=1, Mon=2, Tue=3, Wed=4, Thur=5, Fri=6, Sat=7
			xSettings->Date		  = decToBcd (xSettings->Date);      // 1-28/29/30/31
			xSettings->Month	  = decToBcd (xSettings->Month);     // Jan=1,... Dec=12
			xSettings->Year		  = decToBcd (xSettings->Year);      // '00 - '99
			xSettings->Control    = SQWENABLE;			    		 // enable the 1Hz square wave

			if (I2C_Check_Free_After_Stop() == pdTRUE )
			{
				I2C_Master_Start_Transceiver_With_Data( (uint8_t *)xSettings, sizeof(xRTCArraySto));

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
    return ( (val/10*16) + (val%10) );
}

// Convert binary coded decimal byte to normal decimal number byte
uint8_t bcdToDec(uint8_t val)
{
    return ( (val/16*10) + (val%16) );
}

/*----------------------------------------------------------------*/

#endif
