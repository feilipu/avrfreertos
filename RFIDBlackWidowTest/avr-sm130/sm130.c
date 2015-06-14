/**
 * 	@file	SM130.c
 * 	@brief	SM130 library for avr freeRTOS

 *	@see	http://www.arduino.cc
 *	@see	http://www.sonmicro.com/1356/sm130.php
 *	@see	http://rfid.marcboon.com
 */

#include <string.h>
#include <stdlib.h>

#include <avr/interrupt.h>
#include <avr/pgmspace.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include <i2cMultiMaster.h>

/* serial interface include file. */
#include <serial.h>

#include "sm130.h"

/*------------------- local functions -------------------*/

// Transmit command packet over I2C
static uint8_t sm130_transmitData(pCMDArray xTransmitData);

// Receive response packet over I2C
static uint8_t sm130_receiveData(pCMDArray xReceiveData);

// manipulate some key storage
static uint64_t sixByteArrayToLong(uint8_t *hashKey);
static void longToSixByteArray(uint8_t *hashKey, uint64_t number);


static uint8_t toHex(uint8_t b);


// INT1 interrupt vector - DREADY has to be used as input.
/*------------------- Interrupt Vector -------------------*/

ISR(INT1_vect)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR( xSM130IntrSemaphore, &xHigherPriorityTaskWoken );
}


/********************** Public functions ******************/


/**	Reset the SM130 module
 *
 * 	This function should be called in setup. It initialises the IO pins and
 *	issues a hardware or software reset, depending on the definition of pinRESET.
 *	After reset, a HALT_TAG command is issued to terminate the automatic SEEK mode.
 */
void sm130_reset(uint8_t hardRESET)
{
	// Set all the control lines.
	DREADY_input();				// initialise IO_D3 // 3
	RESET_off();	   			// initialise IO_D4 // 4
	LEDSearch_input(); 			// initialise IO_D5 // 5
	LEDFound_input();			// initialise IO_D6 // 6

 	// Initiate sm130
	if (hardRESET != 0x00) // hardware reset
	{
		RESET_on();
		LEDFound_on();
	 	vTaskDelay( ( 10 / portTICK_PERIOD_MS ) );
	 	LEDFound_off();
	 	LEDFound_input();
	 	RESET_off();
	}
	else // Initiate software reset
	{
		sm130_softReset();
	}
	// Allow enough time for reset
	vTaskDelay( ( 250 / portTICK_PERIOD_MS ) );

	/* Create the semaphore used by the DREADY to indicate that a Tag has been read for processing. */
    vSemaphoreCreateBinary( xSM130IntrSemaphore );

	sm130_setAntennaPower(pdTRUE);

	sm130_haltTag();
}

/*
 *	Sends a SEEK_TAG command
 */
uint8_t sm130_softReset(void)
{
	xCMDArray xData;

	xData.CMD_Code = CMD_RESET;
	xData.CMD_Length = 0x01;

	if( sm130_transmitData(&xData) == pdTRUE )
	{
		return pdTRUE;
	}
	return pdFALSE;
}


/*
 * Get the firmware version string.
 */
uint8_t sm130_getFirmwareVersion(uint8_t *version)
{
	xCMDArray xData;

	xData.CMD_Code = CMD_FIRMWARE_REV;
	xData.CMD_Length = 1;

	if( sm130_transmitData(&xData) )
	{
		 // wait 5ms for the command to be executed, and a response prepared.
		 // we have 50ms to get the response, before it is vaporised.

		vTaskDelay( 5 / portTICK_PERIOD_MS );

		// expect that the firmware version has length of 4
		xData.CMD_Length = 4;
		if( sm130_receiveData(&xData) > 0 )
		{
			if( xData.CMD_Length > 1 && xData.CMD_Code == CMD_FIRMWARE_REV)
			{
				memcpy((char *)version, (char *)&xData.CMD_Data, xData.CMD_Length -1);
				return pdTRUE;
			}
		}
	}
	return pdFALSE;

}

/*
 *	Sends a SEEK_TAG command
 */
uint8_t sm130_seekTag(uint8_t *tag)
{
	xCMDArray xData;

    // enable the INT1 interrupt that is connected to DREADY (active high)
	SM130_DREADY_ISR_ENABLE();

	xData.CMD_Code = CMD_SEEK_TAG;
	xData.CMD_Length = 0x01;

	if( sm130_transmitData(&xData) )
	{
		 // wait 5ms for the command to be executed, and a response prepared.
		 // we have 50ms to get the response, before it is vaporised.

		vTaskDelay( 5 / portTICK_PERIOD_MS );

		// expect that the seek Tag response has length of 2
		xData.CMD_Length = 0x02;
		if( sm130_receiveData(&xData) > 0 )
		{
			if( xData.CMD_Length >= 1 && xData.CMD_Code == CMD_SEEK_TAG && xData.CMD_Data[0] == RESP_CMD_EXCECUTING )
			{
				// wait a minute (60,000 ms), or until we find a tag (signalled by interrupt on DREADY, giving semaphore).
				xSemaphoreTake( xSM130IntrSemaphore, ( TickType_t )(60000 / portTICK_PERIOD_MS) );

				// expect that the seek Tag response has length of up to 6 bytes (Mifare Classic 1k and 4k)
				xData.CMD_Length = 0x06;
				if( sm130_receiveData(&xData) > 0 )
				{
					if( xData.CMD_Length >= 1 && xData.CMD_Code == CMD_SEEK_TAG )
					{
						memcpy((char *)tag, (char *)&xData.CMD_Data, xData.CMD_Length -1 );
						return RESP_SUCCESS;
					}
				}
			}
		}
	}
	return pdFALSE;
}

/*
 *	Sends a SEEK_TAG command
 */
uint8_t sm130_selectTag(uint8_t *tag)
{
	xCMDArray xData;

	xData.CMD_Code = CMD_SELECT_TAG;
	xData.CMD_Length = 0x01;

	if( sm130_transmitData(&xData) )
	{
		 // wait 5ms for the command to be executed, and a response prepared.
		 // we have 50ms to get the response, before it is vaporised.

		vTaskDelay( 5 / portTICK_PERIOD_MS );

		// expect that the select Tag response has length of 6 bytes (Mifare Classic 1k and 4k)
		xData.CMD_Length = 0x06;
		if( sm130_receiveData(&xData) > 0 )
		{
			if( xData.CMD_Length >= 1 && xData.CMD_Code == CMD_SELECT_TAG )
			{
				memcpy((char *)tag, (char *)&xData.CMD_Data, xData.CMD_Length -1 );
				return xData.CMD_Data[0];
			}
		}
	}
	return pdFALSE;
}

/** Authenticate with specified key A or key B.
 *
 *	@param block Block number
 *	@param keyType Which key to use: 0xAA for key A or 0xBB for key B
 *	@param key Key value (6 bytes)
 */
uint8_t sm130_authenticate(uint8_t block, uint8_t keyType, uint8_t *key)
{
	xCMDArray xData;

	xData.CMD_Code = CMD_AUTHENTICATE;
	xData.CMD_Length = 0x09;
	xData.CMD_Data[0] = block;
	xData.CMD_Data[1] = keyType;
	memcpy((char*)&xData.CMD_Data[2], key, 6);

	if( sm130_transmitData(&xData) )
	{
		 // wait 5ms for the command to be executed, and a response prepared.
		 // we have 50ms to get the response, before it is vaporised.

		vTaskDelay( 5 / portTICK_PERIOD_MS );

		// expect that the authentication result has length of 2
		xData.CMD_Length = 0x02;
		if( sm130_receiveData(&xData) == xData.CMD_Length )
		{
			if( xData.CMD_Length >= 1 && xData.CMD_Code == CMD_AUTHENTICATE )
			{
				return xData.CMD_Data[0];
			}
		}
	}
	return pdFALSE;
}

/** Authenticate with Transport Key.
 *
 *	@param block Block number
 *
 */
uint8_t sm130_authenticateTransport(uint8_t block)
{
	xCMDArray xData;

	xData.CMD_Code = CMD_AUTHENTICATE;
	xData.CMD_Length = 0x03;
	xData.CMD_Data[0] = block;
	xData.CMD_Data[1] = KEY_TRANSPORT;

	if( sm130_transmitData(&xData) )
	{
		 // wait 5ms for the command to be executed, and a response prepared.
		 // we have 50ms to get the response, before it is vaporised.

		vTaskDelay( 5 / portTICK_PERIOD_MS );

		// expect that the authentication result has length of 2
		xData.CMD_Length = 0x02;
		if( sm130_receiveData(&xData) == xData.CMD_Length )
		{
			if( xData.CMD_Length >= 1 && xData.CMD_Code == CMD_AUTHENTICATE)
			{
				return xData.CMD_Data[0];
			}
		}
	}
	return pdFALSE;
}



/**	Read 16-uint8_t block.
 *
 *	@param block Block number
 *
 */
uint8_t sm130_readBlock(uint8_t block, uint8_t *dataBlock)
{
	xCMDArray xData;

	xData.CMD_Code = CMD_READ16;
	xData.CMD_Length = 0x02;
	xData.CMD_Data[0] = block;

	if( sm130_transmitData(&xData) )
	{
		 // wait 5ms for the command to be executed, and a response prepared.
		 // we have 50ms to get the response, before it is vaporized.

		vTaskDelay( 5 / portTICK_PERIOD_MS );

		// expect that the read result has length of 0x12 (18 bytes)
		xData.CMD_Length = 0x12;
		if( sm130_receiveData(&xData) == xData.CMD_Length )
		{
			if( xData.CMD_Length >= 1 && xData.CMD_Code == CMD_READ16 && xData.CMD_Data[0] == block )
			{
				memcpy((char *)dataBlock, (char *)&xData.CMD_Data[1], xData.CMD_Length -2 );
				return pdTRUE;
			}
		}
	}
	return pdFALSE;
}


/**	Write 16-uint8_t block.
 *
 *
 *
 *	@param block Block number
 *	@param message 16 bytes. Can be a Null terminated string
 */
uint8_t sm130_writeBlock(uint8_t block, const uint8_t *dataBlock) // xxx
{
	xCMDArray xData;

	xData.CMD_Code = CMD_WRITE16;
	xData.CMD_Length = 0x12;
	xData.CMD_Data[0] = block;
	memcpy((char*)&xData.CMD_Data[1], dataBlock, 0x10);

	if( sm130_transmitData(&xData) )
	{
		 // wait 5ms for the command to be executed, and a response prepared.
		 // we have 50ms to get the response, before it is vaporised.

		vTaskDelay( 5 / portTICK_PERIOD_MS );

		// expect that the write result has length of 0x12, being the data read back
		xData.CMD_Length = 0x12;
		if( sm130_receiveData(&xData) > 0 )
		{
			if( xData.CMD_Length >= 1 && xData.CMD_Code == CMD_WRITE16 && xData.CMD_Data[0] == block)
					return pdTRUE;
		}
	}
	return pdFALSE;
}

/**	Write Master Key block.
 *
 *
 *
 *	@param block Block number
 *	@param message 16 bytes. Can be a Null terminated string
 */
uint8_t sm130_writeKey(uint8_t block, uint8_t keyType, uint8_t *key)
{
	xCMDArray xData;

	xData.CMD_Code = CMD_WRITE_KEY;
	xData.CMD_Length = 0x9;
	xData.CMD_Data[0] = block;
	xData.CMD_Data[1] = keyType;
	memcpy((char*)&xData.CMD_Data[2], key, 6);

	if( sm130_transmitData(&xData) )
	{
		 // wait 5ms for the command to be executed, and a response prepared.
		 // we have 50ms to get the response, before it is vaporised.

		vTaskDelay( 5 / portTICK_PERIOD_MS );

		// expect that the Key write result has length of 2
		xData.CMD_Length = 0x02;
		if( sm130_receiveData(&xData) == xData.CMD_Length )
		{
			if( xData.CMD_Code == CMD_WRITE_KEY)
			{
				return xData.CMD_Data[0];
			}
		}
	}
	return pdFALSE;
}



/**	Turns on/off the RF field.
 *
 *	@param level 0 is off, anything else is on
 */
uint8_t sm130_setAntennaPower(uint8_t level)
{
	xCMDArray xData;

	xData.CMD_Length = 0x02;
	xData.CMD_Code = CMD_ANTENNA_POWER;
	if (level != 0)	level = 0x01;
	xData.CMD_Data[0] = level;

	if( sm130_transmitData(&xData) )
	{
		 // wait 5ms for the command to be executed, and a response prepared.
		 // we have 50ms to get the response, before it is vaporised.

		vTaskDelay( 5 / portTICK_PERIOD_MS );

		// expect that the response has length of 2
		xData.CMD_Length = 0x02;
		if( sm130_receiveData(&xData) == xData.CMD_Length )
		{
			if( xData.CMD_Code == CMD_ANTENNA_POWER && xData.CMD_Data[0] == level )
				return pdTRUE;
		}
	}
	return pdFALSE;
}

/**	Halts the tag searching.
 *
 *	@param level 0 is off, anything else is on
 */
uint8_t sm130_haltTag(void)
{
	xCMDArray xData;

	xData.CMD_Length = 0x01;
	xData.CMD_Code = CMD_HALT_TAG;

	if( sm130_transmitData(&xData) )
	{
		 // wait 5ms for the command to be executed, and a response prepared.
		 // we have 50ms to get the response, before it is vaporised.

		vTaskDelay( 5 / portTICK_PERIOD_MS );

		// expect that the response has length of 2
		xData.CMD_Length = 2;
		if( sm130_receiveData(&xData) == xData.CMD_Length )
		{
			if( xData.CMD_Code == CMD_HALT_TAG || xData.CMD_Code == RESP_FAIL_RF_OFF )
				return pdTRUE;
		}
	}
	return pdFALSE;
}

/**	Turns on/off the RF field.
 *
 *	@param level 0 is off, anything else is on
 */
uint8_t sm130_sleep(void)
{
	xCMDArray xData;

	xData.CMD_Length = 0x01;
	xData.CMD_Code = CMD_SLEEP;

	if( sm130_transmitData(&xData) )
	{
		 // wait 5ms for the command to be executed, and a response prepared.
		 // we have 50ms to get the response, before it is vaporised.

		vTaskDelay( 5 / portTICK_PERIOD_MS );

		// expect that the response has length of 2
		xData.CMD_Length = 2;
		if( sm130_receiveData(&xData) == xData.CMD_Length )
		{
			if ( xData.CMD_Code == CMD_SLEEP && xData.CMD_Data[0] == 0x00 )
				return pdTRUE;
		}
	}
	return pdFALSE;
}


/******************** global helper functions ********************************/

uint8_t getNextLibraryKey(uint8_t *hashKey, uint16_t keyNumber)
{
	uint8_t check = 0x00;

	for (uint8_t i = 0; i < 6; ++i) // there are 6 bytes in a key
	{
		hashKey[i] = pgm_read_byte(&keySpace[keyNumber][i]); // Set the next key to be tried.
		check = check | hashKey[i];
	}
	if ( check == 0x00) return pdFALSE; // check for null key, to exit library search
	return pdTRUE;
}

#define   RANDOM_BITS 		32		// bits of data delivered by random()
#define   BITS_NEEDED		48		// bits needed for the key generation

void getNextRandomKey(uint8_t *hashKey)
{
	uint64_t key = 0;
	uint8_t bits_have = 0;

	/* Concatenate enough pseudo-random bits (need 48 bits or 6 bytes) */
	for (bits_have = 0; bits_have < BITS_NEEDED; bits_have += RANDOM_BITS)
		key |= ((uint64_t) random()) << bits_have;

	/* We got at most 64 bits. */
	if (bits_have > 64)
		bits_have = 64;

	/* Strip excess bits. */
	if (bits_have > BITS_NEEDED)
		key >>= bits_have - BITS_NEEDED;

	longToSixByteArray(hashKey, key);
}


void getNextBruteKey(uint8_t *hashKey)
{
	uint64_t key = sixByteArrayToLong(hashKey) + 1;
	longToSixByteArray(hashKey, key);
}


uint8_t compliantNextKey(uint8_t* hashKey)
{
	return (sixByteArrayToLong(hashKey) <= 0xffffffffffffL) ? pdTRUE : pdFALSE;
}



/**	Print uint8_t array as ASCII string.
 *
 *	Non-printable characters (<0x20 or >0x7E) are printed as dot.
 *
 *	@param	array uint8_t array
 *	@param	len length of uint8_t array
 */
void xSerialPrintArrayASCII(uint8_t *array, uint8_t len)
{
  for (uint8_t i = 0; i < len;)
  {
    uint8_t c = array[i++];
    if (c < 0x20 || c > 0x7e)
    {
      xSerialPrint_P(PSTR("."));
    }
    else
    {
      xSerialPrintf_P(PSTR("%s"),c);
    }
  }
}


/**	Convert uint8_t array to null-terminated hexadecimal string.
 *
 *	@param	s	pointer to destination string
 *	@param	array	uint8_t array to convert
 *	@param	len		length of uint8_t array to convert
 */
void arrayToHex(uint8_t *s, uint8_t *array, uint8_t len)
{
	for (uint8_t i = 0; i < len; i++)
	{
		*s++ = toHex(array[i] >> 4);
		*s++ = toHex(array[i]);
	}
	*s = 0;
}



/********************* private functions ********************************/

/*	Transmit a packet with checksum to the SM130. */

static uint8_t sm130_transmitData( pCMDArray xTransmitData) // xxx
{

	uint8_t i;
	uint8_t csum;

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
			As soon as the module receives the complete command frame, it starts executing the
			command. When the module executes the command, if the Master reads from the
			module, the value returned will be 0x00. When the command execution is complete, the
			length of response will be returned. Once the Master knows the length of response, it
			should read the further response.

			Following is the logic the Master should follow to send a command over I2C
				1. Set up I2C as per data sheet protocol
				2. Send Start
				3. Read a single byte from the module. This byte is the length of the response that the
					module is going to send. This will be zero, till the module completes the execution of
					the command. If the read byte is zero, send a stop.
				4. Repeat steps 2 and 3 till the length of response byte is non-zero.
				5. When the length byte is non-zero, instead of sending a stop, read the number of
					data bytes indicated by the response length.
				6. Send Stop.
				7. After reading all the bytes, verify the checksum of the received packet
			*/

			// set device address and write mode
			xTransmitData->I2CAddress = (SM130_RFID<<I2C_ADR_BITS) + I2C_WRITE;

			// calculate the checksum by adding all the bytes (except the I2C address)
			csum = xTransmitData->CMD_Length + xTransmitData->CMD_Code;
			i = 0;
			while ( i < xTransmitData->CMD_Length - 1)
			{
				csum = csum + xTransmitData->CMD_Data[i++];
			}
			// then put the checksum at the end of the command data
			xTransmitData->CMD_Data[i] = csum;

			// push the array to the I2C transmit buffer
			// Transmit length includes: I2C address, length, and checksum. Therefore +3 on CMD_Length.
			I2C_Master_Start_Transceiver_With_Data( (uint8_t *)xTransmitData, xTransmitData->CMD_Length + 3);
			xSemaphoreGive( xI2CSemaphore );
			return pdTRUE;
		}
	}
	return  pdFALSE;           // return 0 to signify failure.
}

/**	Receives a packet from the SM130 and verifies the checksum.
 *
 *	@param length the number of bytes to receive
 *	@return the number of bytes in the payload, or 0 if bad checksum
 */
static uint8_t sm130_receiveData(pCMDArray xReceiveData) // xxx
{
	uint8_t i; // also used as the expected length of data to be received.
	uint8_t j; // receive loop count
	uint8_t csum;


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
			As soon as the module receives the complete command frame, it starts executing the
			command. When the module executes the command, if the Master reads from the
			module, the value returned will be 0x00. When the command execution is complete, the
			length of response will be returned. Once the Master knows the length of response, it
			should read the further response.

			It is recommended that user wait around 5 to 10 milliseconds to get response
			from SM130 after a command is sent. This is to give time to SM130 make its
			operations with less interrupt. Moreover, when checking continuously for a
			ready response from SM130, use some delay (i.e. 5ms) between each read.

			Following is the logic the Master should follow to send a command over I2C
				1. Send I2C command as per above protocol
				2. Send Start
				3. Read a single byte from the module. This byte is the length of the response that the
					module is going to send. This will be zero, till the module completes the execution of
					the command. If the read byte is zero, send a stop.
				4. Repeat steps 2 and 3 till the length of response byte is non-zero.
				5. When the length byte is non-zero, instead of sending a stop, read the number of
					data bytes indicated by the response length.
				6. Send Stop.
				7. After reading all the bytes, verify the checksum of the received packet
			*/

			// set device address and read mode
			xReceiveData->I2CAddress = (SM130_RFID<<I2C_ADR_BITS) + I2C_READ;

			i = xReceiveData->CMD_Length + 3;	// Set the expected receive data length
												// this includes I2c address, length and checksum bytes,
												// in addition to command and data bytes counted by CMD_Length. Hence +3.
												// Get it out here, otherwise it would be overwritten when receiving null bytes.

			j = 0;								// Loop 9 times, each 5 ms (maximum validity of data is 50 ms).
			// read response
			do{
				// start the transceiver with the address, and ask for all the bytes
				// since we've set I2C_READ, only the first (address) byte is needed
				I2C_Master_Start_Transceiver_With_Data( (uint8_t *)xReceiveData, i);

				// get received data into the buffer
				I2C_Master_Get_Data_From_Transceiver( (uint8_t *)xReceiveData, i);

				if (xReceiveData->CMD_Length == 0) // if we got nothing, then wait.
					vTaskDelay( 5 / portTICK_PERIOD_MS );
				else
					break; // we're golden. onwards.

			}while( ++j < 9 );	// Loop 9 times (maximum validity of data).

			// verify checksum because length > 0 and <= maximum feasible receive size
			if ( xReceiveData->CMD_Length <= (I2C_BUFFER_SIZE - 4))
			{
				// calculate the checksum by adding all the bytes
				csum = xReceiveData->CMD_Length + xReceiveData->CMD_Code;
				i = 0; // reuse i for index.
				while ( i < xReceiveData->CMD_Length - 1)
				{
					csum = csum + xReceiveData->CMD_Data[i++];
				}

				xSemaphoreGive( xI2CSemaphore );
				// return with length of response, or 0xFF if invalid checksum
				return csum == xReceiveData->CMD_Data[i] ? xReceiveData->CMD_Length : 0xFF;
			}

			xSemaphoreGive( xI2CSemaphore );
		}
	}
	return  pdFALSE;           // return 0 to signify failure.
}


static uint64_t sixByteArrayToLong(uint8_t* hashKey)
{
	uint64_t l = ((uint64_t)hashKey[0]&0xff) << 40 | ((uint64_t)hashKey[1]&0xff) << 32 | ((uint64_t)hashKey[2]&0xff) << 24 | ((uint64_t)hashKey[3]&0xff) << 16 | ((uint64_t)hashKey[4]&0xff) << 8 | ((uint64_t)hashKey[5]&0xff);
	return l;
}

static void longToSixByteArray(uint8_t *hashKey, uint64_t number)
{
	hashKey[0] = (uint8_t) (number >> 40 & (uint8_t) 0xff);
	hashKey[1] = (uint8_t) (number >> 32 & (uint8_t) 0xff);
	hashKey[2] = (uint8_t) (number >> 24 & (uint8_t) 0xff);
	hashKey[3] = (uint8_t) (number >> 16 & (uint8_t) 0xff);
	hashKey[4] = (uint8_t) (number >> 8  & (uint8_t) 0xff);
	hashKey[5] = (uint8_t) (number       & (uint8_t) 0xff);
}



/**	Convert low-nibble of uint8_t to ASCII hex.
 *
 *	@param	b	uint8_t to convert
 *	$return	upper case hexadecimal character [0-9A-F]
 */
static inline uint8_t toHex(uint8_t b)
{
	b = b & 0x0f;
	return b < 10 ? b + '0' : b + 'A' - 10;
}


