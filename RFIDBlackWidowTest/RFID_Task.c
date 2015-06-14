
/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"


/* serial interface include file. */
#include "serial.h"

/* i2c Interface include file. */
#include "i2cMultiMaster.h"

/* RFID Reader code */
#include "avr-sm130/sm130.h"

/* CLI include file. */
#include "xatoi.h"

/*--------------Global Variables--------------------*/

uint8_t * LineBuffer;					// put line buffer on heap (with pvPortMalloc).

extern const uint8_t PROGMEM keySpace[][6];


/*--------------PrivateFunctions-------------------*/

// get a command line off the serial port.
static void get_line (uint8_t *buff, uint8_t len);

/*-------------------------------------------------*/

// define which type of key search to use
//#define RANDOM
//#define BRUTE
#define LIBRARY

#define LINE_SIZE 		32				// size of command line (on heap)


/*-----------------------------------------------------------*/
void vRFID_Task(void *pvParameters) // RFID Processing
{
    (void) pvParameters;

    uint8_t key[6];			// storage for the key used to open the card
	uint8_t data[16];		// storage for the data to write to and read from the card
	uint8_t string[33];		// data converted to a string for printing
	uint8_t authResult;		// flag to show whether we've opened the sector

    TickType_t xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

	// initialise I2C master interface, need to do this once only.
	// If there are two I2C processes, then do it during the system initiation.
	I2C_Master_Initialise((ARDUINO<<I2C_ADR_BITS) | (pdTRUE<<I2C_GEN_BIT));

	// create the CLI Line buffer on the heap (so they can be moved later).
	if(LineBuffer == NULL) // if there is no Line buffer allocated (pointer is NULL), then allocate buffer.
		if( !(LineBuffer = (uint8_t *) pvPortMalloc( sizeof(uint8_t) * LINE_SIZE )))
			xSerialPrint_P(PSTR("pvPortMalloc for *LineBuffer fail..!\r\n"));

	// Reset RFID Shield, this will also configure IO pins DREADY and RESET
	sm130_reset(pdTRUE); // TRUE is hardware reset - FALSE is software reset.

	for (uint8_t i = 0; i < 6; ++i)	data[i] = 0; // clear data array for firmware.

	// Read firmware version
	while (sm130_getFirmwareVersion(data) != pdTRUE)
			vTaskDelayUntil( &xLastWakeTime, ( 20 / portTICK_PERIOD_MS ) );

	arrayToHex(string, &data[1], 3);
	xSerialPrintf_P(PSTR("\r\nFirmware: %s"), string);
	vTaskDelayUntil( &xLastWakeTime, ( 100 / portTICK_PERIOD_MS ) );

    while(1)
    {

		for (uint8_t i = 0; i < 6; ++i)	data[i] = 0; // clear data array for tag.

		xSerialPrint_P(PSTR("\r\nSeek Tag... "));
		vTaskDelayUntil( &xLastWakeTime, ( 100 / portTICK_PERIOD_MS ) );

		// Read tag data
		if(sm130_seekTag(data) == RESP_SUCCESS) // Wait on DREADY semaphore for up to 60 seconds,
												// before reissuing this command.
		{
			arrayToHex(string, &data[1], 4);
			xSerialPrintf_P(PSTR("Type: %x  Tag#: %s  "), data[0], string);
			vTaskDelayUntil( &xLastWakeTime, ( 100 / portTICK_PERIOD_MS ) );

#ifdef RANDOM
			srandom( xTaskGetTickCount() ); // seed the random number generator for each attack.
#endif

			for (uint8_t sector = 0; sector < 16; ++sector)
			{
				uint16_t j = 0;	// initialise the first key we're going to use for this sector.
								// can also use this to count the keys we've searched.

				authResult = RESP_FAIL; // set to null to force authentication with new key search for new sector.

				for (uint8_t i = 0; i < 6; ++i)
					key[i] = 0; // clear key array for search in a new sector.
								// only needed to initialise the brute force search.

				for (uint8_t block = 0; block < 4; ++block)
				{
					xSerialPrintf_P(PSTR("\r\nSector: %u  Block: %u"), sector, block);
					vTaskDelayUntil( &xLastWakeTime, ( 100 / portTICK_PERIOD_MS ) );

#ifdef RANDOM
					do{ // do the authentication, searching over random keySpace until the key is correct.

						if(authResult == RESP_FAIL)
						{
							getNextRandomKey(key);

							arrayToHex(string, key, 6);
							xSerialPrintf_P(PSTR("\r\n  Key# %u : %s IS RANDOM -"), j, string); // print the key being used
							vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_PERIOD_MS ) );
						}
#endif

#ifdef LIBRARY
					do{ // do the authentication, searching over library keySpace until the key for the block is correct.

						if( authResult == RESP_FAIL )
						{
							if( (getNextLibraryKey(key, j)))
							{
								arrayToHex(string, key, 6);
								xSerialPrintf_P(PSTR("\r\n  Key# %u: %s IN LIBRARY -"), j, string); // print the key being used
								vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_PERIOD_MS ) );
							} else
								break;
						}
#endif

#ifdef BRUTE

					do{ // do the authentication, searching over brute force keySpace until the key is correct.

						if(authResult == RESP_FAIL)
						{
							getNextBruteKey(key);

							arrayToHex(string, key, 6);
							xSerialPrintf_P(PSTR("\r\n  Key# %u: %s IS BRUTE -"), j, string); // print the key being used
							vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_PERIOD_MS ) );
						}
#endif

						if( compliantNextKey(key))
							authResult = sm130_authenticate( (uint8_t)(sector *4 + block), KEY_A, key); // do the authentication
						else
							break;

						switch(authResult)
						{
							case RESP_SUCCESS:
								xSerialPrint_P(PSTR(" RESP_SUCCESS")); // print the key being used is successful
//								vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_PERIOD_MS ) );

								break;

							case RESP_W_R_FAIL: // Error reading or writing -> try the key again
								xSerialPrint_P(PSTR(" RESP_W_R_FAIL"));
//								vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_PERIOD_MS ) );

								for (uint8_t i = 0; i < 6; ++i)	data[i] = 0; // clear data array for tag.

								while (sm130_selectTag(data) != MIFARE_1K )// reselect the Tag (need to do this if authentication fails).
									vTaskDelayUntil( &xLastWakeTime, ( 5 / portTICK_PERIOD_MS ) );

								break;

							case RESP_FAIL: // Couldn't log in -> try a new key
								xSerialPrint_P(PSTR(" RESP_FAIL"));
//								vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_PERIOD_MS ) );

								for (uint8_t i = 0; i < 6; ++i)	data[i] = 0; // clear data array for tag.

								while (sm130_selectTag(data) != MIFARE_1K )// reselect the Tag (need to do this if authentication fails).
									vTaskDelayUntil( &xLastWakeTime, ( 5 / portTICK_PERIOD_MS ) );

								++j; // select the next library key and / or
									 // just increment the number of keys we've tested.

								break;

							case RESP_INVALID_KEY: // This shouldn't happen. We aren't storing EEPROM keys on SM130
								xSerialPrint_P(PSTR(" RESP_INVALID_KEY"));
//								vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_PERIOD_MS ) );
								break;

							default:
								break;
						}

					} while (authResult != RESP_SUCCESS); // keep on searching till we match keys.

					if( authResult == RESP_SUCCESS ) // if we've got a good key, then grab data from that sector.
					{
						xSerialPrintf_P(PSTR("\r\nSector: %u  Block: %u"), sector, block);
						vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_PERIOD_MS ) );

						for (uint8_t i = 0; i < 16; ++i) data[i] = 0; // clear data array for data.

						while(sm130_readBlock( (uint8_t)(sector *4 + block), data) != pdTRUE) // keep trying until we get a read result.
							for (uint8_t i = 0; i < 16; ++i) data[i] = 0; // clear data array for data.

						arrayToHex(string, data, 16);
						xSerialPrintf_P(PSTR("  Data: %s"), string);
						vTaskDelayUntil( &xLastWakeTime, ( 80 / portTICK_PERIOD_MS ) );
					}
					else
						break;

//		xSerialPrintf_P(PSTR("\r\nvRFID_Task HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
//		vTaskDelayUntil( &xLastWakeTime, ( 200 / portTICK_PERIOD_MS ) );
//		xSerialPrintf_P(PSTR("vRFID_Task Free Heap Size: %u\r\n\n"),xPortGetFreeHeapSize() ); // needs heap_1 or heap_2 or heap_4 for this function to succeed.
//		vTaskDelayUntil( &xLastWakeTime, ( 200 / portTICK_PERIOD_MS ) );

				}
			}
		}
	}
}



		/*-----------------------------------------------------------*/
		/* Monitor                                                   */
		/*-----------------------------------------------------------*/
		static
		void get_line (uint8_t *buff, uint8_t len)
		{
			uint8_t c;
			uint8_t i = 0;

			for (;;) {
				while ( ! xSerialGetChar( &xSerialPort, &c ))
					vTaskDelay( 1 );

				if (c == '\r') break;
				if ((c == '\b') && i) {
					--i;
					xSerialPutChar( &xSerialPort, c );
					continue;
				}
				if (c >= ' ' && i < len - 1) {	/* Visible chars */
					buff[i++] = c;
					xSerialPutChar( &xSerialPort, c );
				}
			}
			buff[i] = 0;
			xSerialPrint_P(PSTR("\r\n"));
		}




const uint8_t PROGMEM keySpace[][6] = // put these sample keys in PROGMEM, to save RAM.
{
//   use pgm_read_byte(&keySpace[y][x]) to get them out of PROGMEM.

		{		0xd3, 0xf7, 0xd3, 0xf7, 0xd3, 0xf7	},
		{		0x1a, 0x98, 0x2c, 0x7e, 0x45, 0x9a	},
		{		0x4d, 0x3a, 0x99, 0xc3, 0x51, 0xdd	},
		{		0xbd, 0x49, 0x3a, 0x39, 0x62, 0xb6	},
		{		0x11, 0x23, 0x43, 0xfc, 0x97, 0xcd  },
		{		0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5	},

		{		0xff, 0xff, 0xff, 0xff, 0xff, 0xff	},

		{		0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff	},
		{		0x01, 0x02, 0x03, 0x04, 0x05, 0x06	},
		{		0x01, 0x23, 0x45, 0x67, 0x89, 0xab	},
		{		0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc	},

		{		0x11, 0x11, 0x11, 0x11, 0x11, 0x11	},
		{		0x22, 0x22, 0x22, 0x22, 0x22, 0x22	},
		{		0x33, 0x33, 0x33, 0x33, 0x33, 0x33	},
		{		0x44, 0x44, 0x44, 0x44, 0x44, 0x44	},
		{		0x55, 0x55, 0x55, 0x55, 0x55, 0x55	},
		{		0x66, 0x66, 0x66, 0x66, 0x66, 0x66	},
		{		0x77, 0x77, 0x77, 0x77, 0x77, 0x77	},
		{		0x88, 0x88, 0x88, 0x88, 0x88, 0x88	},
		{		0x99, 0x99, 0x99, 0x99, 0x99, 0x99	},
		{		0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa	},
		{		0xbb, 0xbb, 0xbb, 0xbb, 0xbb, 0xbb	},
		{		0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc	},
		{		0xdd, 0xdd, 0xdd, 0xdd, 0xdd, 0xdd	},
		{		0xee, 0xee, 0xee, 0xee, 0xee, 0xee	},

		{		0xa0, 0xb0, 0xc0, 0xd0, 0xe0, 0xf0	},
		{		0xa1, 0xb1, 0xc1, 0xd1, 0xe1, 0xf1	},
		{		0xa2, 0xb2, 0xc2, 0xd2, 0xe2, 0xf2	},
		{		0xa3, 0xb3, 0xc3, 0xd3, 0xe3, 0xf3	},
		{		0xa4, 0xb4, 0xc4, 0xd4, 0xe4, 0xf4	},
		{		0xa5, 0xb5, 0xc5, 0xd5, 0xe5, 0xf5	},
		{		0xa6, 0xb6, 0xc6, 0xd6, 0xe6, 0xf6	},
		{		0xa7, 0xb7, 0xc7, 0xd7, 0xe7, 0xf7	},
		{		0xa8, 0xb8, 0xc8, 0xd8, 0xe8, 0xf8	},
		{		0xa9, 0xb9, 0xc9, 0xd9, 0xe9, 0xf9	},
		{		0xaa, 0xba, 0xca, 0xda, 0xea, 0xfa	},
		{		0xab, 0xbb, 0xcb, 0xdb, 0xeb, 0xfb	},
		{		0xac, 0xbc, 0xcc, 0xdc, 0xec, 0xfc	},
		{		0xad, 0xbd, 0xcd, 0xdd, 0xed, 0xfd	},
		{		0xae, 0xbe, 0xce, 0xde, 0xee, 0xfe	},
		{		0xaf, 0xbf, 0xcf, 0xcf, 0xef, 0xff	},

		{		0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5	},
		{		0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5	},
		{		0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5	},
		{		0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5	},
		{		0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5	},

		{		0x00, 0x00, 0x00, 0x00, 0x00, 0x00	} // set this null key to exit the library
}; // */

