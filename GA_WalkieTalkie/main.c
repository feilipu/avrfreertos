////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////    main.c
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include <avr/io.h>
#include <avr/interrupt.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* ring buffer include file. */
#include "ringBuffer.h"

/* serial interface include file. */
#include "serial.h"

/* i2c Interface include file. */
#include "i2cMultiMaster.h"

/* SPI interface include file. */
#include "spi.h"

/* system time include file. */
#include "time.h"

#ifdef portHD44780_LCD
/* LCD (Freetronics 16x2) interface include file. */
#include "hd44780.h"
#endif

#ifdef portRTC_DEFINED
/* RTC interface (using I2C) include file. */
#include "rtc.h"
#endif

/* WPAN and Xbee functions */
#include "xbee/xbee_platform.h"
#include "xbee/xbee_device.h"
#include "xbee/xbee_atcmd.h"

/* Goldilocks Analogue and other DAC functions include file. */
#include "DAC.h"

/* CCITT ITU Compression files */
#include "g711.h"

/* Config File */
#include "WT_Config.h"


/*-----------------------------------------------------------*/

/* Main program loop */
int main(void) __attribute__((OS_main));

int main(void)
{
	// turn on the serial port for communications with the Xbee device.
	xSerialPort = xSerialPortInitMinimal( USART0, 115200, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)

	// turn on the serial port for debugging or for other USART reasons.
	xSerial1Port = xSerialPortInitMinimal( USART1, 115200, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)

	//	avrSerialPrint... doesn't need the scheduler running, so we can see freeRTOS initiation issues
	avrSerialxPrintf_P( &xSerial1Port,PSTR("\r\n\nHello World!\r\n")); // Ok, so we're alive...

#ifdef	portHD44780_LCD
	lcd_Init();

	lcd_Print_P(PSTR("Hello World!"));
	lcd_Locate (0, 0);
#endif

    xTaskCreate(
		TaskBlinkRedLED
		,  (const char *)"RedLED" // LED Blink
		,  256				// Tested 9 free @ 208
		,  NULL
		,  1
		,  NULL ); // */


    xTaskCreate(
		TaskWalkieTalkie
		,  (const char *) "WalkieTalkie"
		,  1024  // This stack size can be checked & adjusted by reading Highwater
		,  NULL
		,  3
		,  NULL ); // */


	avrSerialxPrintf_P( &xSerial1Port, PSTR("\r\nFree Heap Size: %u\r\n"),xPortGetMinimumEverFreeHeapSize() ); // needs heap_1.c, heap_2.c or heap_4.c

    vTaskStartScheduler(); // start the task scheduler, which takes over control of individual tasks. Should never return to here.

    avrSerialxPrint_P( &xSerial1Port, PSTR("\r\n\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...

#ifdef portHD44780_LCD
	lcd_Locate (1, 0);
	lcd_Print_P(PSTR("DEAD BEEF!"));
#endif
}

/*--------------------------------------------------*/
/*---------------Task Definitions-------------------*/
/*--------------------------------------------------*/


static void TaskBlinkRedLED(void *pvParameters) // Main Red LED Flash
{
    (void) pvParameters;;
    TickType_t xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

#if defined (portRTC_DEFINED)
	tm CurrTimeDate; 			// set up an array for the RTC info.

	// initialise I2C master interface, need to do this once only.
	// If there are two I2C processes, then do it during the system initiation.
	I2C_Master_Initialise((ARDUINO<<I2C_ADR_BITS) | (pdTRUE<<I2C_GEN_BIT));

#endif

    while(1)
    {
    	xSerialxPrintf_P( &xSerial1Port, PSTR("\r\nFree Heap Size: %u"),xPortGetMinimumEverFreeHeapSize() ); // needs heap_1.c, heap_2.c or heap_4.c

    	vTaskDelayUntil( &xLastWakeTime, ( 2000 / portTICK_PERIOD_MS ) );

#if defined (portHD44780_LCD)
		lcd_Locate (0, 0);
		lcd_Printf_P(PSTR("Min Heap:%7u"), xPortGetMinimumEverFreeHeapSize() ); // needs heap_4 for this function to succeed.

#if defined (portRTC_DEFINED)
		if (getDateTimeDS1307( (ptm)&CurrTimeDate ) == pdTRUE){
			lcd_Locate (1, 8);
			lcd_Printf_P(PSTR("%02u:%02u:%02u"), CurrTimeDate.tm_hour, CurrTimeDate.tm_min, CurrTimeDate.tm_sec);
		}
#else
		lcd_Locate (1, 0);
		lcd_Printf_P(PSTR("Sys Tick:%7lu"), time(NULL));
#endif // portRTC_DEFINED

#endif // portHD44780_LCD

    }
}


static void TaskWalkieTalkie(void *pvParameters) // Operate the WalkieTalkie Network
{
	(void) pvParameters;

	int status;
	xbee_command_list_context_t clc;
	xb_buf buffer;

	/* Create the ring-buffer used by transmitter section. */
	if( (TxDataPtr = (uint8_t *)pvPortMalloc( sizeof(uint16_t) * XBEE_PACKET )))
		ringBuffer_InitBuffer( &TxBuffer, TxDataPtr, sizeof(uint16_t) * XBEE_PACKET);
	else
		xSerialxPrint_P( &xSerial1Port, PSTR("\r\nTx Ringbuffer malloc failure"));

	/* Create the ring-buffer used by receiver section. */
	if( (RxDataPtr = (uint8_t *)pvPortMalloc( sizeof(uint16_t) * XBEE_PACKET )))
		ringBuffer_InitBuffer( &RxBuffer, RxDataPtr, sizeof(uint16_t) * XBEE_PACKET);
	else
		xSerialxPrint_P( &xSerial1Port, PSTR("\r\nRx Ringbuffer malloc failure"));

	xSerialxPrintf_P( &xSerial1Port, PSTR("\r\nWalkieTalkie Initial HighWater: %u\r\n"), uxTaskGetStackHighWaterMark(NULL));


	xSerialxPrint_P( &xSerial1Port, PSTR("\r\nDAC_Codec_init:"));
//	DAC_init();

	xSerialxPrint_P( &xSerial1Port, PSTR(" will"));

	/* Initialise the sample interrupt timer. Exact multiples of 2000Hz are ok with 8 bit Timer0, otherwise use 16 bit Timer1 */
//	AudioCodec_Timer0_init(SAMPLE_RATE);	// xxx set up the sampling Timer0 to 48000Hz (or lower), runs at audio sampling rate in Hz.

	xSerialxPrint_P( &xSerial1Port, PSTR(" soon"));

	tx_filter.cutoff = 0xc000;				// set filter to 3/8 of sample frequency (0xffff is 1/2 sample frequency)
	setIIRFilterLPF( &tx_filter );			// initialise transmit sample filter

	xSerialxPrint_P( &xSerial1Port, PSTR(" be"));
	AudioCodec_ADC_init();					// set up ADC sampling on the Microphone on ADC7.

	AudioCodec_setHandler(audioCodec_dsp, &ch_A_out, &ch_B_out); // Set the call back function to do the audio processing.
																 //	Done this way so that we can change the audio handling depending on what we want to achieve.
	xSerialxPrint_P( &xSerial1Port, PSTR(" done."));

	//----------------

	// initialize the serial and device layer for this XBee device
	// XBEE_SERPORT, xbee_awake_pin and xbee_reset_pin setup in xbee_config.h
	// based on the board type being used to run the sample
	status = xbee_dev_init( &my_xbee, &xSerialPort, xbee_awake_pin, xbee_reset_pin);
	if ( status != 0)
		xSerialxPrintf_P( &xSerial1Port, PSTR("\nXBee initialisation error: %d.\n"), status);
	else
		xSerialxPrint_P( &xSerial1Port, PSTR("\nXBee initialisation complete.\n"));

//	char test[3] = "V";
//	xbee_cmd_simple( &my_xbee, test, 0);

//	xbee_dev_dump_settings( &my_xbee, 0);

	// Initialize the AT Command layer for this XBee device and have the
	// driver query it for basic information (hardware version, firmware version,
	// serial number, IEEE address, etc.)
	status = xbee_cmd_init_device( &my_xbee);
	xSerialxPrintf_P( &xSerial1Port, PSTR( "\nWaiting for driver to query the XBee device... Status %d"), status);

	do {
		vTaskDelay( 3 );
		xbee_dev_tick( &my_xbee);
		status = xbee_cmd_query_status( &my_xbee);
	} while (status == -EBUSY);
	if (!status)  //Check for valid response or error condition
   {
		xSerialxPrint_P( &xSerial1Port, PSTR("\nXBee initialisation and query complete.\n\n"));
		xSerialxPrintf_P( &xSerial1Port, PSTR("XBee Hardware Version: 0x%04x\n"), my_xbee.hardware_version);
		xSerialxPrintf_P( &xSerial1Port, PSTR("XBee Firmware Version: 0x%08lx\n"), my_xbee.firmware_version);
   }
   else
   {  // Error detected, display code and error message
//	   xSerialxPrintf_P( &xSerial1Port, PSTR( "Xbee_init failed. Result code: %d (%ls)\n"), status, error_message(status));
	   xSerialxPrintf_P( &xSerial1Port, PSTR( "\nXbee_init failed. Result code: %d"), status );
   }

    // Execute pre-defined list of commands and save results to buffer
		xbee_cmd_list_execute( &my_xbee, &clc, query_regs, &buffer, NULL );
		while ((status = xbee_cmd_list_status(&clc)) == XBEE_COMMAND_LIST_RUNNING)
		{
			vTaskDelay( 3 );
			xbee_dev_tick( &my_xbee );      // Drive device layer to completion
		}

    if (status == XBEE_COMMAND_LIST_TIMEOUT)   // Check for timeout condition
    {
    	xSerialxPrint_P( &xSerial1Port, PSTR("XBee command list timed out.\n"));
    }
    else
    {  // Command list completed
	   if (status == XBEE_COMMAND_LIST_ERROR)   // Check for complete w/error
       {
		   xSerialxPrint_P( &xSerial1Port, PSTR("XBee command list completed with errors.\n\n"));
       }

	   xSerialxPrintf_P( &xSerial1Port, PSTR("XBee serial baud rate: %ld\n"), xbee_baud[buffer.baud]);
	   xSerialxPrintf_P( &xSerial1Port, PSTR("XBee max network payload: %d\n"), buffer.payload);
    }


#if 0
#endif
	//----------------

//	vTaskSuspend(NULL);						// Well, we're pretty much done here.
//	vTaskEndScheduler();					// Rely on Timer1 Interrupt for regular output.

	for(;;)
	{
		xSerialxPrintf_P( &xSerial1Port, PSTR("\r\nWalkieTalkie HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
		vTaskDelay( 2000 / portTICK_PERIOD_MS );
	}
}


/*--------------------------------------------------*/
/*------------Analogue Processing HERE--------------*/
/*--------------------------------------------------*/


void audioCodec_dsp( uint16_t * ch_A,  uint16_t * ch_B)
{
	int16_t xn;
	uint8_t cn;
	/*----- Audio Rx -----*/

	/* Get the next character from the ring buffer. */

	if( ringBuffer_IsEmpty( (ringBuffer_t*) &(xSerialPort.xRxedChars) ) )
	{
		cn = 0x80 ^ 0x55; // put A-Law nulled signal on the output.
	}
	else if (ringBuffer_GetCount( &(xSerialPort.xRxedChars) ) > (portSERIAL_BUFFER_RX>>1) ) // if the buffer is more than half full.
	{
		cn = ringBuffer_Pop( (ringBuffer_t*) &(xSerialPort.xRxedChars) ); // pop two samples to catch up, discard first one.
		cn = ringBuffer_Pop( (ringBuffer_t*) &(xSerialPort.xRxedChars) );
	}
	else
	{
		cn = ringBuffer_Pop( (ringBuffer_t*) &(xSerialPort.xRxedChars) ); // pop a sample
	}

	alaw_expand1(&cn, &xn);	// expand the A-Law compression

	*ch_A = *ch_B = (uint16_t)(xn + 0x7fff); // move the signal to positive values, and put signal out on A & B channel.

	/*----- Audio Tx -----*/

	AudioCodec_ADC( &mod7_value.u16 );	// sample is 10bits left justified.

	xn = mod7_value.u16 - 0x7fe0;	// centre the sample to 0 by subtracting 1/2 10bit range.

	IIRFilter( &tx_filter, &xn);	// filter transmitted sample train

	alaw_compress1(&xn, &cn);	// compress using A-Law

	xSerialPutChar( &xSerialPort, cn);	// transmit the sample
}


