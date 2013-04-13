////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////    main.c
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>


/* Scheduler include files. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <digitalAnalog.h>
#include <lib_serial.h>


/* Microbridge include files. */
#include <avr.h>
#include <adb.h>

/* Danger ADB include files. */
#include "danger.h"

/*-----------------------------------------------------------*/

/* Create a handle for the serial port. */
xComPortHandle xSerialPort;

// Create a Semaphore mutex flag for the ADC. To ensure only single access.
xSemaphoreHandle xADCSemaphore;


// Variables for the analogue conversion on ADC Sensors
xADCArray values;         					// holds the return values from the ADC

adb_connection * shell;		// create adb connection: shell
adb_connection * connection;	// create adb connection: tcp

/*-----------------------------------------------------------*/

// Event handler to process incoming data from ADB.
static void adbEventHandler(adb_connection * connection, adb_eventType event, uint16_t length, uint8_t * data)
{
	switch (event)
	{
	case ADB_CONNECT:
		xSerialPrint_P(PSTR("ADB EVENT CONNECT\r\n"));
		break;
	case ADB_DISCONNECT:
		xSerialPrint_P(PSTR("ADB EVENT DISCONNECT\r\n"));
		break;
	case ADB_CONNECTION_OPEN:
		xSerialPrintf_P(PSTR("ADB EVENT OPEN connection=[%s]\r\n"), connection->connectionString);
		break;
	case ADB_CONNECTION_CLOSE:
		xSerialPrintf_P(PSTR("ADB EVENT CLOSE connection=[%s]\r\n"), connection->connectionString);
		break;
	case ADB_CONNECTION_FAILED:
		xSerialPrintf_P(PSTR("ADB EVENT FAILED connection=[%s]\r\n"), connection->connectionString);
		break;
	case ADB_CONNECTION_RECEIVE:

		for (int16_t i=0; i<length; i++)
			xSerialPrintf_P(PSTR("%c"), data[i]);

		break;
	}

}

/*-----------------------------------------------------------*/


/* Main program loop */
int16_t main(void) __attribute__((OS_main));

int16_t main(void)
    {

    vSemaphoreCreateBinary( xADCSemaphore ); // binary semaphore for ADC

    xSerialPort = xSerialPortInitMinimal( 115200, 80, 16); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)

    avrSerialPrint_P(PSTR("\r\n\n\n\nHello World!\r\n")); // Ok, so we're alive...

    xTaskCreate(
		TaskBlinkYellowLED
		,  (const signed portCHAR *)"YellowLED"
		,  168 // tested 2 free
		,  NULL
		,  2
		,  NULL );

   xTaskCreate(
		TaskADB
		,  (const signed portCHAR *)"ADB"
		,  720
		,  NULL
		,  1
		,  NULL ); // */


/*    xTaskCreate(
		TaskWrite7SEG
		,  (const signed portCHAR *)"Write7SEG"
		,  168 // tested 4 free
		,  NULL
		,  3
		,  NULL ); // */

    avrSerialPrintf_P(PSTR("\r\n\nFree Heap Size: %u\r\n"),xPortGetFreeHeapSize() );

    vTaskStartScheduler();

    }


/*-----------------------------------------------------------*/


static void TaskBlinkYellowLED(void *pvParameters) // Main Yellow LED Flash
    {
    (void) pvParameters;;
    portTickType xLastWakeTime;
	// The xLastWakeTime variable needs to be initialised with the current tick
	// count.  Note that this is the only time we access this variable.  From this
	// point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	// API function.
	xLastWakeTime = xTaskGetTickCount();

	setDigitalOutput(IO_D5, LOW); // initialise Yellow D5 LED
	setDigitalOutput(IO_D6, LOW); // initialise Yellow D6 LED + Used for testing only

	uint8_t cRxedChar; // store a received character

    while(1)
        {
    	setDigitalOutput( IO_D6, 0);               // main (green IO_D6) LED off
		vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_RATE_MS ) );

		setDigitalOutput( IO_D6, 1);               // main (green IO_D6) LED on
		vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_RATE_MS ) );

        while(  xSerialGetChar( xSerialPort, &cRxedChar, xNoBlock ) )
		{
			xSerialPutChar( xSerialPort,  cRxedChar, xNoBlock );
//        	if (shell->status==ADB_OPEN)
			{
//				adb_write(shell, 1, cRxedChar);
			}
		}
//		xSerialPrintf_P(PSTR("\r\nYellowLED HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
        }
    }

/*-----------------------------------------------------------*/

static void TaskADB(void *pvParameters) // ADB Setup and Poll
    {
    (void) pvParameters;;
    portTickType xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

	adb_init(); 					// Initialise USB host shield.

	// Create a new ADB connection, run command (eg. logcat) on the phone
	shell = adb_addConnection("shell:exec", true, adbEventHandler);

	xSerialPrintf_P(PSTR("adb_addConnection: %u @ Tick: %u\r\n"), shell->connectionString, xTaskGetTickCount() ); // FIXME remove this debugging

	// Create a new ADB connection, open TCP port 4567 on the phone
	connection = adb_addConnection("tcp:4567", true, adbEventHandler);

	xSerialPrintf_P(PSTR("adb_addConnection: %u @ Tick: %u\r\n"), connection->connectionString, xTaskGetTickCount() ); // FIXME remove this debugging


	while(1)
        {


		setDigitalOutput( IO_D5, 0);               // yellow D5 LED off
		vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_RATE_MS ) );

		adb_poll();

		setDigitalOutput( IO_D5, 1);               // yellow D5 LED on
		vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_RATE_MS ) );

    	ReadADCSensors(); // use this slow task to read ADC and write global values.

		if (connection->status==ADB_OPEN) // this is the link to Android
			adb_write(connection, 1, &values.adc1);

		xSerialPrintf_P(PSTR("\r\nADB HighWater @ %u"), uxTaskGetStackHighWaterMark(NULL));

        }
    }


/*-----------------------------------------------------------*/




static void TaskWrite7SEG(void *pvParameters) // Write to 7 Segment display (via shift register)
{
    (void) pvParameters;

    portTickType xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

    uint8_t i;
    uint8_t character;

    setDigitalOutput(LATCH, LOW);				// Latch  IO_D7 // 7
    setDigitalOutput(CLOCK, LOW);				// Clock  IO_B0 // 8
    setDigitalOutput(DATA,  LOW);				// Data   IO_D4 // 4

    while(1)
	{
    	ReadADCSensors(); // use this slow task to read ADC and write global values.

		i = (values.adc1 >> 3); // reduce the 8 bit value to 5 bits (32 values, 7 segment) code

		character = pgm_read_byte(&ledCharSet[i]); // retrieve the character from PROGMEM; only do this once.

//			if (connection->status==ADB_OPEN) // this is the link to Android
//				adb_write(connection, 1, &xValues.adc1);

		shiftOut(DATA,CLOCK,LATCH,MSBFIRST,~(character | 0b10000000)); // turn on decimal point
		vTaskDelayUntil( &xLastWakeTime, ( 40 / portTICK_RATE_MS ) );
		shiftOut(DATA,CLOCK,LATCH,MSBFIRST,~(character & 0b01111111)); // turn off decimal point

//		xSerialPrintf_P(PSTR("A0: %3u, A1: %3u, A2: %3u, Photo: %3u \r"), values.adc0, values.adc1, values.adc2, values.adc3);
//		xSerialPrintf_P(PSTR("Write7Seg HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));

		vTaskDelayUntil( &xLastWakeTime, ( 160 / portTICK_RATE_MS ) );
//		taskYIELD();     // yield until we want to display again
	}
}



/*-----------------------------------------------------------*/


static void ReadADCSensors(void)  // Read ADC Sensors
{

	if( xADCSemaphore != NULL )
	{
		// See if we can obtain the semaphore.  If the semaphore is not available
		// wait 10 ticks to see if it becomes free.
		if( xSemaphoreTake( xADCSemaphore, ( portTickType ) 10 ) == pdTRUE )
		{
		// We were able to obtain the semaphore and can now access the
		// shared resource.
		// We want to have the ADC for us alone, as it takes some time to sample,
		// so we don't want it getting stolen during the middle of a conversion.

			setAnalogMode(MODE_8_BIT);    // 8-bit analogue-to-digital conversions

			startAnalogConversion(0, 0);   // start next conversion
			while( analogIsConverting() )
				_delay_us(25);     // yield until conversion ready

			values.adc0 = analogConversionResult();

			startAnalogConversion(1, 0);   // start next conversion
			while( analogIsConverting() )
				 _delay_us(25);       // yield until conversion ready

			values.adc1 = analogConversionResult();

			startAnalogConversion(2, 0);   // start next conversion
			while( analogIsConverting() )
				 _delay_us(25);      // yield until conversion ready

			 values.adc2 = analogConversionResult();

			startAnalogConversion(3, 0);   // start next conversion
			while( analogIsConverting() )
				 _delay_us(25);      // yield until conversion ready

			 values.adc3 = analogConversionResult();

			xSemaphoreGive( xADCSemaphore );

			return;
		}
	}
}

/*-----------------------------------------------------------*/

//   uint8_t character;

//	character = pgm_read_byte(&ledCharSet[(values.adc1 >> 3)]); // retrieve the character from PROGMEM; only do this once.
// reduce the 8 bit value to 5 bits (32 values, 7 segment) code

//	shiftOut(DATA,CLOCK,LATCH,MSBFIRST,~(character | 0b10000000)); // turn on decimal point

//	shiftOut(DATA,CLOCK,LATCH,MSBFIRST,~(character & 0b01111111)); // turn off decimal point


static void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t latchPin, uint8_t bitOrder, uint8_t bitVal)
{
  uint8_t i;
  setDigitalOutput(latchPin,LOW); // prepare the shift register storage to receive.
  for (i = 0; i < 8; i++)
  {
	  setDigitalOutput(clockPin, LOW);
	  if (bitOrder == LSBFIRST)
		  setDigitalOutput(dataPin, bitVal & (1 << i));
	  else
		  setDigitalOutput(dataPin, bitVal & (1 << (7 - i)));
	  _delay_us( 0.1 );  // needs 100ns.
	  setDigitalOutput(clockPin, HIGH); // move data along shift registers on +ve edge.
	  _delay_us( 0.1 );   // needs 100ns.
  }
  setDigitalOutput(latchPin,HIGH); // move the shift register values into store on +ve edge.
}


/*-----------------------------------------------------------*/

