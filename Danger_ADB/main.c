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
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "digitalAnalog.h"
#include "serial.h"

/* Microbridge include files. */
#include "adb.h"

/* Danger Shield include files. */
#include "danger.h"

/*-----------------------------------------------------------*/

/* Create a handle for the serial port. */
xComPortHandle xSerialPort;

// Create a Semaphore mutex flag for the ADC. To ensure only single access.
SemaphoreHandle_t xADCSemaphore;


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
		xSerialPrint_P(PSTR("\r\nADB EVENT CONNECT"));
		break;
	case ADB_DISCONNECT:
		xSerialPrint_P(PSTR("\r\nADB EVENT DISCONNECT"));
		break;
	case ADB_AUTHORISATION:
		xSerialPrintf_P(PSTR("\r\nADB EVENT AUTH connection=[%s]"), connection->connectionString);
		break;
	case ADB_CONNECTION_OPEN:
		xSerialPrintf_P(PSTR("\r\nADB EVENT OPEN connection=[%s]"), connection->connectionString);
		break;
	case ADB_CONNECTION_CLOSE:
		xSerialPrintf_P(PSTR("\r\nADB EVENT CLOSE connection=[%s]"), connection->connectionString);
		break;
	case ADB_CONNECTION_FAILED:
		xSerialPrintf_P(PSTR("\r\nADB EVENT FAILED connection=[%s]"), connection->connectionString);
		break;
	case ADB_CONNECTION_RECEIVE:
		xSerialPrint_P(PSTR("\r\nADB EVENT RECIEVE \r\n"));
		for (int16_t i=0; i<length; ++i)
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

    // turn on the serial port for setting or querying the time .
	xSerialPort = xSerialPortInitMinimal( USART0, 115200, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)

    avrSerialPrint_P(PSTR("\r\n\n\n\nHello World!\r\n")); // Ok, so we're alive...

    xTaskCreate(
		TaskBlinkYellowLED
		,  (const char *)"YellowLED"
		,  168 // tested 2 free
		,  NULL
		,  2
		,  NULL );

   xTaskCreate(
		TaskADB
		,  (const char *)"ADB"
		,  1024
		,  NULL
		,  1
		,  NULL ); // */


/*    xTaskCreate(
		TaskWrite7SEG
		,  (const char *)"Write7SEG"
		,  168 // tested 4 free
		,  NULL
		,  3
		,  NULL ); // */

    avrSerialPrintf_P(PSTR("\r\nFree Heap Size: %u\r\n"),xPortGetFreeHeapSize() );

    vTaskStartScheduler();

    }


/*-----------------------------------------------------------*/


static void TaskBlinkYellowLED(void *pvParameters) // Main Yellow LED Flash
    {
    (void) pvParameters;;
    TickType_t xLastWakeTime;
	// The xLastWakeTime variable needs to be initialised with the current tick
	// count.  Note that this is the only time we access this variable.  From this
	// point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	// API function.
	xLastWakeTime = xTaskGetTickCount();

	DDRB |= _BV(DDB7);

	uint8_t cRxedChar; // store a received character

    while(1)
        {
    	PORTB |=  _BV(PORTB7);       // main (red IO_B7) LED on. EtherMega LED on
		vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_PERIOD_MS ) );

		PORTB &= ~_BV(PORTB7);       // main (red IO_B7) LED off. EtherMega LED off
		vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_PERIOD_MS ) );

        while(  xSerialGetChar( &xSerialPort, &cRxedChar ) )
		{
			xSerialPutChar( &xSerialPort,  cRxedChar );
        	if (shell->status==ADB_OPEN)
			{
				adb_write(shell, 1, &cRxedChar);
			}
		}
//		xSerialPrintf_P(PSTR("\r\nRedLED HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
        }
    }

/*-----------------------------------------------------------*/

static void TaskADB(void *pvParameters) // ADB Setup and Poll
    {
    (void) pvParameters;;
    TickType_t xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

	adb_init(); 					// Initialise USB host shield.

	// Create a new ADB connection, run command (eg. logcat) on the phone
	shell = adb_addConnection("shell:ls /", true, adbEventHandler);

	xSerialPrintf_P(PSTR("\r\nadb_addConnection: %s @ Tick: %u"), shell->connectionString, xTaskGetTickCount() ); // FIXME remove this debugging

	// Create a new ADB connection, open TCP port 4568 on the phone
	connection = adb_addConnection("tcp:4568", true, adbEventHandler);

	xSerialPrintf_P(PSTR("\r\nadb_addConnection: %s @ Tick: %u"), connection->connectionString, xTaskGetTickCount() ); // FIXME remove this debugging

	while(1)
        {

		vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_PERIOD_MS ) );

		adb_poll();

		vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_PERIOD_MS ) );

    	ReadADCSensors(); // use this slow task to read ADC and write global values.

		if (connection->status==ADB_OPEN) // this is the link to Android
			adb_write(connection, 2, &values.adc0);


//		xSerialPrintf_P(PSTR("\r\nADB HighWater @ %u"), uxTaskGetStackHighWaterMark(NULL));

        }
    }


/*-----------------------------------------------------------*/




static void TaskWrite7SEG(void *pvParameters) // Write to 7 Segment display (via shift register)
{
    (void) pvParameters;

    TickType_t xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

    uint8_t i;
    uint8_t character;

//    setDigitalOutput(LATCH, LOW);				// Latch  IO_D7 // 7
//   setDigitalOutput(CLOCK, LOW);				// Clock  IO_B0 // 8
//    setDigitalOutput(DATA,  LOW);				// Data   IO_D4 // 4

    while(1)
	{
    	ReadADCSensors(); // use this slow task to read ADC and write global values.

		i = (values.adc1 >> 3); // reduce the 8 bit value to 5 bits (32 values, 7 segment) code

		character = pgm_read_byte(&ledCharSet[i]); // retrieve the character from PROGMEM; only do this once.

//			if (connection->status==ADB_OPEN) // this is the link to Android
//				adb_write(connection, 1, &xValues.adc1);

//		shiftOut(DATA,CLOCK,LATCH,MSBFIRST,~(character | 0b10000000)); // turn on decimal point
		vTaskDelayUntil( &xLastWakeTime, ( 40 / portTICK_PERIOD_MS ) );
//		shiftOut(DATA,CLOCK,LATCH,MSBFIRST,~(character & 0b01111111)); // turn off decimal point

//		xSerialPrintf_P(PSTR("A0: %3u, A1: %3u, A2: %3u, Photo: %3u \r"), values.adc0, values.adc1, values.adc2, values.adc3);
//		xSerialPrintf_P(PSTR("Write7Seg HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));

		vTaskDelayUntil( &xLastWakeTime, ( 160 / portTICK_PERIOD_MS ) );
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
		if( xSemaphoreTake( xADCSemaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
		// We were able to obtain the semaphore and can now access the
		// shared resource.
		// We want to have the ADC for us alone, as it takes some time to sample,
		// so we don't want it getting stolen during the middle of a conversion.

			setAnalogMode(MODE_10_BIT);    // 10-bit analogue-to-digital conversions

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

#if 0
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

#endif

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    char *pcTaskName )
{

	DDRB  |= _BV(DDB7);
	PORTB |= _BV(PORTB7);       // main (red PB7) LED on. Mega main LED on and die.
	while(1);
}

/*-----------------------------------------------------------*/

