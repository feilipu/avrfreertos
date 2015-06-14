////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////    main.c
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <avr/io.h>
#include <avr/interrupt.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* this is from avr/time.h, but it is not released into the main stream currently */
#include "time.h"

/* serial interface include file. */
#include "serial.h"

/* LCD (Freetronics 16x2) interface include file. */
#include "hd44780.h"

/*-----------------------------------------------------------*/
/* Optionally, create reference to the handle for the serial port. */
extern xComPortHandle xSerialPort;

static void TaskBlinkRedLED(void *pvParameters); // Main Arduino Mega 2560, Freetronics EtherMega (Red) LED Blink

static void TaskBlinkGreenLED(void *pvParameters); // Main Arduino Uno 328p (Green) LED Blink
/*-----------------------------------------------------------*/

/* Main program loop */
int main(void) __attribute__ ((OS_main));

int main(void)
{
    // turn on the serial port for debugging or for other USART reasons.
	xSerialPort = xSerialPortInitMinimal( USART0, 115200, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)

	avrSerialxPrint_P(&xSerialPort, PSTR("\r\n\n\nHello World!\r\n")); // Ok, so we're alive...

    xTaskCreate(
		TaskBlinkRedLED
		,  (const portCHAR *)"RedLED" // Main Arduino Mega 2560, Freetronics EtherMega (Red) LED Blink
		,  256				// Tested 9 free @ 208
		,  NULL
		,  3
		,  NULL ); // */

    xTaskCreate(
		TaskBlinkGreenLED
		,  (const portCHAR *)"GreenLED" // Main Arduino Uno 328p (Green) LED Blink
		,  256				// Tested 9 free @ 208
		,  NULL
		,  3
		,  NULL ); // */


	avrSerialxPrintf_P(&xSerialPort, PSTR("Free Heap Size: %u\r\n"), xPortGetFreeHeapSize() ); // needs heap_1,  heap_2 or heap_4 for this function to succeed.
	avrSerialxPrintf_P(&xSerialPort, PSTR("Minimum Free Heap Size: %u\r\n"), xPortGetMinimumEverFreeHeapSize() ); // needs heap_4 for this function to succeed.
	vTaskStartScheduler();

	avrSerialxPrint_P(&xSerialPort, PSTR("\r\n\n\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...

}

/*-----------------------------------------------------------*/


static void TaskBlinkRedLED(void *pvParameters) // Main Red LED Flash
{
    (void) pvParameters;
    TickType_t xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

//	int8_t i  = 6;
//	uint8_t j = 0;

#ifdef portHD44780_LCD
	lcd_Init();
#endif

	DDRB |= _BV(DDB7);

    for(;;)
    {

    	PORTB |=  _BV(PORTB7);       // main (red IO_B7) LED on. EtherMega LED on
		vTaskDelayUntil( &xLastWakeTime, ( 500 / portTICK_PERIOD_MS ) );

#ifdef portHD44780_LCD
		lcd_Locate (0, 0);
		lcd_Printf_P(PSTR("Sys Tick:%7lu"), time(NULL));

		lcd_Locate (1, 0);
		lcd_Printf_P(PSTR("Min Heap:%7u"), xPortGetMinimumEverFreeHeapSize() ); // needs heap_4 for this function to succeed.

#if _USE_CURSOR
		lcd_Cursor (1);
#endif

#if _USE_FUEL
		lcd_Locate (1, 0);
		lcd_PutFuel (--i, 0);
		if (i < 0) i = 6;
#endif

#if _USE_BAR
		lcd_Locate (1, 2);
		lcd_PutBar (j++, 14, 2);
		if (j > _MAX_BAR) j = 0;
#endif

#endif

		PORTB &= ~_BV(PORTB7);       // main (red IO_B7) LED off. EtherMega LED off
		vTaskDelayUntil( &xLastWakeTime, ( 500 / portTICK_PERIOD_MS ) );

//		xSerialxPrintf_P(&xSerialPort, PSTR("RedLED HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
    }

}

/*-----------------------------------------------------------*/
static void TaskBlinkGreenLED(void *pvParameters) // Main Green LED Flash
{
    (void) pvParameters;
    TickType_t xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

	DDRB |= _BV(DDB5);

    for(;;)
    {
    	PORTB |=  _BV(PORTB5);       // main (red PB5) LED on. Arduino LED on
		vTaskDelayUntil( &xLastWakeTime, ( 500 / portTICK_PERIOD_MS ) );

		PORTB &= ~_BV(PORTB5);       // main (red PB5) LED off. Arduino LED off
		vTaskDelayUntil( &xLastWakeTime, ( 500 / portTICK_PERIOD_MS )  );

		xSerialxPrintf_P(&xSerialPort, PSTR("Current Timestamp: %lu xTaskGetTickCount(): %u\r\n"), time(NULL), xTaskGetTickCount());
		xSerialxPrintf_P(&xSerialPort, PSTR("Minimum Free Heap Size: %u\r\n"), xPortGetMinimumEverFreeHeapSize() ); // needs heap_4 for this function to succeed.
    }
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    portCHAR *pcTaskName )
{
	DDRB  |= _BV(DDB5);
	PORTB |= _BV(PORTB5);       // main (red PB5) LED on. Arduino LED on and die.
	while(1);
}

/*--------------------------------------------*/

