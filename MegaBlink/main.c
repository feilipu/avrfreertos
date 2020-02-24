////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////    main.c
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>


#include <avr/io.h>
#include <avr/sleep.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* serial interface include file. */
#include "serial.h"


/* system time include file. */
#include "time.h"

#ifdef portHD44780_LCD
/* LCD (Freetronics 16x2) interface include file. */
#include "hd44780.h"
#endif

/*-----------------------------------------------------------*/
/* Create a handle for the serial port. */
extern xComPortHandle xSerialPort;

static void TaskBlinkRedLED(void *pvParameters); // Main Arduino Mega 2560, Freetronics EtherMega (Red) LED Blink

static void TaskBlinkGreenLED(void *pvParameters); // Main Arduino Uno 328p (Green) LED Blink
/*-----------------------------------------------------------*/

/* Main program loop */
int main(void) __attribute__((OS_main));

int main(void)
{

    // turn on the serial port for debugging or for other USART reasons.
	xSerialPort = xSerialPortInitMinimal( USART0, 38400, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)

	avrSerialxPrint_P( &xSerialPort, PSTR("\r\n\n\nHello World!\r\n")); // Ok, so we're alive...

#ifdef	portHD44780_LCD
	lcd_Init();

	lcd_Print_P(PSTR("Hello World!"));
	lcd_Locate (0, 0);
#endif

    xTaskCreate(
		TaskBlinkRedLED
		,  (const char *)"RedLED" // Main Arduino Mega 2560, Freetronics EtherMega (Red) LED Blink
		,  256				// Tested 9 free @ 208
		,  NULL
		,  3
		,  NULL ); // */

    xTaskCreate(
		TaskBlinkGreenLED
		,  (const char *)"GreenLED" // Main Arduino Uno 328p (Green) LED Blink
		,  256				// Tested 9 free @ 208
		,  NULL
		,  3
		,  NULL ); // */


//	avrSerialPrintf_P(PSTR("\r\n\nFree Heap Size: %u\r\n"),xPortGetFreeHeapSize() ); // needs heap_1 or heap_2 for this function to succeed.

	vTaskStartScheduler();

	avrSerialxPrint_P( &xSerialPort, PSTR("\r\n\n\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...

#ifdef portHD44780_LCD
	lcd_Print_P(PSTR("DEAD BEEF!"));
#endif
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

	DDRB |= _BV(DDB7);

    while(1)
    {
    	PORTB |=  _BV(PORTB7);       // main (red IO_B7) LED on. EtherMega LED on
		vTaskDelayUntil( &xLastWakeTime, ( 100 / portTICK_PERIOD_MS ) );

		PORTB &= ~_BV(PORTB7);       // main (red IO_B7) LED off. EtherMega LED off
		vTaskDelayUntil( &xLastWakeTime, ( 400 / portTICK_PERIOD_MS ) );

//		if ( _SLEEP_CONTROL_REG & (_BV(SE) ) ); //| (_BV(SM0) | _BV(SM1) | _BV(SM2))) )
//			xSerialPortReInit( &xSerialPort );

		xSerialxPrintf_P( &xSerialPort, PSTR("RedLED HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
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

    while(1)
    {
    	PORTB |=  _BV(PORTB5);       // main (red PB5) LED on. Arduino LED on

		vTaskDelayUntil( &xLastWakeTime, ( 100 / portTICK_PERIOD_MS ) );

#ifdef portHD44780_LCD
		lcd_Locate (0, 0);
		lcd_Printf_P(PSTR("Sys Tick:%7lu"), time(NULL));
		lcd_Locate (1, 0);
		lcd_Printf_P(PSTR("Min Heap:%7u"), xPortGetMinimumEverFreeHeapSize() ); // needs heap_4 for this function to succeed.
#endif // portHD44780_LCD

		PORTB &= ~_BV(PORTB5);       // main (red PB5) LED off. Arduino LED off

		vTaskDelayUntil( &xLastWakeTime, ( 400 / portTICK_PERIOD_MS ) );

//		if ( _SLEEP_CONTROL_REG & (_BV(SE) ) ); // | (_BV(SM0) | _BV(SM1) | _BV(SM2))) )
//			xSerialPortReInit( &xSerialPort );
		xSerialxPrintf_P( &xSerialPort, PSTR("GreenLED HighWater @ %u\n    system_tick:%7lu \r\n"), uxTaskGetStackHighWaterMark(NULL), time(NULL));
    }
}
