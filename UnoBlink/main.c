////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////    main.c
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <avr/io.h>

/* Scheduler include files. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

/* serial interface include file. */
#include <lib_serial.h>

/* LCD (Freetronics 16x2) interface include file. */
//#include <hd44780.h>


/*-----------------------------------------------------------*/
/* Create a handle for the serial port. */
xComPortHandle xSerialPort;

static void TaskBlinkRedLED(void *pvParameters); // Main Arduino Mega 2560, Freetronics EtherMega (Red) LED Blink

static void TaskBlinkGreenLED(void *pvParameters); // Main Arduino Uno 328p (Green) LED Blink
/*-----------------------------------------------------------*/

/* Main program loop */
int16_t main(void) __attribute__((OS_main));

int16_t main(void)
{

    // turn on the serial port for debugging or for other USART reasons.
	xSerialPort = xSerialPortInitMinimal( 115200, portSERIAL_BUFFER, portSERIAL_BUFFER); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)

	avrSerialPrint_P(PSTR("\r\n\n\nHello World!\r\n")); // Ok, so we're alive...

    xTaskCreate(
		TaskBlinkRedLED
		,  (const signed portCHAR *)"RedLED" // Main Arduino Mega 2560, Freetronics EtherMega (Red) LED Blink
		,  256				// Tested 9 free @ 208
		,  NULL
		,  3
		,  NULL ); // */

    xTaskCreate(
		TaskBlinkGreenLED
		,  (const signed portCHAR *)"GreenLED" // Main Arduino Uno 328p (Green) LED Blink
		,  256				// Tested 9 free @ 208
		,  NULL
		,  3
		,  NULL ); // */


	avrSerialPrintf_P(PSTR("\r\n\nFree Heap Size: %u\r\n"),xPortGetFreeHeapSize() ); // needs heap_1 or heap_2 for this function to succeed.

	vTaskStartScheduler();

	avrSerialPrint_P(PSTR("\r\n\n\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...

}

/*-----------------------------------------------------------*/


static void TaskBlinkRedLED(void *pvParameters) // Main Red LED Flash
{
    (void) pvParameters;;
    portTickType xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

//	int8_t i;
//	uint8_t j;

//	lcd_Init();

	DDRB |= _BV(DDB7);

    while(1)
    {

    	PORTB |=  _BV(PORTB1);       // main (red IO_B7) LED on. EtherMega LED on
		vTaskDelayUntil( &xLastWakeTime, ( 10 / portTICK_RATE_MS ) );

//		lcd_Locate (0, 0);
//		lcd_Printf_P(PSTR("HighWater @ %u"), uxTaskGetStackHighWaterMark(NULL));
//		lcd_Cursor (1);

#if _USE_FUEL
		lcd_Locate (1, 0);
		lcd_PutFuel (--i, 0);
		if (i < 0) i = 6;
#endif

#if _USE_BAR
		lcd_Locate (1, 2);
		lcd_PutBar (j++, 14, 2);
#endif

		PORTB &= ~_BV(PORTB1);       // main (red IO_B7) LED off. EtherMega LED off
		vTaskDelayUntil( &xLastWakeTime, ( 40 / portTICK_RATE_MS ) );

//		xSerialPrintf_P(PSTR("RedLED HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
    }

}

/*-----------------------------------------------------------*/
static void TaskBlinkGreenLED(void *pvParameters) // Main Green LED Flash
{
    (void) pvParameters;;
    portTickType xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

	DDRB |= _BV(DDB5);

    while(1)
    {
    	PORTB |=  _BV(PORTB5);       // main (red PB5) LED on. Arduino LED on
		vTaskDelayUntil( &xLastWakeTime, ( 100 / portTICK_RATE_MS ) );

		PORTB &= ~_BV(PORTB5);       // main (red PB5) LED off. Arduino LED off
		vTaskDelayUntil( &xLastWakeTime, ( 400 / portTICK_RATE_MS ) );

		xSerialPrintf_P(PSTR("GreenLED HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
    }
}

/*-----------------------------------------------------------*/


void vApplicationStackOverflowHook( xTaskHandle xTask,
                                    signed portCHAR *pcTaskName )
{
	DDRB  |= _BV(DDB5);
	PORTB |= _BV(PORTB5);       // main (red PB5) LED on. Arduino LED on and die.
	while(1);
}

/*-----------------------------------------------------------*/
