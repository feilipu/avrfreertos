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

/* serial interface include file. */
#include "serial.h"

/* system time include file. */
#include "time.h"

#include "spi.h"

#ifdef portHD44780_LCD
/* LCD (Freetronics 16x2) interface include file. */
#include "hd44780.h"
#endif

/*-----------------------------------------------------------*/
/* Create a handle for the serial port. */
extern xComPortHandle xSerialPort;

static void TaskBlinkRedLED(void *pvParameters);	// Main Arduino Mega 2560, Freetronics EtherMega (Red) LED Blink

extern void TaskKermit(void *pvParameters);   		// Manage Kermit Server
/*-----------------------------------------------------------*/

/* Main program loop */
int main(void) __attribute__((OS_main));

int main(void)
{
	// turn on the serial port for debugging or for other USART reasons.
	xSerialPort = xSerialPortInitMinimal( USART0, 115200, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)

	//	avrSerialPrint... doesn't need the scheduler running, so we can see freeRTOS initiation issues
	avrSerialPrint_P(PSTR("\r\n\nHello World!\r\n")); // Ok, so we're alive...

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
		TaskKermit
		,  (const char *) "Kermit"
		,  256  // This stack size can be checked & adjusted by reading Highwater
		,  NULL
		,  1
		,  NULL ); // */


	avrSerialPrintf_P(PSTR("\r\nFree Heap Size: %u\r\n"),xPortGetMinimumEverFreeHeapSize() ); // needs heap_1.c, heap_2.c or heap_4.c

    vTaskStartScheduler(); // start the task scheduler, which takes over control of individual tasks. Should never return to here.

    avrSerialPrint_P(PSTR("\r\n\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...

#ifdef portHD44780_LCD
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

	DDRD |= _BV(DDD5);

    while(1)
    {
    	PORTD |=  _BV(PORTD5);       // DangerShield (red IO_D5) LED on.
		vTaskDelayUntil( &xLastWakeTime, ( 128 / portTICK_PERIOD_MS ) );
//		vTaskDelay(0);

#ifdef portHD44780_LCD
		lcd_Locate (0, 0);
		lcd_Printf_P(PSTR("Sys Tick:%7lu"), time(NULL));
		lcd_Locate (1, 0);
		lcd_Printf_P(PSTR("Min Heap:%7u"), xPortGetMinimumEverFreeHeapSize() ); // needs heap_4 for this function to succeed.
#endif // portHD44780_LCD

		PORTD &= ~_BV(PORTD5);       // DangerShield (red IO_D5) LED off.

		vTaskDelayUntil( &xLastWakeTime, ( 128 / portTICK_PERIOD_MS ) );
//		vTaskDelay(0);
//		xSerialPrintf_P(PSTR("RedLED HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
    }
}

/*--------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    char *pcTaskName )
{

	DDRB  |= _BV(DDB7);
	PORTB |= _BV(PORTB7);       // main (red PB7) LED on. Mega main LED on and die.
	while(1);
}

/*-----------------------------------------------------------*/
