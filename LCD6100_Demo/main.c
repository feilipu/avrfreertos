////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////    main.c
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////


#include <stdlib.h>
#include <string.h>
#include <stdio.h>
//#include <ctype.h>

#include <avr/io.h>

/* define CPU frequency in Hz here if not defined in Makefile */
#ifndef F_CPU
#define F_CPU configCPU_CLOCK_HZ
#endif

/* Scheduler include files. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

/* digital and analog functions include file. */
#include <digitalAnalog.h>

/* SPI Interface include file. */
#include <SPI9master.h>

/* serial interface include file. */
#include <lib_serial.h>

/* Nokia 6100 LCDInterface include file. */
#include <LCD_driver.h>

/*-----------------------------------------------------------*/

static void TaskBlinkGreenLED(void *pvParameters); // Main Arduino (Green) LED Blink

static void TaskWriteSPILCD(void *pvParameters);   // Write SPI LCD

/* Create a Semaphore mutex flag for the SPI Bus. To ensure only single access. */
xSemaphoreHandle xSPISemaphore;

/* Create a Semaphore mutex flag for the LCD. To ensure only single access. */
xSemaphoreHandle xLCDSemaphore;

/* Create a handle for the serial port. */
// xComPortHandle xSerialPort; // To enable serial port

/*-----------------------------------------------------------*/


/* Main program loop */
int16_t main(void) __attribute__((OS_main));

int16_t main(void)
    {

    xSPISemaphore = xSemaphoreCreateMutex(); // mutex semaphore for SPI bus
    xLCDSemaphore = xSemaphoreCreateMutex(); // mutex semaphore for LCD

//	To enable serial port for debugging or other purposes.
//	xSerialPort = xSerialPortInitMinimal( 115200, 32, 8); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)

//	avrSerialPrint... doesn't need the scheduler running, so we can see freeRTOS initiation issues
//  avrSerialPrint_P(PSTR("\r\n\n\nHello World!\r\n")); // Ok, so we're alive...

    xTaskCreate(
		TaskBlinkGreenLED
		,  (const signed portCHAR *)"Green LED"
		,  168  // 23 bytes free. This stack size can be checked & adjusted by reading Highwater
		,  NULL
		,  3
		,  NULL );


    xTaskCreate(
    		TaskWriteSPILCD
		,  (const signed portCHAR *)"Write LCD"
		,  188  // 39 bytes free. This stack size can be checked & adjusted by reading Highwater
		,  NULL
		,  1
		,  NULL );


//	avrSerialPrintf_P(PSTR("\r\n\nFree Heap Size: %u"),xPortGetFreeHeapSize() ); // needs heap_1.c or heap_2.c

    vTaskStartScheduler();

//    avrSerialPrint_P(PSTR("\r\n\n\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...

    }



/*-----------------------------------------------------------*/


    static void TaskBlinkGreenLED(void *pvParameters) // Main Arduino (Green) LED Flash
    {
    (void) pvParameters;;
    portTickType xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();


    while(1)
        {
        setDigitalOutput( IO_B5, LOW);               // main green LED off
		vTaskDelayUntil( &xLastWakeTime, ( 500 / portTICK_RATE_MS ) ); // blink at 1s cycle.

        setDigitalOutput( IO_B5, HIGH);               // main green LED on
		vTaskDelayUntil( &xLastWakeTime, ( 500 / portTICK_RATE_MS ) );

//		xLCDPrintf_P( 16, 4, BOLD, WHITE, GREEN, PSTR("LED HWM: %u"), uxTaskGetStackHighWaterMark(NULL));
//		xSerialPrintf_P(PSTR("GreenLED HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
        }
    }



    static void TaskWriteSPILCD(void *pvParameters) // Write to SPI LCD
    {
    (void) pvParameters;;

    portTickType xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

    uint8_t	s1, s2, s3 = 0;

  	//*	setup the switches for input
	//*	set the pull up resisters
	setDigitalInput(kSwitch1_PIN, HIGH);
	setDigitalInput(kSwitch2_PIN, HIGH);
	setDigitalInput(kSwitch3_PIN, HIGH);

	xLCDInit();	            //Initialise the LCD
    xLCDContrast(44);

    xLCDClear(WHITE);    // Clear LCD to a solid color
    xLCDPrint_P(  0, 4, BOLD,   WHITE, RED, PSTR("Click a button!") ); // Write instructions on display
    xLCDPrint_P( 32, 4, BOLD, BLACK, YELLOW, PSTR("This library") );
    xLCDPrint_P( 48, 4, BOLD, BLACK, YELLOW, PSTR("is for world") );
    xLCDPrint_P( 64, 4, BOLD, BLACK, YELLOW, PSTR("domination!") );
    xLCDPrint_P( 80, 4, BOLD, BLACK, YELLOW, PSTR("and other") );
    xLCDPrint_P( 96, 4, BOLD, BLACK, YELLOW, PSTR("fun stuff!") );


    while(1)
		{
			if( xLCDSemaphore != NULL )
			{
				// See if we can obtain the semaphore.  If the semaphore is not available
				// wait 10 ticks to see if it becomes free.
				if( xSemaphoreTake( xLCDSemaphore, ( portTickType ) 10 ) == pdTRUE )
				{
					// We were able to obtain the semaphore and can now access the
					// shared resource.
					if( xSPISemaphore != NULL )
					{
						// See if we can obtain the semaphore.  If the semaphore is not available
						// wait 10 ticks to see if it becomes free.
						if(  xSemaphoreTake( xSPISemaphore, ( portTickType ) 10 ) == pdTRUE  )
						{

								s1	=	isDigitalInputHigh(kSwitch1_PIN);
								s2	=	isDigitalInputHigh(kSwitch2_PIN);
								s3	=	isDigitalInputHigh(kSwitch3_PIN);


								if (!s1)
								{
									xLCDClear(WHITE);    // Clear LCD to WHITE
									xLCDPrint_P( 0, 4, BOLD, ORANGE, WHITE, PSTR("Lines!")); // Write information on display

									xLCDPrint_P( 78, 4, BOLD,   BLUE, WHITE, PSTR("LCDDrawLine")); // Write code information on display
									xLCDPrint_P( 94, 4, NORMAL, BLUE, WHITE, PSTR("(x0, y0, x1, y1,"));
									xLCDPrint_P( 110, 4, NORMAL, BLUE, WHITE, PSTR("colour)"));

									xLCDDrawLine(45, 5, 21, 130, BLACK); // Write a bunch of lines
									xLCDDrawLine(46, 5, 22, 130, RED);
									xLCDDrawLine(47, 5, 23, 130, GREEN);
									xLCDDrawLine(48, 5, 24, 130, BLUE);
									xLCDDrawLine(49, 5, 25, 130, CYAN);
									xLCDDrawLine(50, 5, 26, 130, MAGENTA);
									xLCDDrawLine(51, 5, 27, 130, YELLOW);
									xLCDDrawLine(52, 5, 28, 130, BROWN);
									xLCDDrawLine(53, 5, 29, 130, ORANGE);
									xLCDDrawLine(54, 5, 30, 130, PINK);

									xLCDDrawLine(50, 50, 130, 130, PINK);
									xLCDDrawLine(80, 50, 50, 130, BROWN);
									xLCDDrawLine(110, 12, 30, 80, BROWN);

								}

								else if (!s2)
								{ 	//  LCDDrawRect(x0, y0, x1, y1, fill, color);
									xLCDClear(WHITE);    // Clear LCD to WHITE
									xLCDPrint_P( 0, 4, BOLD, ORANGE, WHITE, PSTR("Rectangles!")); // Write information on display

									xLCDPrint_P( 78, 4, BOLD,   BLUE, WHITE, PSTR("LCDDrawRect")); // Write information on display
									xLCDPrint_P( 94, 4, NORMAL, BLUE, WHITE, PSTR("(x0, y0, x1, y1,")); // Write information on display
									xLCDPrint_P(110, 4, NORMAL, BLUE, WHITE, PSTR("fill, colour)")); // Write information on display									xLCDPrint_P(  0, 4, BOLD, ORANGE, WHITE, PSTR("Rectangles!")); // Write information on display

									xLCDDrawRect(20, 20, 50, 60, 1, YELLOW); // filled rectangle
									xLCDDrawRect(20, 20, 50, 60, 0, MAGENTA);// Line around filled rectangle
									xLCDDrawRect(60, 40, 80, 60, 0, GREEN);// Unfilled rectangle
									xLCDDrawRect(60, 70, 80, 120, 0, BLUE);// Unfilled rectangle number 2
									xLCDDrawRect(20, 70, 50, 120, 0, BLUE);// Unfilled rectangle number 3
								}

								else if (!s3)
								{
									xLCDClear(WHITE);    // Clear LCD to WHITE
									xLCDPrint_P( 0, 4, BOLD, ORANGE, WHITE, PSTR("Circles!")); // Write information on display

									xLCDPrint_P( 78, 4, BOLD, BLUE, WHITE, PSTR("LCDDrawCircle"));
									xLCDPrint_P( 94, 4, NORMAL, BLUE, WHITE, PSTR("(x, y, radius,"));
									xLCDPrint_P( 110, 4, NORMAL, BLUE, WHITE,PSTR("sector, colour)"));

									xLCDDrawCircle (55, 55, 30, FULLCIRCLE, RED); // draw a circle
									xLCDDrawCircle (70, 70, 20, FULLCIRCLE, BLUE); // draw a circle
									xLCDDrawCircle (85, 85, 10, FULLCIRCLE, GREEN); // draw a circle

								}

							xSemaphoreGive( xSPISemaphore );
						}
					}
					xLCDPrintf_P( 12, 4, NORMAL, WHITE, RED, PSTR("Tick#: %u"), xTaskGetTickCount() );
					xSemaphoreGive( xLCDSemaphore );
				}
			}
//			xSerialPrintf_P(PSTR("LCD HighWater @ %u Executed in %u ticks\r\n"), uxTaskGetStackHighWaterMark(NULL), xTaskGetTickCount()-xLastWakeTime);

			vTaskDelayUntil( &xLastWakeTime, ( 100 / portTICK_RATE_MS ) ); // yield, and wait for 100ms exactly
//			taskYIELD(); // give other tasks a chance to run, before resuming at next tick.

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

