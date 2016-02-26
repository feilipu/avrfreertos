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
#include <avr/eeprom.h>
#include <util/delay.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Pololu derived include files. */
#include "digitalAnalog.h"

/* serial interface include file. */
#include "serial.h"

/* i2c Interface include file. */
#include "i2cMultiMaster.h"

/* extended string to integer */
#include "xatoi.h"

/* RTC interface (using I2C) include file */
#include "rtc.h"

/* hd44780 LCD control interface file. */
#include "hd44780.h"

/* Clock include file. */
#include "electricimp.h"


/*--------------PrivateFunctions-------------------*/

static void get_line (uint8_t *buff, uint8_t len);

static portFLOAT ReadADCSensors(void);   // Read ADC Sensor for thermal LM335z

/*--------------Functions---------------------------*/

/* Main program loop */
int16_t main(void) __attribute__((OS_main));

int16_t main(void)
{

    // turn on the serial port for setting or querying the time .
	xSerialPort = xSerialPortInitMinimal( USART0, 115200, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)

    // Memory shortages mean that we have to minimise the number of
    // threads, hence there are no longer multiple threads using a resource.
    // Still, semaphores are useful to stop a thread proceeding, where it should be stopped because it is using a resource.
    vSemaphoreCreateBinary( xADCSemaphore ); 		// binary semaphore for ADC - Don't sample temperature when hands are moving (voltage droop).

	// initialise I2C master interface, need to do this once only.
	// If there are two I2C processes, then do it during the system initiation.
	I2C_Master_Initialise((ARDUINO<<I2C_ADR_BITS) | (pdTRUE<<I2C_GEN_BIT));

	avrSerialPrint_P(PSTR("\r\n\nHello World!\r\n")); // Ok, so we're alive...

    xTaskCreate(
		TaskBlinkGreenLED
		,  (const portCHAR *)"GreenLED" // Main Arduino Uno 328p (Green) LED Blink
		,  256				// Tested 9 free @ 208
		,  NULL
		,  3
		,  NULL ); // */

    xTaskCreate(
		TaskWriteLCD
		,  (const portCHAR *)"WriteLCD"
		,  192
		,  NULL
		,  2
		,  NULL ); // */


   xTaskCreate(
		TaskMonitor
		,  (const portCHAR *)"SDMonitor" // Serial Monitor
		,  208
		,  NULL
		,  3
		,  NULL ); // */

	avrSerialPrintf_P(PSTR("\r\nFree Heap Size: %u\r\n"),xPortGetFreeHeapSize() ); // needs heap_1 or heap_2 for this function to succeed.

    vTaskStartScheduler();

	avrSerialPrint_P(PSTR("\r\n\n\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...

#if defined (portHD44780_LCD)
	lcd_Locate (0, 1);
	lcd_Print_P(PSTR("DEAD BEEF!"));
#endif


}


/*-----------------------------------------------------------*/
static void TaskBlinkGreenLED(void *pvParameters) // Main Green LED Flash
{
    (void) pvParameters;;
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

		PORTB &= ~_BV(PORTB5);       // main (red PB5) LED off. Arduino LED off
		vTaskDelayUntil( &xLastWakeTime, ( 400 / portTICK_PERIOD_MS ) );

		xSerialPrintf_P(PSTR("GreenLED HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
    }
}

/*-----------------------------------------------------------*/

static void TaskMonitor(void *pvParameters) // Monitor for Serial Interface
{
    (void) pvParameters;;

	uint8_t *ptr;
	int32_t p1;

	// create the buffer on the heap (so they can be moved later).
	if(LineBuffer == NULL) // if there is no Line buffer allocated (pointer is NULL), then allocate buffer.
		if( !(LineBuffer = (uint8_t *) pvPortMalloc( sizeof(uint8_t) * LINE_SIZE )))
			xSerialPrint_P(PSTR("pvPortMalloc for *LineBuffer fail..!\r\n"));

#if defined (portRTC_DEFINED)

	if(SetTimeDate == NULL) // if there is no SetTimeDate allocated (pointer is NULL), then allocate buffer.
		if( !(SetTimeDate = (pRTCArraySto) pvPortMalloc( sizeof(xRTCArraySto) )))
			xSerialPrint_P(PSTR("pvPortMalloc for *Buff fail..!\r\n"));
#endif


    while(1)
    {

    	xSerialPutChar(&xSerialPort, '>');

		ptr = LineBuffer;
		get_line(ptr, (uint8_t)(sizeof(uint8_t)* LINE_SIZE)); //sizeof (Line);

		switch (*ptr++) {

#ifdef portRTC_DEFINED
		case 't' :	/* t [<year yy> <month mm> <date dd> <day: Sun=1> <hour hh> <minute mm> <second ss>] */

			if (xatoi(&ptr, &p1)) {
				SetTimeDate->Year = (uint8_t)p1;
				xatoi(&ptr, &p1); SetTimeDate->Month = (uint8_t)p1;
				xatoi(&ptr, &p1); SetTimeDate->Date = (uint8_t)p1;
				xatoi(&ptr, &p1); SetTimeDate->Day = (uint8_t)p1;
				xatoi(&ptr, &p1); SetTimeDate->Hour = (uint8_t)p1;
				xatoi(&ptr, &p1); SetTimeDate->Minute = (uint8_t)p1;
				if (!xatoi(&ptr, &p1))
					break;
				SetTimeDate->Second = (uint8_t)p1;

				xSerialPrintf_P(PSTR("Set: %u/%u/%u %2u:%02u:%02u\r\n"), SetTimeDate->Year, SetTimeDate->Month, SetTimeDate->Date, SetTimeDate->Hour, SetTimeDate->Minute, SetTimeDate->Second);
				if (setDateTimeDS1307( SetTimeDate ) == pdTRUE)
					xSerialPrint_P( PSTR("Setting successful\r\n") );

			} else {

				if (getDateTimeDS1307( &xCurrentTempTime.DateTime) == pdTRUE)
					xSerialPrintf_P(PSTR("Current: %u/%u/%u %2u:%02u:%02u\r\n"), xCurrentTempTime.DateTime.Year + 2000, xCurrentTempTime.DateTime.Month, xCurrentTempTime.DateTime.Date, xCurrentTempTime.DateTime.Hour, xCurrentTempTime.DateTime.Minute, xCurrentTempTime.DateTime.Second);
			}

			break;
#endif

		case 'r' : // reset
			switch (*ptr++) {
			case 't' : // temperature

				xMaximumTempTime = xCurrentTempTime;
				xMinimumTempTime = xCurrentTempTime;
				// Now we commit the time and temperature to the EEPROM, forever...
				eeprom_update_block(&xMaximumTempTime, &xMaximumEverTempTime, sizeof(xRTCTempArray));
				eeprom_update_block(&xMinimumTempTime, &xMinimumEverTempTime, sizeof(xRTCTempArray));

				break;
			default :
				break;
			}
			break;

		default :
			break;

		}
// 		xSerialPrintf_P(PSTR("\r\nSD Monitor HighWater @ %u\r\n\n"), uxTaskGetStackHighWaterMark(NULL));
    }

}


/*-----------------------------------------------------------*/


static void TaskWriteLCD(void *pvParameters) // Write to LCD
{
    (void) pvParameters;

    TickType_t xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

	uint8_t temperature_print; // true if temperature can be displayed.

    eeprom_read_block(&xMaximumTempTime, &xMaximumEverTempTime, sizeof(xRTCTempArray));
    eeprom_read_block(&xMinimumTempTime, &xMinimumEverTempTime, sizeof(xRTCTempArray));

    lcd_Init();	// initialise LCD, move cursor to start of top line

    while(1)
    {
    	if(getDateTimeDS1307(&xCurrentTempTime.DateTime))
    	{
			if (0) // ( (xCurrentTempTime.Temperature = ReadADCSensors())) // if non 0 then a reading returned.
			{	// trigger a temperature reading

				temperature_print = true;
				xCurrentTempTime.Temperature -= 273.15; // Convert from Kelvin to Celcius

				if( (xCurrentTempTime.Temperature < 65) && (xCurrentTempTime.Temperature > xMaximumTempTime.Temperature)) // check for maximum temp
				// we don't expect the temperature sensor to work above 65C
				{
					xMaximumTempTime = xCurrentTempTime;

					// Now we commit the time and temperature to the EEPROM, forever...
					eeprom_update_block(&xMaximumTempTime, &xMaximumEverTempTime, sizeof(xRTCTempArray));
				}

				if( (xCurrentTempTime.Temperature > (-30)) && (xCurrentTempTime.Temperature < xMinimumTempTime.Temperature)) // and check for minimum temp
				// we don't expect the temperature sensor to work below -30C
				{
					xMinimumTempTime = xCurrentTempTime;

					// Now we commit the time and temperature to the EEPROM, forever...
					eeprom_update_block(&xMinimumTempTime, &xMinimumEverTempTime, sizeof(xRTCTempArray));
				}
			}
			else temperature_print = false;

			lcd_Locate(0, 0);  // go to the first character of the first LCD line
			switch( xCurrentTempTime.DateTime.Day )
			{
				case Sunday:
					lcd_Print_P(PSTR("Sunday   "));
					break;
				case Monday:
					lcd_Print_P(PSTR("Monday   "));
					break;
				case Tuesday:
					lcd_Print_P(PSTR("Tuesday  "));
					break;
				case Wednesday:
					lcd_Print_P(PSTR("Wednesday"));
					break;
				case Thursday:
					lcd_Print_P(PSTR("Thursday "));
					break;
				case Friday:
					lcd_Print_P(PSTR("Friday   "));
					break;
				case Saturday:
					lcd_Print_P(PSTR("Saturday "));
					break;
				default:
					lcd_Print_P(PSTR("NotMyDay "));
					break;
			}

			// display Day Date/Month/Year
			lcd_Locate(0, 10);              // go to the eleventh character of the first LCD line
			lcd_Printf_P( PSTR("%2u/%2u/%4u"), xCurrentTempTime.DateTime.Date, xCurrentTempTime.DateTime.Month, (xCurrentTempTime.DateTime.Year + 2000) );

			// display the current temperature
			lcd_Locate(1, 1);			// LCD cursor to third character of the second LCD line
			if ( temperature_print )	// print the temperature if you got it
				lcd_Printf_P( PSTR("%6.2f"), xCurrentTempTime.Temperature); // print Celcius temperature

			// display the current time
			lcd_Locate(1, 9);             // go to the ninth character of the second LCD line
			lcd_Printf_P(PSTR("%2u:%02u:%02u"),xCurrentTempTime.DateTime.Hour, xCurrentTempTime.DateTime.Minute, xCurrentTempTime.DateTime.Second);

			// display the maximum temperature, time and date
			lcd_Locate(2, 0);          // go to the first character of the third LCD line
			lcd_Printf_P(PSTR("Max%5.1f"),xMaximumTempTime.Temperature);			// print the maximum temperature value

			lcd_Locate(2, 9);          // go to the ninth character of the third LCD line
			lcd_Printf_P(PSTR("%2u:%02u %2u/%2u"),xMaximumTempTime.DateTime.Hour, xMaximumTempTime.DateTime.Minute, xMaximumTempTime.DateTime.Date, xMaximumTempTime.DateTime.Month );

			// display the m temperature, time and date
			lcd_Locate(3, 0);          // go to the first character of the forth LCD line
			lcd_Printf_P(PSTR("Min%5.1f"),xMinimumTempTime.Temperature);			// print the minimum temperature value

			lcd_Locate(3, 9);          // go to the ninth character of the fourth LCD line
			lcd_Printf_P(PSTR("%2u:%02u %2u/%2u"),xMinimumTempTime.DateTime.Hour, xMinimumTempTime.DateTime.Minute, xMinimumTempTime.DateTime.Date, xMinimumTempTime.DateTime.Month );

    	}
//		xSerialPrintf_P(PSTR("LCD HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
        vTaskDelayUntil( &xLastWakeTime, ( 200 / portTICK_PERIOD_MS ) );
	}
}


/*-----------------------------------------------------------*/
/* Additional helper functions */
/*-----------------------------------------------------------*/

portFLOAT ReadADCSensors(void) // Read ADC Sensor for Thermal LM335z
{
	// Variables for the analogue conversion on ADC Sensors

    uint32_t samples = 0;               		// holds the summated samples
    uint16_t i = (uint16_t) _BV(2 * ADC_SAMPLES); // 4 ^ ADC_SAMPLES

	if( xADCSemaphore != NULL )
	{
		// See if we can obtain the semaphore.  If the semaphore is not available
		// wait 5 ticks to see if it becomes free.

		if( xSemaphoreTake( xADCSemaphore, ( TickType_t ) 5 ) == pdTRUE )
		{
			// We were able to obtain the semaphore and can now access the shared resource.
			// We want to have the ADC for us alone, as it takes some time to sample,
			// so we don't want it getting stolen during the middle of a conversion.

		    setAnalogMode(MODE_10_BIT);    // 10-bit analog-to-digital conversions

			do
			{
				startAnalogConversion(0, EXTERNAL_REF);   // start next conversion
				do _delay_loop_1(F_CPU / 5e5);     // wait until conversion read. This is about 6 uS, which should be enough.
				while( analogIsConverting() );

				samples += (uint32_t)analogConversionResult();	// sum the results

			} while (--i);

			xSemaphoreGive( xADCSemaphore );

			samples >>= ADC_SAMPLES; 		// Decimate the samples (to get better accuracy), see AVR8003.doc

			/*
			For the LM335Z we want to calculate the resistance R1 required to ensure that we have 500 uA minimum at the maximum
			temperature we intend to measure.
			Assume maximum 60C this is 273K + 60 = 333K or will be measured at 3330mV

			If Vcc is 4.9V (USB) then the R = V/I calculation gives (4.9 - 3.3)/ 0.0005 = 3200 Ohm

			This leads to using a 3200 Ohm resistor, or there about being 3300 Ohm.

			Testing gives us 0.58mA with the resistor actual at 3250 Ohm.

			Analogue testing gives with this set up: 2.952V at 20C actual... or 22C indicated

			Lets see what the Arduino ADC gives us...
			*/

			// The 497 is the Power Supply Voltage in mV / 10. _BV(10 + ADC_SAMPLES) is the number of ADC values.
			// 271.15 is the adjustment from Kelvin (273.15) and the offset relating to the Temp Sensor error correction.
		} else {
			return 0; // no sample taken so return 0.
		}
	}
	return ( (portFLOAT) samples * 497.0 / (((uint16_t)_BV(10 + ADC_SAMPLES)) -1) );  // and return the current Kelvin temp
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


/*-----------------------------------------------------------*/


void vApplicationStackOverflowHook( TaskHandle_t xTask,
									signed portCHAR *pcTaskName )
{
	/*---------------------------------------------------------------------------*\
	Usage:
	   called by task system when a stack overflow is noticed
	Description:
	   Stack overflow handler -- Shut down all interrupts, send serious complaint
	    to command port.
	Arguments:
	   pxTask - pointer to task handle
	   pcTaskName - pointer to task name
	Results:
	   <none>
	Notes:
	   This routine will never return.
	   This routine is referenced in the task.c file of FreeRTOS as an extern.
	\*---------------------------------------------------------------------------*/

	uint8_t* pC;
	uint16_t baud;

	/* shut down all interrupts */
	portDISABLE_INTERRUPTS();

	/* take over the command line buffer to generate our error message */
	pC = (uint8_t*) LineBuffer;

	strcat( (char*) pC, "\r\n" );
	strcat( (char*) pC, (char*) pcTaskName );
	strcat( (char*) pC, "\r\n" );

	pC = (uint8_t*) LineBuffer;

	/* Force the UART control register to be the way we want, just in case */

	UCSR0C = ( _BV( UCSZ01 ) | _BV( UCSZ00 ) );		// 8 data bits
	UCSR0B = _BV( TXEN0 );							// only enable transmit
	UCSR0A = 0;

	/* Calculate the baud rate register value from the equation in the
	* data sheet.  This calculation rounds to the nearest factor, which
	* means the resulting rate may be either faster or slower than the
	* desired rate (the old calculation was always faster).
	*
	* If the system clock is one of the Magic Frequencies, this
	* computation will result in the exact baud rate
	*/
	baud = ( ( ( configCPU_CLOCK_HZ / ( ( 16UL * 115200 ) / 2UL ) ) + 1UL ) / 2UL ) - 1UL;
	UBRR0 = baud;

	/* Send out the message, without interrupts.  Hard wired to USART 0 */
	while ( *pC )
	{
		while (!(UCSR0A & (1 << UDRE0)));
		UDR0 = *pC;
		pC++;
	}

	DDRB  |= _BV(DDB5);
	PORTB |= _BV(PORTB5);       // main (red PB5) LED on. Arduino LED on and die.
	while(1);
}

/*-----------------------------------------------------------*/

