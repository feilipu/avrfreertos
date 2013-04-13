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

/* Pololu include files - for LCD access */
#include <pololu/orangutan.h>

/* Scheduler include files. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

/* Pololu derived include files. */
#include <digitalAnalog.h>

/* serial interface include file. */
#include <lib_serial.h>

/* i2c Interface include file. */
#include <i2cMultiMaster.h>

/* RTC interface (using I2C) include file */
#include <rtc.h>

/* Servo PWM Timer include file */
#include <servoPWM.h>

/* Clock include file. */
#include "retrograde.h"

/*--------------Private Functions-------------------*/


static int8_t ReadADCSensors(void);   // Read ADC Sensor for thermal LM335z
static void setDateDs1307(void);      // Set the date & time initially & as needed.

/*--------------------------------------------------*/

/* Main program loop */
// This program only works using .../MemMang/heap_3.c. Somehow it uses too much SRAM, and only use of malloc seems to help.
int16_t main(void) __attribute__((OS_main));

int16_t main(void)
{

    // turn on the serial port for debugging or for other USART reasons.
	xSerialPort = xSerialPortInitMinimal( 115200, 48, 8); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)

    // Memory shortages mean that we have to minimise the number of
    // threads, hence there are no longer multiple threads using a resource.
    // Still, semaphores are useful to stop a thread proceeding, where it should be stopped.
	// vSemaphoreCreateBinary( xI2CSemaphore ) binary semaphore for I2C bus <<< FYI!!! This is now done in the I2C library.
    vSemaphoreCreateBinary( xADCSemaphore ); 		// binary semaphore for ADC - Don't sample temperature when hands are moving (voltage drop).
    vSemaphoreCreateBinary( xRetrogradeSemaphore ); // binary semaphore for Retrograde Analogue Hands - Only move hands on the minute (servo chatter).

	xI2CRTCQueue = xQueueCreate( 2, sizeof( xRTCArray) );  // queue for i2c RTC values

	avrSerialPrint_P(PSTR("\r\n\n\nHello World!\r\n")); // Ok, so we're alive...

    xTaskCreate(
		TaskBlinkGreenLED
		,  (const signed portCHAR *)"RedLED" // On Freetronics
		,  156				// Tested 9 free
		,  NULL
		,  3
		,  NULL ); // */

    xTaskCreate(
        TaskReadI2CRTC
        ,  (const signed portCHAR *)"ReadI2CRTC"
        ,  224				// 208 = Tested 34 free (needed for time setting)
        ,  NULL
        ,  2
        ,  NULL ); // */

    xTaskCreate(
		TaskWriteLCD
		,  (const signed portCHAR *)"WriteLCD"
		,  256				// 204 = Tested 22 free
		,  NULL
		,  1
		,  NULL ); // */

/*   xTaskCreate(
		TaskWriteRTCRetrograde
		,  (const signed portCHAR *)"WriteRTCRetrograde"
		,  204				// Tested 20 free
		,  NULL
		,  1
		,  NULL ); // */

	avrSerialPrintf_P(PSTR("\r\n\nFree Heap Size: %u\r\n"),xPortGetFreeHeapSize() ); // needs heap_1 or heap_2 for this function to succeed.

    vTaskStartScheduler();

	avrSerialPrint_P(PSTR("\r\n\n\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...

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

	LED_PORT_DIR |= _BV(LED_PIN); // Arduino LED to output on IO_B5

    while(1)
    {
    	LED_PORT |=  _BV(LED_PIN);       // main (green IO_C4) LED on  (Arduino IO_B5)
		vTaskDelayUntil( &xLastWakeTime, ( 200 / portTICK_RATE_MS ) );

		LED_PORT &= ~_BV(LED_PIN);       // main (green IO_C4) LED off  (Arduino IO_B5)
		vTaskDelayUntil( &xLastWakeTime, ( 200 / portTICK_RATE_MS ) );

//		xSerialPrintf_P(PSTR("GreenLED HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
    }

}
/*-----------------------------------------------------------*/

static void TaskReadI2CRTC(void *pvParameters) // Read i2c Bus for Real Time Clock DS1307
{
    (void) pvParameters;;

    portTickType xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

    I2C_Master_Initialise((ARDUINO<<I2C_ADR_BITS) | (pdTRUE<<I2C_GEN_BIT)); // init I2C master interface, need to do this once only.
    // If there are two I2C processes, then do it during the system initiation.

 	xRTCArray xReadings;     // Holds return values from the RTC DS1307

    /*  To adjust the date/time uncomment this function, and set a time in the near future in function below.
        Load the program. 6 seconds before the newly programmed time, press the reset button.
        Then comment out this line, recompile, and then reload the program.
        Don't press restart before the new code (without this function) is loaded.
    */
//	setDateDs1307();    // set the date & time. Set the correct values in function below, and it will run once.
							// then comment this out again on the and reboot.

    while(1)
    {
    	getDateTimeDS1307( &xReadings);

		if( xI2CRTCQueue != 0 )
		{
			// Send the Time structure. Don't wait if queue full.
			xQueueSendToBack( xI2CRTCQueue, &xReadings, pdFALSE );
		}

//		xSerialPrintf_P(PSTR("I2C HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
        vTaskDelayUntil( &xLastWakeTime, ( 200 / portTICK_RATE_MS ) );
    }
}

/*-----------------------------------------------------------*/


static void TaskWriteLCD(void *pvParameters) // Write to LCD
{
    (void) pvParameters;;

    xRTCTempArray xCurrentTempTime; // variable to hold the I2C Current time value peeked off the queue
    xRTCTempArray xMaximumTempTime;
    xRTCTempArray xMinimumTempTime;

    eeprom_read_block(&xMaximumTempTime, &xMaximumEverTempTime, sizeof(xRTCTempArray));
    eeprom_read_block(&xMinimumTempTime, &xMinimumEverTempTime, sizeof(xRTCTempArray));

    xMaximumTempTime.Temperature = -30; // Use these to initialise the EEPROM on first load, or to reset Temperature extremes.
    xMinimumTempTime.Temperature =  65; // Then comment out afterwards.

    clear();			// clear the LCD, move cursor to start of top line

    setAnalogMode(MODE_10_BIT);    // 10-bit analog-to-digital conversions

    while(1)
    {

		xQueueReceive( xI2CRTCQueue, &(xCurrentTempTime.DateTime), portMAX_DELAY); 	// block, until there is a time to grab.

		if ( (xCurrentTempTime.Temperature = ReadADCSensors()) != 0x7f) // if 0x7f then no reading returned.
		{							// trigger a temperature reading

			if( (xCurrentTempTime.Temperature < 65) && (xCurrentTempTime.Temperature > xMaximumTempTime.Temperature)) // check for maximum temp
			// we don't expect the temperature sensor to work above 65C
			{
				xMaximumTempTime = xCurrentTempTime;

				// Now we commit the time and temperature to the EEPROM, forever...
				eeprom_write_block(&xMaximumTempTime, &xMaximumEverTempTime, sizeof(xMaximumTempTime));
			}


			if( (xCurrentTempTime.Temperature > (-30)) && (xCurrentTempTime.Temperature < xMinimumTempTime.Temperature)) // and check for minimum temp
			// we don't expect the temperature sensor to work below -30C
			{
				xMinimumTempTime = xCurrentTempTime;

				// Now we commit the time and temperature to the EEPROM, forever...
				eeprom_write_block(&xMinimumTempTime, &xMinimumEverTempTime, sizeof(xMinimumTempTime));
			}
		}

		lcd_goto_xy(0, 0);              // go to the first character of the first LCD line
		switch( xCurrentTempTime.DateTime.Day )
		{
			case Sunday:
				print_from_program_space(PSTR("Sunday   "));
				break;
			case Monday:
				print_from_program_space(PSTR("Monday   "));
				break;
			case Tuesday:
				print_from_program_space(PSTR("Tuesday  "));
				break;
			case Wednesday:
				print_from_program_space(PSTR("Wednesday"));
				break;
			case Thursday:
				print_from_program_space(PSTR("Thursday "));
				break;
			case Friday:
				print_from_program_space(PSTR("Friday   "));
				break;
			case Saturday:
				print_from_program_space(PSTR("Saturday "));
				break;
		}

		// display Day Date/Month/Year
		lcd_goto_xy(10, 0);              // go to the eleventh character of the first LCD line
		print_long( xCurrentTempTime.DateTime.Date );
		print_from_program_space(PSTR("/"));
		print_long( xCurrentTempTime.DateTime.Month );
		print_from_program_space(PSTR("/20"));
		print_long( xCurrentTempTime.DateTime.Year );
		if((xCurrentTempTime.DateTime.Date < 10) && (xCurrentTempTime.DateTime.Month < 10)) print_from_program_space(PSTR("  "));// added spaces are to overwrite left over chars
		else if((xCurrentTempTime.DateTime.Date < 10) || (xCurrentTempTime.DateTime.Month < 10)) print_from_program_space(PSTR(" "));// added spaces are to overwrite left over chars

		// display the current temperature
		lcd_goto_xy(2, 1);		// LCD cursor to third character of the second LCD line
		print_long( xCurrentTempTime.Temperature ); // print temperature
		print_from_program_space(PSTR("C  "));			// added spaces are to overwrite left over chars

		// display the current time
		lcd_goto_xy(8, 1);             // go to the ninth character of the second LCD line
		if(xCurrentTempTime.DateTime.Hour < 10) print_from_program_space(PSTR(" "));
		print_long( xCurrentTempTime.DateTime.Hour);
		if(xCurrentTempTime.DateTime.Minute < 10) print_from_program_space(PSTR(":0"));
		else print_from_program_space(PSTR(":"));
		print_long( xCurrentTempTime.DateTime.Minute );
		if(xCurrentTempTime.DateTime.Second < 10) print_from_program_space(PSTR(":0"));
		else print_from_program_space(PSTR(":"));
		print_long( xCurrentTempTime.DateTime.Second );

		// display the maximum temperature, time and date
		lcd_goto_xy(0, 2);          // go to the first character of the third LCD line
		print_from_program_space(PSTR("Max "));			// added spaces are to overwrite left over chars
		print_long( xMaximumTempTime.Temperature  ); // print the maximum temperature value
		print_from_program_space(PSTR("C "));			// added spaces are to overwrite left over chars

		lcd_goto_xy(8, 2);          // go to the ninth character of the third LCD line
		if(xMaximumTempTime.DateTime.Hour < 10) print_from_program_space(PSTR(" "));
		print_long( xMaximumTempTime.DateTime.Hour);
		if(xMaximumTempTime.DateTime.Minute < 10) print_from_program_space(PSTR(":0"));
		else print_from_program_space(PSTR(":"));
		print_long( xMaximumTempTime.DateTime.Minute );
		print_from_program_space(PSTR(" "));
		print_long( xMaximumTempTime.DateTime.Date );
		print_from_program_space(PSTR("/"));

		// display the minimum temperature, time and date
		print_long( xMaximumTempTime.DateTime.Month );
		if((xMaximumTempTime.DateTime.Date < 10) && (xMaximumTempTime.DateTime.Month < 10)) print_from_program_space(PSTR("  "));// added spaces are to overwrite left over chars
		else if((xMaximumTempTime.DateTime.Date < 10) || (xMaximumTempTime.DateTime.Month < 10)) print_from_program_space(PSTR(" "));// added spaces are to overwrite left over chars

		lcd_goto_xy(0, 3);          // go to the first character of the forth LCD line
		print_from_program_space(PSTR("Min "));			// added spaces are to overwrite left over chars
		print_long( xMinimumTempTime.Temperature ); // print the minimum temperature value
		print_from_program_space(PSTR("C "));			// added spaces are to overwrite left over chars

		lcd_goto_xy(8, 3);          // go to the ninth character of the fourth LCD line
		if(xMinimumTempTime.DateTime.Hour < 10) print_from_program_space(PSTR(" "));
		print_long( xMinimumTempTime.DateTime.Hour);
		if(xMinimumTempTime.DateTime.Minute < 10) print_from_program_space(PSTR(":0"));
		else print_from_program_space(PSTR(":"));
		print_long( xMinimumTempTime.DateTime.Minute );
		print_from_program_space(PSTR(" "));
		print_long( xMinimumTempTime.DateTime.Date );
		print_from_program_space(PSTR("/"));
		print_long( xMinimumTempTime.DateTime.Month );
		if((xMinimumTempTime.DateTime.Date < 10) && (xMinimumTempTime.DateTime.Month < 10)) print_from_program_space(PSTR("  "));// added spaces are to overwrite left over chars
		else if((xMinimumTempTime.DateTime.Date < 10) || (xMinimumTempTime.DateTime.Month < 10)) print_from_program_space(PSTR(" "));// added spaces are to overwrite left over chars

		if(xCurrentTempTime.DateTime.Second == 0)
			xSemaphoreGive( xRetrogradeSemaphore );

//		xSerialPrintf_P(PSTR("LCD HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
	}
}


static void TaskWriteRTCRetrograde(void *pvParameters) // Write RTC to Retrograde Hands
{
    (void) pvParameters;;

    xRTCArray xValues;     // Holds result values from the Real Time Clock DS1307.

    uint16_t servoHours_uS = 1500;
    uint16_t servoMinutes_uS = 1500;
    uint8_t firstPass = pdTRUE;

	if( xSemaphoreTake( xADCSemaphore, portMAX_DELAY ) == pdTRUE )
	{
		// We were able to obtain the semaphore and can now access the shared resource.
		// We don't want anyone using the ADC during servo moves, so take the semaphore.
		// There is too much noise on Vcc to get a clean sample.

		start_PWM_hardware();  // start the PWM TimerX hardware depending on the Timer #define in FreeRTOSConfig.h
		// Servos driving the hands, drags the Vcc down, drastically affecting the ADC0 (temperature) reading.

		// delay 2000mS to ensure hands are stopped before releasing.
		vTaskDelay( 2000 / portTICK_RATE_MS );

		xSemaphoreGive( xADCSemaphore ); // now the ADC can be used again.
	}

    while(1)
    {
    	// See if we can obtain the semaphore.  If the semaphore is not available
		// wait 50 ticks to see if it becomes free.
		if( xSemaphoreTake( xRetrogradeSemaphore, ( portTickType ) 50 ) == pdTRUE )
		{
			// We were able to obtain the semaphore and can now access the
			// shared resource.

			/* Block on the queue to wait for any data to arrive. */
			if( xI2CRTCQueue != 0 )
			{
				if( xQueuePeek( xI2CRTCQueue, &xValues, portMAX_DELAY) == pdPASS )
				{

					if( firstPass == pdTRUE) // Set hour hand servo on power-on once,
					{
						// convert to a range of 700uS to 2300uS over 24 hours.
						servoHours_uS = (uint16_t)(2300 - ((float)xValues.Minute + (float)xValues.Hour*60 )/1439*(2300-700));
						firstPass = pdFALSE;

					} else {

						switch( xValues.Minute )  // otherwise update the hour hand once every quarter hour.
						{
							case 0:
							case 15:
							case 30:
							case 45:
							// convert to a range of 700uS to 2300uS over 24 hours.
							servoHours_uS = (uint16_t)(2300 - ((float)xValues.Minute + (float)xValues.Hour*60 )/1439*(2300-700));
							break;
						}

					}

					// convert to a range of 700uS to 2300uS over 60 minutes.
					servoMinutes_uS = (uint16_t)(2300 - (float)xValues.Minute/59*(2300-700));

					// See if we can obtain the ADC semaphore.  If the semaphore is not available
					// wait for as long as we can to see if it becomes free.
					if( xSemaphoreTake( xADCSemaphore, portMAX_DELAY ) == pdTRUE )
					{
						// We were able to obtain the semaphore and can now access the shared resource.
						// We don't want anyone using the ADC during servo moves, so take the semaphore.
						// There is too much noise on Vcc to get a clean sample.

						set_PWM_hardware( servoHours_uS, servoMinutes_uS );

						// Servos driving, drags the Vcc down, drastically affecting the ADC0 reading.
						// delay 2000mS to ensure hands are stopped before releasing.
						vTaskDelay( 2000 / portTICK_RATE_MS );
						xSemaphoreGive( xADCSemaphore ); // now the ADC can be used again.
					}


				}
			}
		}

//       xSerialPrintf_P(PSTR("RTC Servo HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
    }
}



/*-----------------------------------------------------------*/

/*
    This function is only run when the time needs to be set.
    Enter the correct time (for some time in the near future).
    Compile and load the program, check the time is loading correctly.
    Then, press the restart button 4 seconds before time entered.
    Comment out the call in main() once the time is set correctly.
*/

// set the date & time to the values given here.

void setDateDs1307()
{
	xRTCArraySto xSettings;     // Holds  values for the RTC DS1307

	xSettings.I2CAddress = DS1307 + I2C_WRITE;		// set device address and write mode
	xSettings.Command	= 0x00;						// Write to the first address 0x00 (Seconds)
	xSettings.Second	= 0;			// 0-59
    xSettings.Minute	= 24;     		// 0-59
    xSettings.Hour		= 22;       	// 1-23
    xSettings.Day		= 7;          	// Sun=1, Mon=2, Tue=3, Wed=4, Thur=5, Fri=6, Sat=7
    xSettings.Date		= 25;        	// 1-28/29/30/31
    xSettings.Month		= 2;       		// Jan=1,... Dec=12
    xSettings.Year		= 12;        	// '00 - '99
    xSettings.Control   = SQWDISABLEOUT;			// disable the 1Hz square wave

    setDateTimeDS1307( &xSettings );
//	if (I2C_Check_Free_After_Stop() == pdTRUE )
//		I2C_Master_Start_Transceiver_With_Data( (uint8_t *)&xSettings, sizeof(xSettings));
}


/*-----------------------------------------------------------*/

int8_t ReadADCSensors(void) // Read ADC Sensor for Thermal LM335z
{
	// Variables for the analogue conversion on ADC Sensors
    const uint8_t samples = 20;        	// determines the number of samples taken for averaging
    uint16_t sum;               		// holds the summated samples
    uint8_t i = samples;

	if( xADCSemaphore != NULL )
	{
		// See if we can obtain the semaphore.  If the semaphore is not available
		// wait 5 ticks to see if it becomes free.

		if( xSemaphoreTake( xADCSemaphore, ( portTickType ) 5 ) == pdTRUE )
		{
			// We were able to obtain the semaphore and can now access the shared resource.
			// We want to have the ADC for us alone, as it takes some time to sample,
			// so we don't want it getting stolen during the middle of a conversion.

			do
			{
				startAnalogConversion(0, EXTERNAL_REF);   // start next conversion

				while( analogIsConverting() )
				{
					_delay_ms(1);     // wait until conversion ready
				}
				sum += analogConversionResult();	// sum the results

			} while (--i);

			xSemaphoreGive( xADCSemaphore );

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

			// The 497 is the Power Supply Voltage in mV / 10. 1023 is the number of ADC values.
			// 271.15 is the adjustment from Kelvin (273.15) and the offset relating to the Temp Sensor error correction.
		} else {
			return 0x7f; // no sample taken so return 0x7f.
		}
	}
	return (int8_t) (( (portFLOAT) sum * 497.0 / (portFLOAT)(1023 * samples))- 273.15);  // and return the current temp
}


