#ifndef CHEADER_RETROGRADE
# define CHEADER_RETROGRADE

/*
File:   retrograde.h
*/

#ifdef __cplusplus
extern "C" {
#endif

#include <avr/eeprom.h>

/* RTC interface (using I2C) include file */
#include <rtc.h>


/*--------------Definitions-------------------*/

#define LED_PORT		PORTB
#define LED_PORT_DIR	DDRB
#define LED_PIN			PINB5

#define LINE_SIZE 		80			// size of command line (on heap)

#define ADC_SAMPLES		5 			// is a 4 ^ n number. ie. 5 is 4^5 = 1024 samples. 6 is the maximum for uint16_t division.


/*--------------Structures--------------------*/

/* structure to receive the DS1307 RTC & Temperature parameters */
typedef struct
{
	xRTCArray 	DateTime;      // Structure containing the Date and Time
    portFLOAT	Temperature;   // Insert a Temperature, eg for Maximum or Minimum temps.
} xRTCTempArray, * pRTCTempArray;


/*--------------Global Variables--------------------*/


xTaskHandle xTaskWriteRTCRetrograde;	// make a Task handle so we can suspend and resume the Retrograde hands task.

/* Create a Semaphore binary flag for the ADC. To ensure only single access. */
xSemaphoreHandle xADCSemaphore;

/* Create a handle for the serial port. */
xComPortHandle xSerialPort;

uint8_t * LineBuffer;					// put line buffer on heap (with pvPortMalloc).

pRTCArraySto SetTimeDate;				// this pointer to a structure for storing the time to be set.

xRTCTempArray xCurrentTempTime; 		// structure to hold the I2C Current time value
xRTCTempArray xMaximumTempTime;
xRTCTempArray xMinimumTempTime;

//  EEPROM structures to hold the extreme temperatures, and the time these were achieved.
xRTCTempArray EEMEM xMaximumEverTempTime;
xRTCTempArray EEMEM xMinimumEverTempTime;



/*-----------------------------------------------------------*/

static void TaskWriteLCD(void *pvParameters); // Write to LCD

static void TaskWriteRTCRetrograde(void *pvParameters); // Write RTC to Retrograde Hands

static void TaskMonitor(void *pvParameters);		// Serial monitor for Retrograde

/*-----------------------------------------------------------*/


#ifdef __cplusplus
}
#endif



#endif // CHEADER_RETROGRADE

