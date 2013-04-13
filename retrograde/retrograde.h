#ifndef CHEADER_RETROGRADE
# define CHEADER_RETROGRADE

/*
File:   retrograde.h
*/

#ifdef __cplusplus
extern "C" {
#endif

/* RTC interface (using I2C) include file */
#include <rtc.h>
/*-----------------------------------------------------------*/

#define LED_PORT		PORTB
#define LED_PORT_DIR	DDRB
#define LED_PIN			PINB5

/*-----------------------------------------------------------*/

/*
 * Declare a variable of type xQueueHandle.
 * One queue will have the i2c RTC values written to it.
 */
xQueueHandle xI2CRTCQueue;

/* Create a handle for the serial port. */
xComPortHandle xSerialPort;

/* Create a Semaphore binary flag for the ADC. To ensure only single access. */
xSemaphoreHandle xADCSemaphore;

/* Create a Semaphore binary flag to trigger the Retrograde Analogue Hands. */
xSemaphoreHandle xRetrogradeSemaphore;

/*-----------------------------------------------------------*/

static void TaskBlinkGreenLED(void *pvParameters); // Main Arduino (Green) LED Blink

static void TaskReadI2CRTC(void *pvParameters);   // Read I2C Bus for RTC

static void TaskWriteLCD(void *pvParameters); // Write to LCD

static void TaskWriteRTCRetrograde(void *pvParameters); // Write RTC to Retrograde Hands


/*-----------------------------------------------------------*/

//  EEPROM Arrays to hold the extreme temperatures, and the time these were achieved.
xRTCTempArray EEMEM xMaximumEverTempTime;
xRTCTempArray EEMEM xMinimumEverTempTime;


#ifdef __cplusplus
}
#endif



#endif // CHEADER_RETROGRADE

