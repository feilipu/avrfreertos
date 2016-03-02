/*
 * rtc.h
 *
 *  Created on: 16/02/2012
 *      Author: Phillip Stevens
 */

#ifndef RTC_H_
#define RTC_H_

#include "FreeRTOS.h"
#include "time.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef portRTC_DEFINED

// Definitions for the DS1307 RTC

/* Define the addresses of the RTC device that we want to manage on i2c bus.*/
#define DS1307			0xD0		// device address of DS1307 Real Time Clock is 0xD0 (see data sheet)
#define DS3231			0xD0		// device address of DS3231 Real Time Clock is 0xD0 (see data sheet)

/* definitions for use in the DS1307 Control byte */
# define SQWDISABLEOUT  0b10000000  //  level SQW should take if DISABLED
# define SQWENABLE      0b00010000  //  set to 1 to enable SQW output
# define SQWRS1         0b00000010  //  RS1 & RS0 set to 0 for 1Hz output
# define SQWRS0         0b00000001

/*-----------------------------------------------------------*/

// initialise I2C master interface, need to do this once only BEFORE using the RTC.
// If there are two I2C processes, then do it during the system initiation.
// I2C_Master_Initialise((ARDUINO<<I2C_ADR_BITS) | (pdTRUE<<I2C_GEN_BIT));

uint8_t getDateTimeDS1307(tm * timeDate);    // Get the date & time as needed.

uint8_t setDateTimeDS1307(tm * timeDateSet); // Set the date & time initially & as needed.

/*----------------------------------------------------------------*/
#endif

#ifdef __cplusplus
}
#endif


#endif /* RTC_H_ */
