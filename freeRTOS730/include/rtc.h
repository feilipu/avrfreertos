/*
 * rtc.h
 *
 *  Created on: 16/02/2012
 *      Author: Phillip Stevens
 */

#ifndef RTC_H_
#define RTC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <FreeRTOS.h>

#ifdef portRTC_DEFINED


// Definitions for the DS1307 RTC

/* Define the addresses of the RTC device that we want to manage on i2c bus.*/
#define DS1307          0xD0      // device address of DS1307 Real Time Clock is 0xD0 (see data sheet)
#define ARDUINO			0x32	  // device address of Freetronics / Arduino (just making this up, not special)

// Sample I2C transmission commands
#define I2C_GEN_CALL         0x00  // The General Call address is 0
#define I2C_CMD_MASTER_WRITE 0x10
#define I2C_CMD_MASTER_READ  0x20

// Sample I2C transmission states, used in the main application.
#define SEND_DATA             0x01
#define REQUEST_DATA          0x02
#define READ_DATA_FROM_BUFFER 0x03

/* definitions for use in the Control byte */
# define SQWDISABLEOUT  0b10000000   //  level SQW should take if DISABLED
# define SQWENABLE      0b00010000   //  set to 1 to enable SQW output
# define SQWRS1         0b00000010   //  RS1 & RS0 set to 0 for 1Hz output
# define SQWRS0         0b00000001

typedef enum {
	Sunday = 1,
	Monday    ,
	Tuesday   ,
	Wednesday ,
	Thursday  ,
	Friday    ,
	Saturday
} DayOfWeek;

/* structure to receive the DS1307 RTC parameters */
typedef struct
{
	uint8_t	   I2CAddress;	 // Address and read/write bit for I2C transaction. First Byte.
	uint8_t    Second;       //
	uint8_t    Minute;       //
	uint8_t    Hour;         // 1-12, 0-23 (depending on am pm/24 bit 6)
	DayOfWeek  Day;          // Sun=1, Mon=2, Tue=3, Wed=4, Thur=5, Fri=6, Sat=7
	uint8_t    Date;         //
	uint8_t    Month;        // Jan=1,... Dec=12
	uint8_t    Year;         // '00 through '99
	uint8_t    Control;      //
} xRTCArray, * pRTCArray;

/* structure to hold the DS1307 RTC parameters for transmission*/
// used ONLY for SETTING the time, where the Command byte is required.
typedef struct
{
	uint8_t	   I2CAddress;	  // Address and read/write bit for I2C transaction. First Byte.
	uint8_t    Command;       // Command or Address on the I2C bus
	uint8_t    Second;        //
	uint8_t    Minute;        //
	uint8_t    Hour;          // 1-12, 0-23 (depending on am pm/24 bit 6)
	DayOfWeek  Day;           // Sun=1, Mon=2, Tue=3, Wed=4, Thur=5, Fri=6, Sat=7
	uint8_t    Date;          //
	uint8_t    Month;         // Jan=1,... Dec=12
	uint8_t    Year;          // '00 through '99
	uint8_t    Control;       //
} xRTCArraySto, * pRTCArraySto;

/*-----------------------------------------------------------*/

uint8_t getDateTimeDS1307(pRTCArray xTimeDate);         // Get the date & time as needed.

uint8_t setDateTimeDS1307(pRTCArraySto xSettings);      // Set the date & time initially & as needed.

/*----------------------------------------------------------------*/
#endif

#ifdef __cplusplus
}
#endif


#endif /* RTC_H_ */
