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

/* time interface include file */
#include "time.h"

/* extended string to integer */
#include "xatoi.h"

/* Gameduino 2 include file. */
#include "FT_Platform.h"

#include "./FT_SampleApp.h"


/*--------------Definitions-------------------*/

#define LINE_SIZE 		80			// size of command line (on heap)

/*--------------Global Variables--------------------*/

/* Create a Semaphore binary flag for the ADC. To ensure only single access. */
SemaphoreHandle_t xADCSemaphore;

//  EEPROM to hold the current Time Zone.
int8_t EEMEM eeSavedTZ;				// the time zone (in hours) saved through reboots.

uint8_t * LineBuffer = (void *)0;	// put line buffer on heap (with pvPortMalloc).

/*--------------Functions-------------------*/
static void TaskWriteLCD(void *pvParameters); // Write to LCD

static void TaskMonitor(void *pvParameters);  // Serial monitor for Gameduino2

static ft_void_t FT_Main_Menu_Clock(); // build the main menu and draw clock.
static ft_void_t FT_Set_Clock();       // set the system time and date.

static void get_line (uint8_t *buff, uint8_t len);

/*--------------Functions---------------------------*/

/* Main program loop */
int main(void) __attribute__((OS_main));

int main(void)
{

    // turn on the serial port for setting or querying the time .
	xSerialPort = xSerialPortInitMinimal( USART0, 115200, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)

    // Semaphores are useful to stop a thread proceeding, where it should be stopped because it is using a resource.
    if( xADCSemaphore == (void *)0 ) 					// Check to see if the ADC semaphore has not been created.
    {
    	xADCSemaphore = xSemaphoreCreateBinary();	// binary semaphore for ADC - Don't sample temperature when hands are moving (voltage droop).
		if( ( xADCSemaphore ) != (void *)0 )
			xSemaphoreGive( ( xADCSemaphore ) );	// make the ADC available
    }

	eeprom_busy_wait();
	set_zone( (int32_t)eeprom_read_byte((ft_const_uint8_t *)&eeSavedTZ) * ONE_HOUR ); // The Time Zone that we saved previously.

	avrSerialxPrint_P(&xSerialPort, PSTR("\r\nHello World!\r\n")); // Ok, so we're alive...

    xTaskCreate(
		TaskWriteLCD
		,  (const char *)"WriteLCD"
		,  512		// measured 73 free stack bytes
		,  (void *)0
		,  3
		,  (void *)0 ); // */

   xTaskCreate(
		TaskMonitor
		,  (const char *)"SerialMonitor"
		,  512		// measured 124 free stack bytes
		,  (void *)0
		,  2
		,  (void *)0 ); // */

	avrSerialPrintf_P(PSTR("\r\nFree Heap Size: %u\r\n"), xPortGetFreeHeapSize() ); // needs heap_1, heap_2 or heap_4 for this function to succeed.

    vTaskStartScheduler();

	avrSerialPrint_P(PSTR("\r\n\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...
}


/*-----------------------------------------------------------*/
/* Monitor                                                   */
/*-----------------------------------------------------------*/

static void TaskMonitor(void *pvParameters) // Monitor for Serial Interface
{
    (void) pvParameters;

	uint8_t *ptr;
	int32_t p1;

	time_t timestamp;
	tm calendar;

	// create the buffer on the heap (so they can be moved later).
	if(LineBuffer == (void *)0) // if there is no Line buffer allocated (pointer is NULL), then allocate buffer.
		if( !(LineBuffer = (uint8_t *) pvPortMalloc( sizeof(uint8_t) * LINE_SIZE )))
			xSerialPrint_P(PSTR("pvPortMalloc for *LineBuffer fail..!\r\n"));

    while(1)
    {
    	xSerialPutChar(&xSerialPort, '>');

		ptr = LineBuffer;
		get_line(ptr, (uint8_t)(sizeof(uint8_t)* LINE_SIZE)); //sizeof (Line);

		switch (*ptr++) {

		case 'h' : // help
			xSerialPrint_P( PSTR("s  - show minimum ever heap size\r\n") );
			xSerialPrint_P( PSTR("h  - show this message\r\n") );
			xSerialPrint_P( PSTR("b  - reboot ft800 device\r\n") );
			xSerialPrint_P( PSTR("i  - show ft800 logo\r\n") );
			xSerialPrint_P( PSTR("r  - reset to blank display\r\n") );
			xSerialPrint_P( PSTR("t  - set / show the time: t [<year yyyy> <month mm> <date dd> <hour hh> <minute mm> <second ss>]\r\n") );
			xSerialPrint_P( PSTR("z  - set the time zone +-hours (before you first set the time): z [<timezone zz>] \r\n") );
			break;

		case 's' : // reset
			xSerialPrintf_P(PSTR("\r\nMinimum Free Heap Size: %u\r\n"), xPortGetMinimumEverFreeHeapSize() ); // needs heap_1, heap_2 or heap_4 for this function to succeed.
			xSerialPrintf_P(PSTR("Current Free Heap Size: %u\r\n"), xPortGetFreeHeapSize() ); // needs heap_1, heap_2 or heap_4 for this function to succeed.
			break;

		case 'b' : // reboot

			FT_API_Boot_Config();

			xSerialPrintf_P(PSTR("reg_touch_rz = 0x%x"), FT_GPU_HAL_Rd16(phost, REG_TOUCH_RZ));
			xSerialPrintf_P(PSTR("\r\nreg_touch_rzthresh = 0x%x"), FT_GPU_HAL_Rd32(phost, REG_TOUCH_RZTHRESH));
			xSerialPrintf_P(PSTR("\r\nreg_touch_tag_xy = 0x%x"),FT_GPU_HAL_Rd32(phost, REG_TOUCH_TAG_XY));
			xSerialPrintf_P(PSTR("\r\nreg_touch_tag = 0x%x\r\n"),FT_GPU_HAL_Rd32(phost, REG_TOUCH_TAG));
			break;

		case 'i' : // info
			FT_Info();
			break;

		case 'r' : // reset to blank screen.
			FT_Home_Setup();
			break;

		case 't' :	/* t [<year yyyy> <month mm> <date dd> <hour hh> <minute mm> <second ss>] */

			if (xatoi(&ptr, &p1)) {
				calendar.tm_year = (uint16_t)p1 - 1900;
				xatoi(&ptr, &p1); calendar.tm_mon = (uint8_t)p1 - 1; // FFS January is 0 month.
				xatoi(&ptr, &p1); calendar.tm_mday = (uint8_t)p1;
				xatoi(&ptr, &p1); calendar.tm_hour = (uint8_t)p1;
				xatoi(&ptr, &p1); calendar.tm_min = (uint8_t)p1;
				xatoi(&ptr, &p1); calendar.tm_sec = (uint8_t)p1;

				calendar.tm_isdst = 0;
		        set_system_time( mktime(&calendar) );
			}

			time(&timestamp);
			ctime_r( (time_t *)&timestamp, (char *)LineBuffer );
			xSerialPrintf_P(PSTR("Local Time: %s - %u\r\n"), LineBuffer, timestamp);
			break;

		case 'z' :	/* z [<Time Zone (-)zz>]  */

			if (xatoi(&ptr, &p1)) {
				set_zone( ((int8_t)p1 * (int32_t)ONE_HOUR) );
				eeprom_busy_wait();
				eeprom_update_byte( (ft_uint8_t *)&eeSavedTZ, (int8_t)p1 );
				xSerialPrintf_P(PSTR("Input Time Zone %i\r\n"), (int8_t)p1 );
			}
			eeprom_busy_wait();
			xSerialPrintf_P(PSTR("Saved Time Zone %i\r\n"), (int8_t)eeprom_read_byte((ft_const_uint8_t *)&eeSavedTZ) );

			time(&timestamp);
			ctime_r( (time_t *)&timestamp, (char *)LineBuffer );
			xSerialPrintf_P(PSTR("Local Time: %s - %u\r\n"), LineBuffer, timestamp );
			break;

		default :
			break;

		}
// 		xSerialPrintf_P(PSTR("\r\nSerial Monitor: Stack HighWater @ %u"), uxTaskGetStackHighWaterMark(NULL));
//		xSerialPrintf_P(PSTR("\r\nMinimum Ever Heap Free: %u\r\n"), xPortGetMinimumEverFreeHeapSize() ); // needs heap_1, heap_2 or heap_4 for this function to succeed.
    }

}


/*-----------------------------------------------------------*/
static void TaskWriteLCD(void *pvParameters) // Write to LCD
{
    (void) pvParameters;

    TickType_t xLastWakeTime __attribute__ ((unused));
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

	ft_uint8_t readTag;

	FT_API_Boot_Config();
	FT_API_Touch_Config();

//	xSerialPrintf_P(PSTR("reg_touch_rz = 0x%x"), FT_GPU_HAL_Rd16(phost, REG_TOUCH_RZ));
//	xSerialPrintf_P(PSTR("\r\nreg_touch_rzthresh = 0x%x"), FT_GPU_HAL_Rd32(phost, REG_TOUCH_RZTHRESH));
//	xSerialPrintf_P(PSTR("\r\nreg_touch_tag_xy = 0x%x"),FT_GPU_HAL_Rd32(phost, REG_TOUCH_TAG_XY));
//	xSerialPrintf_P(PSTR("\r\nreg_touch_tag = 0x%x\r\n"),FT_GPU_HAL_Rd32(phost, REG_TOUCH_TAG));

    while(1)
    {

    	FT_Main_Menu_Clock();

    	readTag = FT_GPU_HAL_Rd8(phost, REG_TOUCH_TAG);

    	if(readTag != 0x00 && readTag != 0xFF)
    		{
			/* Play sound  */
			FT_GPU_HAL_Wr8(phost, REG_VOL_SOUND,0xFF); //set the volume to maximum
			FT_GPU_HAL_Wr16(phost, REG_SOUND, (0x48<< 8) | 0x41); // C5 MIDI note on xzylophone
			FT_GPU_HAL_Wr8(phost, REG_PLAY, 1); // play the sound
    		}

		switch (readTag) {

#ifdef FT_APP_ENABLE_APIS_SET0
			case 10 : //sample apps
				/* Sample code for GPU primitives */
				FT_APP_Screen_P(PSTR("Set 0   START"));
				FT_APP_GPU_Fonts();
				FT_APP_GPU_Text8x8();
				FT_APP_GPU_TextVGA();
				FT_APP_GPU_Points();
				FT_APP_GPU_Lines();
				FT_APP_GPU_Rectangles();
				FT_APP_GPU_Bitmap();
				FT_APP_GPU_LineStrips();
				FT_APP_GPU_EdgeStrips();
				FT_APP_GPU_Scissor();
				FT_APP_GPU_Polygon();
				FT_APP_GPU_Cube(); // */
				FT_APP_GPU_Ball_Stencil();
//				FT_APP_GPU_Bargraph(); // fixme fix the random range to stop overflow
				FT_APP_GPU_FTDIString();
				FT_APP_GPU_StreetMap();
				FT_APP_GPU_AdditiveBlendText();
				FT_APP_GPU_MacroUsage();
				FT_APP_GPU_AdditiveBlendPoints();
				FT_APP_Screen_P(PSTR("Set 0   End!"));
				break;
#endif /* #ifdef FT_API_ENABLE_APIS_SET0 */

#ifdef FT_APP_ENABLE_APIS_SET1
			case 11 : //sample apps

				FT_APP_Screen_P(PSTR("Set 1   START"));
				FT_APP_CoPro_Widget_Logo();
//		    	FT_APP_CoPro_Widget_Calibrate();
				FT_APP_CoPro_Widget_Clock();
				FT_APP_CoPro_Widget_Gauge();
				FT_APP_CoPro_Widget_Gradient();
				FT_APP_CoPro_Widget_Keys();
				FT_APP_CoPro_Widget_Keys_Interactive();
				FT_APP_CoPro_Widget_Number();
				FT_APP_CoPro_Widget_Text();
				FT_APP_CoPro_Widget_Button();
				FT_APP_CoPro_Widget_Progressbar();
				FT_APP_CoPro_Widget_Scroll();
				FT_APP_CoPro_Widget_Slider();
				FT_APP_CoPro_Widget_Dial();
				FT_APP_CoPro_Widget_Toggle();
				FT_APP_CoPro_Widget_Spinner();
				FT_APP_Screen_P(PSTR("Set 1   END!"));
				break;
#endif /* #ifdef FT_API_ENABLE_APIS_SET1 */

#ifdef FT_APP_ENABLE_APIS_SET2
			case 12 : //sample apps
				FT_APP_Screen_P(PSTR("Set 2   START"));
				FT_APP_CoPro_Inflate();
				FT_APP_CoPro_Loadimage();
				FT_APP_Screen_P(PSTR("Set 2   END!"));
				break;
#endif /* #ifdef FT_API_ENABLE_APIS_SET2 */

#ifdef FT_APP_ENABLE_APIS_SET3
			case 13 : //sample apps
				FT_APP_Screen_P(PSTR("Set 3   START"));
				FT_APP_CoPro_Setfont();
				FT_APP_Screen_P(PSTR("Set 3   END!"));
				break;
#endif /* #ifdef FT_API_ENABLE_APIS_SET3 */

#ifdef FT_APP_ENABLE_APIS_SET4
			case 14 : //sample apps
				FT_APP_Screen_P(PSTR("Set 4   START"));
				/* Sample code for coprocessor widgets */

				FT_APP_CoPro_AppendCmds();
//				FT_APP_CoPro_Snapshot(); // fixme not working properly
				FT_APP_CoPro_Matrix();
				FT_APP_CoPro_Screensaver();
//				FT_APP_CoPro_Widget_Calibrate();
				FT_APP_Touch();
				FT_APP_CoPro_Track();
				FT_APP_CoPro_Sketch();

				FT_APP_Sound();

//				FT_APP_PowerMode();

				FT_APP_Screen_P(PSTR("Set 4   END!"));
				break;
#endif /* #ifdef FT_API_ENABLE_APIS_SET4 */

			case 20 : // set the time / date
				FT_Set_Clock();

				break;

			default :
				break;
		}
//		xSerialPrintf_P(PSTR("\r\nWriteLCD: Stack HighWater @ %u"), uxTaskGetStackHighWaterMark(NULL));
//		xSerialPrintf_P(PSTR("\r\nMinimum Ever Heap Free: %u\r\n"), xPortGetMinimumEverFreeHeapSize() ); // needs heap_1, heap_2 or heap_4 for this function to succeed.
		vTaskDelayUntil( &xLastWakeTime, 50 / portTICK_PERIOD_MS );
	}
}


/*-----------------------------------------------------------*/
/* static functions */
/*-----------------------------------------------------------*/

ft_void_t FT_Main_Menu_Clock()
{
	ft_int16_t xOffset,yOffset,bWidth,bHeight,bDisty;

	time_t currentTime;
	tm calendar;

	bWidth = 120;						// Draw buttons 120x28 resolution
	bHeight = 28;
	bDisty = bHeight + (bHeight>>1);	// separate buttons by half height
	xOffset = bWidth>>1;				// half a button width from the edge
	yOffset = (FT_DispHeight/2 - 3*bDisty);

	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_X11(BLACK));
	FT_API_Write_CoCmd(CLEAR(1,1,1));
	FT_API_Write_CoCmd(COLOR_X11(YELLOW));

	FT_API_Write_CoCmd(TAG_MASK(FT_TRUE));

	/* 3d effect with gradient colour */
	FT_GPU_CoCmd_FgColor(phost, X11(DARKGREEN));
	FT_GPU_CoCmd_GradColor(phost, X11(TOMATO));

	FT_API_Write_CoCmd(TAG(10));
	FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,OPT_3D,PSTR("Set 0"));

	yOffset += bDisty;
	FT_API_Write_CoCmd(TAG(11));
	FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,OPT_3D,PSTR("Set 1"));

	yOffset += bDisty;
	FT_API_Write_CoCmd(TAG(12));
	FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,OPT_3D,PSTR("Set 2"));

	yOffset += bDisty;
	FT_API_Write_CoCmd(TAG(13));
	FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,OPT_3D,PSTR("Set 3"));

	yOffset += bDisty;
	FT_API_Write_CoCmd(TAG(14));
	FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,OPT_3D,PSTR("Set 4"));

	yOffset += bDisty;
	FT_GPU_CoCmd_Text_P(phost, xOffset - 30, yOffset + 10, 28, 0, PSTR("Gameduino 2 Demos"));

	time(&currentTime);
	localtime_r(&currentTime, &calendar);
	FT_API_Write_CoCmd(TAG(20));
	FT_GPU_CoCmd_Clock(phost, FT_DispWidth - (FT_DispHeight/2),  FT_DispHeight/2, FT_DispHeight/2, OPT_NOBACK, calendar.tm_hour, calendar.tm_min, calendar.tm_sec, 0);

	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);

}

ft_void_t FT_Set_Clock()
{
    TickType_t xLastWakeTime __attribute__ ((unused));
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

	ft_int16_t xOffset,yOffset,bWidth,bHeight,bDistx,bDisty;

	ft_int8_t tz;
	time_t currentTime;
	tm calendar;

	ft_char8_t timeString[40];

	ft_uint8_t readTag;

	bWidth = 70;	// Draw buttons 70x28 resolution
	bHeight = 28;
	bDistx = bWidth + (bWidth>>1); // separate buttons by half width
	bDisty = bHeight + (bHeight>>1); // separate buttons by half height

	eeprom_busy_wait();
	tz = eeprom_read_byte( (ft_uint8_t *)&eeSavedTZ ); // get the current Time Zone

	time(&currentTime); // get the current system time stamp
	localtime_r(&currentTime, &calendar); // break it into the calendar structure

    while(1)
    {

		FT_GPU_CoCmd_Dlstart(phost);
		FT_API_Write_CoCmd(CLEAR_COLOR_X11(BLACK));
		FT_API_Write_CoCmd(CLEAR(1,1,1));

		FT_API_Write_CoCmd(TAG_MASK(FT_TRUE));		// turn on the TAG_MASK

		/* 3d effect with gradient colour */
		FT_GPU_CoCmd_FgColor(phost,0xFF0000);
		FT_GPU_CoCmd_GradColor(phost,0xB90007);

		FT_API_Write_CoCmd(COLOR_X11(YELLOW));		// write in YELLOW

		xOffset = (bWidth>>1); // half a button width from the edge
		yOffset = (FT_DispHeight/2 - 3*bDisty);		// centre 6 buttons

		FT_API_Write_CoCmd(TAG(10));				// set the first button to have TAG 10
		FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,OPT_3D, PSTR("-Yr"));

		yOffset += bDisty;
		FT_API_Write_CoCmd(TAG(20));
		FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,OPT_3D, PSTR("-Mth"));

		yOffset += bDisty;
		FT_API_Write_CoCmd(TAG(30));
		FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,OPT_3D, PSTR("-Day"));

		yOffset += bDisty;
		FT_API_Write_CoCmd(TAG(40));
		FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,OPT_3D, PSTR("-Hr"));

		yOffset += bDisty;
		FT_API_Write_CoCmd(TAG(50));
		FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,OPT_3D, PSTR("-Min"));

		yOffset += bDisty;
		FT_API_Write_CoCmd(TAG(60));
		FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,OPT_3D, PSTR("-Sec"));

		FT_API_Write_CoCmd(COLOR_X11(GREEN));		// write in GREEN

		xOffset += bDistx;
		yOffset = (FT_DispHeight/2 - 3*bDisty);

		FT_API_Write_CoCmd(TAG(11));
		FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,OPT_3D, PSTR("+Yr"));

		yOffset += bDisty;
		FT_API_Write_CoCmd(TAG(21));
		FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,OPT_3D, PSTR("+Mth"));

		yOffset += bDisty;
		FT_API_Write_CoCmd(TAG(31));
		FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,OPT_3D, PSTR("+Day"));

		yOffset += bDisty;
		FT_API_Write_CoCmd(TAG(41));
		FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,OPT_3D, PSTR("+Hr"));

		yOffset += bDisty;
		FT_API_Write_CoCmd(TAG(51));
		FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,OPT_3D, PSTR("+Min"));

		yOffset += bDisty;
		FT_API_Write_CoCmd(TAG(61));
		FT_GPU_CoCmd_Button_P(phost,xOffset,yOffset,bWidth,bHeight,28,OPT_3D, PSTR("+Sec"));

		xOffset += bDistx;
		yOffset = (FT_DispHeight/2 - 3*bDisty);

		timeString[0] = '\0';
		strcat_P(timeString, PSTR("TZ "));			// fill the temporary string with the TZ label we're setting on the Button
		FT_GPU_HAL_Dec2ASCII(timeString, tz);

		FT_API_Write_CoCmd(TAG(70));
		FT_GPU_CoCmd_Button(phost,xOffset,yOffset,bWidth,bHeight,28,OPT_3D, timeString ); // can't use PROGMEM here, computed string

		FT_API_Write_CoCmd(COLOR_X11(RED));			// write in RED
		FT_API_Write_CoCmd(TAG(80));
		FT_GPU_CoCmd_Clock(phost, FT_DispWidth - (FT_DispHeight/2),  FT_DispHeight/2, FT_DispHeight/2 - 20, OPT_NOBACK, calendar.tm_hour, calendar.tm_min, calendar.tm_sec, 0);

		asctime_r(&calendar, timeString);
		FT_GPU_CoCmd_Text(phost, FT_DispWidth - (FT_DispHeight/2), FT_DispHeight - 20, 28, OPT_CENTER, timeString);

		FT_API_Write_CoCmd(COLOR_X11(GREEN));		// write in GREEN

		xOffset = FT_DispWidth - bDistx;
		yOffset = FT_DispHeight/2 - 3*bDisty;

		FT_API_Write_CoCmd(TAG(81));
		FT_GPU_CoCmd_Button_P(phost, xOffset, yOffset, bWidth, bHeight, 28, OPT_3D, PSTR("SET"));

		FT_API_Write_CoCmd(DISPLAY());
		FT_GPU_CoCmd_Swap(phost);

		vTaskDelayUntil( &xLastWakeTime, 100 / portTICK_PERIOD_MS ); // slow down touches, or we can't humanly keep up

		/* Wait till coprocessor completes the operation */
		FT_GPU_HAL_WaitCmdfifo_empty(phost);

		readTag = FT_GPU_HAL_Rd8(phost, REG_TOUCH_TAG);

		if (readTag != 0x00 && readTag != 0xFF) // if we get a touch on one of our TAGS, play a sound
			{
			/* Play sound  */
			FT_GPU_HAL_Wr8(phost, REG_VOL_SOUND,0xFF); //set the volume to maximum
			FT_GPU_HAL_Wr16(phost, REG_SOUND, (0x48<< 8) | 0x41); // C5 MIDI note on xzylophone
			FT_GPU_HAL_Wr8(phost, REG_PLAY, 1); // play the sound
			}

		switch (readTag){

			case 10:
				if(calendar.tm_year == 100) {calendar.tm_year = 335;}
				else calendar.tm_year -= 1;
				break;
			case 20:
				if(calendar.tm_mon == 0) {calendar.tm_mon = 11; calendar.tm_year -= 1;}
				else calendar.tm_mon -= 1;
				break;
			case 30:
				if(calendar.tm_mday <= 1) {calendar.tm_mday = 31; calendar.tm_mon -= 1;}
				else calendar.tm_mday -= 1;
				break;
			case 40:
				if(calendar.tm_hour == 0) {calendar.tm_hour = 23; calendar.tm_mday -= 1;}
				else calendar.tm_hour -= 1;
				break;
			case 50:
				if(calendar.tm_min == 0) {calendar.tm_min = 59; calendar.tm_hour -= 1;}
				else calendar.tm_min -= 1;
				break;
			case 60:
				if(calendar.tm_sec == 0) {calendar.tm_sec = 59; calendar.tm_min -= 1;}
				else calendar.tm_sec -= 1;
				break;

			case 11:
				if(calendar.tm_year >= 335) {calendar.tm_year = 100;}
				else calendar.tm_year += 1;
				break;
			case 21:
				if(calendar.tm_mon >= 11) {calendar.tm_mon = 0; calendar.tm_year += 1;}
				else calendar.tm_mon += 1;
				break;
			case 31:
				if(calendar.tm_mday >= 31) {calendar.tm_mday = 1; calendar.tm_mon += 1;}
				else calendar.tm_mday += 1;
				break;
			case 41:
				if(calendar.tm_hour >= 23) {calendar.tm_hour = 0; calendar.tm_mday += 1;}
				else calendar.tm_hour += 1;
				break;
			case 51:
				if(calendar.tm_min >= 59) {calendar.tm_min = 0; calendar.tm_hour += 1;}
				else calendar.tm_min += 1;
				break;
			case 61:
				if(calendar.tm_sec >= 59) {calendar.tm_sec = 0; calendar.tm_min += 1;}
				else calendar.tm_sec += 1;
				break;

			case 70 :
				if(tz >= 12) {tz = -12;}
				else tz += 1;
				break;

			case 80 :
				return;
				break;

			case 81 :
				set_zone( tz * (int32_t)ONE_HOUR ); 	// first set the Time Zone, so the time being set represents the local time
		        set_system_time( mktime( &calendar ) ); // then set the system time here based on what we've entered into the calendar structure
		        eeprom_busy_wait();
		        eeprom_update_byte( (ft_uint8_t *)&eeSavedTZ, tz );		// write the TZ into EEPROM, so we don't have to do this every restart
				return;
				break;

			default:
				break;
		}

		mktime(&calendar); // normalise the broken down time, to make sure we stay inside real dates
	}
}

/*-----------------------------------------------------------*/
/* Additional helper functions */
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
									char *pcTaskName )
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

	strcat_P( (char*) pC, PSTR("\r\n"));
	strcat( (char*) pC, (char*) pcTaskName );
	strcat_P( (char*) pC, PSTR("\r\n"));

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

	while(1){ PINB |= _BV(PINB7); _delay_ms(100); } // main (red PB7) LED flash and die.
}

/*-----------------------------------------------------------*/

