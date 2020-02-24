////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////    main.c
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////


/* freeRTOS Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Gameduino 2 include file. */
#include "FT_Platform.h"

/*----------Global used for HAL management-------------*/
extern FT_GPU_HAL_Context_t * phost;			// optional, just to make it clear where this comes from

/*--------------Function Definitions-------------------*/

int main(void) __attribute__((OS_main));		// optional, just good practice

static void TaskWriteLCD(void *pvParameters);	// define a single task to write to Gameduino 2 LCD

/*-----------------Functions---------------------------*/

/* Main program loop */
int main(void)
{
    xTaskCreate(	// create a task to write on the Gameduino 2 LCD
		TaskWriteLCD
		,  (const char *)"WriteLCD"
		,  128		// number of bytes for the task stack
		,  NULL
		,  3		// priority of task (1 is highest priority, 4 lowest).
		,  NULL );

    vTaskStartScheduler();	// now freeRTOS has taken over, and the scheduler is running
}


/*-----------------------------------------------------------*/
/* Tasks                                                     */
/*-----------------------------------------------------------*/


static void TaskWriteLCD(void *pvParameters) // A Task to write to Gameduino 2 LCD
{
    (void) pvParameters;

	FT_API_Boot_Config();										// initialise the Gameduino 2.

	while(1)													// a freeRTOS task should never return
    {
		FT_API_Write_CoCmd( CMD_DLSTART );						// initialise and start a display list
//    	FT_API_Write_CoCmd( CLEAR_COLOR_RGB(0x10, 0x30, 0x00) );// set the colour to which the screen is cleared (using RGB triplets) OR
    	FT_API_Write_CoCmd( CLEAR_COLOR_X11(FORESTGREEN) );		// set the colour to which the screen is cleared (using X11 colour definitions)
    	FT_API_Write_CoCmd( CLEAR(1,1,1) );						// clear the screen

    	FT_GPU_CoCmd_Text_P(phost,FT_DispWidth/2, FT_DispHeight/2, 31, OPT_CENTER, PSTR("Hello World"));
    															// write "Hello World" to centre OPT_CENTRE of screen, with font 31
    															// String "Hello World" is stored in PROGMEM
    															// Functions with *_P all use PROGMEM Strings (don't use RAM)

    	FT_API_Write_CoCmd( DISPLAY() );						// close the display list
    	FT_API_Write_CoCmd( CMD_SWAP );							// swap active display list (double buffering), to display the new "Hello World" list
	}
}

/*-----------------------------------------------------------*/
/* Static Functions 										 */
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*\
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
\*-----------------------------------------------------------*/
void vApplicationStackOverflowHook( TaskHandle_t xTask,
									char *pcTaskName )
{
	DDRB  |= _BV(DDB5);
	PORTB |= _BV(PORTB5);       // main (red PB5) LED on. Turn Arduino LED on, and die.
	while(1);
}
/*-----------------------------------------------------------*/

