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

#include "sprites_assets.h"

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

    uint8_t t = 0;

	FT_API_Boot_Config();										// initialise the Gameduino 2.

	FT_GPU_HAL_WrCmdBuf_P(phost, sprites_assets, sizeof(sprites_assets));
																// copy James' magic list of commands into the command buffer

	while(1)													// a freeRTOS task should never return
    {
		FT_API_Write_CoCmd( CMD_DLSTART );						// initialise and start a display list (DL)
		FT_API_Write_CoCmd( CLEAR(1,1,1) );						// clear the screen

		FT_API_Write_CoCmd( BEGIN(BITMAPS) );					// start to write BITMAPS into the DL
    	uint8_t j = t;
    	uint32_t v;
    	uint32_t r;
    	int16_t nspr = min(2001, max(256, 19 * t));
    	ft_prog_uint32_t * pv = sprites;

		for (uint16_t i = 0; i < nspr; ++i) {
			v = pgm_read_dword(pv++);
			r = pgm_read_dword(circle + j++);
			FT_GPU_HAL_WrCmd32(phost, v + r);
		}
		FT_API_Write_CoCmd( END());								// finish writing BITMAPS into the DL

		FT_API_Write_CoCmd( BEGIN(LINES) );						// start to write LINES into the DL
		FT_API_Write_CoCmd( COLOR_RGB(0x00, 0x00, 0x00) );
		FT_API_Write_CoCmd( COLOR_A(140) );						// set alpha channel transparency
		FT_API_Write_CoCmd( LINE_WIDTH( 28 * 16) );
		FT_API_Write_CoCmd( VERTEX2II(240 - 110, 136, 0, 0) );	// draw an alpha transparency background line
		FT_API_Write_CoCmd( VERTEX2II(240 + 110, 136, 0, 0) );
		FT_API_Write_CoCmd( END() );							// finish writing LINES into the DL

		FT_API_Write_CoCmd( RESTORE_CONTEXT() );				// with no SAVE_CONTEXT() command this restores default colours and values.

		FT_GPU_CoCmd_Number(phost, 215, 110, 31, OPT_RIGHTX, nspr);
    	FT_GPU_CoCmd_Text_P(phost, 229, 110, 31, 0, PSTR("sprites")); // use a PROGMEM string function, to save RAM

		FT_API_Write_CoCmd( DISPLAY() );						// close the active display list (DL)
		FT_API_Write_CoCmd( CMD_SWAP );						    // Do a DL swap to render the just written DL

		t++;
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

