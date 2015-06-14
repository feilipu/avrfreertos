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

#define NBLOBS      128
#define OFFSCREEN   -16384

/*----------Global used for HAL management-------------*/
extern FT_GPU_HAL_Context_t * phost;			// optional, just to make it clear where this comes from

struct xy {										// somewhere to store all the blob locations
  int16_t x, y;
} blobs[NBLOBS];

/*--------------Function Definitions-------------------*/

int main(void) __attribute__((OS_main));		// optional, just good practice

static void TaskWriteLCD(void *pvParameters);	// define a single task to write to Gameduino 2 LCD

/*-----------------Functions---------------------------*/

/* Main program loop */
int main(void)
{
    xTaskCreate(	// create a task to write on the Gameduino 2 LCD
		TaskWriteLCD
		,  (const portCHAR *)"WriteLCD"
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
	FT_API_Touch_Config();										// initialise the FT800 Touch capability.

	for (uint8_t i = 0; i < NBLOBS; ++i) {
		blobs[i].x = OFFSCREEN;
		blobs[i].y = OFFSCREEN;
	}

	while(1)													// a freeRTOS task should never return
    {
		static uint8_t blob_i;									// the blob we're currently processing
		uint32_t readTouch;										// xy coordinates of a touch are stored in uint32_t

		// this is the touch interface stuff
		readTouch = FT_GPU_HAL_Rd32(phost, REG_TOUCH_SCREEN_XY);// the screen location of the last touch is stored in REG_TOUCH_SCREEN_XY
		if (readTouch != NIL_TOUCH_XY)							// if there was a touch
		{
			blobs[blob_i].x  = (int16_t)((readTouch >> 16) & 0xffff) << 4; // read where x axis touch occurred, and scale it
			blobs[blob_i].y = (int16_t)(readTouch & 0xffff) << 4;		   // read where y axis touch occurred, and scale it
		} else {
		    blobs[blob_i].x = OFFSCREEN;						// if there was no touch, draw the blob OFFSCREEN
		    blobs[blob_i].y = OFFSCREEN;
		}
		blob_i = (blob_i + 1) & (NBLOBS - 1);					// increment to the next blob for touch interaction

		// this is the display interface stuff
		FT_API_Write_CoCmd( CMD_DLSTART );						// initialise and start a display list (DL)

		FT_API_Write_CoCmd( CLEAR_COLOR_RGB(0xe0, 0xe0, 0xe0) );// set the colour to which the screen will be cleared
		FT_API_Write_CoCmd( CLEAR(1,1,1) );						// clear the screen

		FT_API_Write_CoCmd( BEGIN(POINTS) );					// start to write POINTS into the Display List (DL)

		for (uint8_t i = 0; i < NBLOBS; ++i)
		{
			// Blobs fade away and swell as they age
			FT_API_Write_CoCmd( COLOR_A(i << 1) );				// set an alpha transparency
			FT_API_Write_CoCmd( POINT_SIZE((1024 + 16) - (i << 3)) );

			// Random colour for each blob, keyed from (blob_i + i)
			uint8_t j = (blob_i + i) & (NBLOBS - 1);
			uint8_t r = j * 17;
			uint8_t g = j * 23;
			uint8_t b = j * 147;
			FT_API_Write_CoCmd( COLOR_RGB(r, g, b) );

			// Draw it!
			FT_API_Write_CoCmd( VERTEX2F(blobs[j].x, blobs[j].y) );
		}

		FT_API_Write_CoCmd( END() );							// finish writing POINTS into the active DL

		FT_API_Write_CoCmd( DISPLAY() );						// close the active Display List (DL)
		FT_API_Write_CoCmd( CMD_SWAP );						    // Do a DL swap to render the just written DL

		FT_API_WaitCmdfifo_empty();								// Wait till co-processor completes all operations
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
									portCHAR *pcTaskName )
{
	DDRB  |= _BV(DDB5);
	PORTB |= _BV(PORTB5);       // main (red PB5) LED on. Turn Arduino LED on, and die.
	while(1);
}
/*-----------------------------------------------------------*/

