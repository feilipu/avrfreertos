/*
* Copyright 2008 Jay Clegg.  All rights reserved.
*
*    This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
	Peggy2-i2c interface, Copyright 2008 by Jay Clegg.  All rights reserved.

	This code is designed for an unmodified Peggy 2.0 board sold by evilmadscience.com

	The code configures the Peggy as an TWI (I2C) slave.

	Companion code for an Arduino allows it to act as an TWI master, so that it can transmit
	frames to the peggy.

	Please see http://www.planetclegg.com/projects/Twi2Peggy.html for explanation of how all
	this is supposed to work.

	Credits goes to:
		Windell H Oskay, (http://www.evilmadscientist.com/)
			for creating the Peggy 2.0 kit, and getting 16 shades of gray working
		Geoff Harrison (http://www.solivant.com/peggy2/),
			for proving that interrupt driven display on the Peggy 2.0 was viable.
*/

////////////////////////////////////////////////////////////////////////////////////////////
// FPS must be high enough to not have obvious flicker, low enough that main loop has
// time to process one byte per pass.
// ~140 seems to be about the absolute max for me (with this code on avr-gcc 4.2, -Os),
// but compiler differences might make this maximum value larger or smaller.
// if the value is too high errors start to occur or it will stop receiving altogether
// conversely, any lower than 60 and flicker becomes apparent.
// note: further code optimization might allow this number to
// be a bit higher, but only up to a point...
// it *must* result in a value for OCR0A in the range of 1-255


////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////    main.c
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////


#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

// freeRTOS Scheduler include files.
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// freeRTOS added i2c Interface include file.
// I2C_BUFFER_SIZE: Set this to the largest message size that will be sent including address byte.

#include "i2cMultiMaster.h"
#include "spi.h"
#include "lib_crc.h"

/* serial interface & Digital include file. */
#include "serial.h"  // only need this for testing
#include "digitalAnalog.h"

// Peggy Video include file.
#include "PeggyVideoPong.h"

// Buzzer include file
#include "Buzzer.h"

/**************************************************************************************
 * Set the brightness of the pixel at x, y to 'brightness' (0=off).
 *************************************************************************************/
uint8_t getPixel(int8_t x,int8_t y);

void drawPixel(uint8_t x,uint8_t y, uint8_t brightness);

void drawLine(int8_t x, int8_t y, int8_t x2, int8_t y2, uint8_t brightness);
void drawSquare(int8_t x, int8_t y, int8_t x2, int8_t y2, uint8_t brightness);
void drawRect(int8_t x, int8_t y, int8_t x2, int8_t y2, uint8_t brightness);
void drawCircle (int8_t xCenter, int8_t yCenter, int8_t radius, int8_t circleType, int8_t brightness);

/////////////// Global DATA structures /////////////////

// Stores pixels in two per byte packed.
Pixel frameBuffer[DISP_COLUMN_LENGTH][DISP_BYTES_LENGTH];

/*
 * Variables for the analogue conversion on ADC Sensors
 */
xADCArray values;         					// holds the return values from the ADC

/*Game variables*/
int8_t p1_y	=	DISP_COLUMN_LENGTH / 2;     //Player 1 initial paddle position
int8_t p2_y	=	DISP_COLUMN_LENGTH / 2;     //Player 2 initial paddle position
uint8_t paddle;								// Paddle length
uint8_t score_p1 = 0;      					//Player 1 score
uint8_t score_p2 = 0;      					//Player 2 score

/* Buzzer Variables */

volatile uint8_t buzzerFinished;	// flag: 0 while playing
const int8_t *buzzerSequence;
uint8_t buzzerInitialized;

///////////////////// Main program loop ////////////////
int main(void) __attribute__((OS_main));

int main(void)
{
	// Serial port just for testing. Remove or the game runs quite slow.
//	xSerialPort = xSerialPortInitMinimal( USART0, 115200, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)

    if( xADCSemaphore == NULL ) 					// Check to see if the ADC semaphore has not been created.
     {
    	xADCSemaphore = xSemaphoreCreateBinary();	// binary semaphore for ADC
 		if( ( xADCSemaphore ) != NULL )
 			xSemaphoreGive( ( xADCSemaphore ) );	// make the ADC available
     }

    xI2CQueue = xQueueCreate( 2, sizeof( uint8_t) );  // queue for i2c video buffer values

//	avrSerialPrint_P(PSTR("\r\nHello World!\r\n")); // Ok, so we're alive...

    xTaskCreate(
		TaskPong							// This will be the ball and scoring at 200Hz
		,  (const portCHAR *)"Pong"
		,  256				// Tested 13 free
		,  NULL
		,  3
		,  NULL ); // */

/*   xTaskCreate(
		TaskBall							// This is a bouncing ball (instead of Pong + SlowLoop)
		,  (const portCHAR *)"Ball"
		,  186				// Tested 5 free
		,  NULL
		,  3
		,  NULL ); // */

/*    xTaskCreate(
		TaskFace							// This is a digital face (instead of Pong + SlowLoop)
		,  (const portCHAR *)"Face"
		,  186				// Tested 20 free
		,  NULL
		,  3
		,  NULL ); // */

    xTaskCreate(
		TaskSlowLoop						// This is the borders, paddles, scoring & ADC running at 20Hz
		,  (const portCHAR *)"SlowLoop"
		,  186				// Tested 23 free
		,  NULL
		,  2
		,  NULL ); // */

    xTaskCreate(
        TaskWriteI2CVideo					// This writes to I2C on 200Hz frame rate
        ,  (const portCHAR *)"WriteI2CVideo"
        ,  240				// Tested x free
        ,  NULL
        ,  2
        ,  NULL ); // */

//	avrSerialxPrintf_P(&xSerialPort, PSTR("Free Heap Size: %u\r\n"), xPortGetFreeHeapSize() ); // needs heap_1,  heap_2 or heap_4 for this function to succeed.

	vTaskStartScheduler();

	// check whether the scheduler started properly. If there's no free RAM, then it won't start.
//	avrSerialxPrint_P(&xSerialPort, PSTR("\r\n\n\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...

}

static void TaskPong(void *pvParameters) // xxx Write to LED Buffer
{
    (void) pvParameters;

    TickType_t xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
 	xLastWakeTime = xTaskGetTickCount();


 	setDigitalInput(RESTART, HIGH_IMPEDANCE );   // initialise Button D10
 	setDigitalInput(SERVE,   HIGH_IMPEDANCE );   // initialise Button D11
 	setDigitalInput(PAUSE,   HIGH_IMPEDANCE );   // initialise Button D12


	float xOld = 0; // a few x & y position values
	float yOld = 0;

	float VxOld = 0; //  x & y velocity values
	float VyOld = 0;

	float xNew,  yNew;
	float VxNew, VyNew;

	// uint8_t for writing to the frame buffer
	uint8_t xp = 0;
	uint8_t yp = 0;
	uint8_t oldPixVal = 0;

	uint8_t hits = 0; 			//Used to increase the speed of the Ball

    uint8_t serve = false;		// serve the ball if it left the field.
	uint8_t initialise = true;	// start a new game.

//	playFromProgramSpace(guminam);		// play a little Guminam (or Bach) to kick things off.

// 	vTaskDelayUntil( &xLastWakeTime, ( 16000 / portTICK_PERIOD_MS ) ); // 16 sec pause.

    while(1)
    {

		if (! isDigitalInputHigh(RESTART) || initialise == true ) // New Game
		{


			playFromProgramSpace(cmajor);		// play a little scale
			while(isPlaying());

			srand((uint16_t)xLastWakeTime); // seed a random number

			// Initial position centre of the screen.
			xOld = (float) DISP_ROW_LENGTH / 2;
			yOld = (float) DISP_COLUMN_LENGTH / 2;


			//Random initial x-velocity. Serve either way.
			VxOld = ((float)rand() / ((float)RAND_MAX + 1) * 2*ACCEL - ACCEL); // Initial velocity: up to +/- ACCEL

			//Random initial y-velocity:
			VyOld = ((float)rand() / ((float)RAND_MAX + 1) * 2*g) - g;   // Initial velocity: up to +/- g.

			// make sure the initial horizontal velocity is greater than vertical velocity
			if (fabs(VxOld) < fabs(VyOld))
				VxOld = VyOld;


			score_p1 = 0;		//Player 1 score
			score_p2 = 0;		//Player 2 score

			hits	=	0; 		//Used to increase the speed of the Ball

			initialise = false;

		}

		if (! isDigitalInputHigh(SERVE) || serve == true) // SERVE
		{

			playFrequency( 523, 150); // starting tone

			// Initial position centre of the screen
			xOld = (float) DISP_ROW_LENGTH / 2;
			yOld = (float) DISP_COLUMN_LENGTH / 2;

			//Random initial x-velocity. Serve either way.
			VxOld = ((float)rand() / ((float)RAND_MAX + 1) * 2*ACCEL - ACCEL); // Initial velocity: up to +/- ACCEL

			//Random initial y-velocity:
			VyOld = ((float)rand() / ((float)RAND_MAX + 1) * 2*g) - g;   // Initial velocity: up to +/- g.

			// make sure the initial horizontal velocity is greater than vertical velocity
			if (fabs(VxOld) < fabs(VyOld))
				VxOld = VyOld;

			hits	=	0; 			//Used to increase the speed of the Ball

			serve = false;

		}

		if (! isDigitalInputHigh(PAUSE)) // PAUSE
		{
			vTaskDelayUntil( &xLastWakeTime, ( 20 / portTICK_PERIOD_MS ) ); // debounce
			while ( isDigitalInputHigh(PAUSE) )
				vTaskDelayUntil( &xLastWakeTime, ( 100 / portTICK_PERIOD_MS ) ); // wait for button press
			vTaskDelayUntil( &xLastWakeTime, ( 20 / portTICK_PERIOD_MS ) ); // debounce
		}



		// Physics time!
		//x' = x + v*t + at*t/2
		//v' = v + a*t

		//Horizontal (X) axis: acceleration; a = ACCEL.
		//Vertical (Y) axis: a = -g
		//

		xNew = xOld + VxOld;
		yNew = yOld + VyOld - 0.5*g*TIMESTEP*TIMESTEP;

		VyNew = VyOld - g*TIMESTEP;
		VxNew = VxOld;


		// Bounce at floor
		if (yNew < 0)
		{
			yNew = 0;
			if (VyNew < 0)
			{
				VyNew *= -BOUNCE;
				playFrequency( DIV_BY_10 | 654, 100);
			}
		}

		// Bounce at ceiling
		if (yNew >= 24)
		{
			yNew = 24;
			 if (VyNew > 0)
			 {
				VyNew *= -BOUNCE/2; // bounce softly off the ceiling
				playFrequency( 262, 50);
			 }
		}

		// Bounce at paddles, before off the ends.

		if( xNew < PLAYER1_X && yNew >= (p1_y - paddle) && yNew < (p1_y + paddle) )
		{
			xNew = PLAYER1_X;
			VxNew *= -1   * (1 + ACCEL*TIMESTEP*TIMESTEP * hits * 8 );
			VyNew *= -1.3 * (1 + ((float)rand() / ((float)RAND_MAX + 1) * ACCEL - ACCEL/2));		// Add a bit of random speed off the paddle; to counter bounce decay
			playFrequency( 440, 100 );
			hits++; 			//Used to increase the speed of the Ball each time we go around.
		}


		if( xNew >= PLAYER2_X && yNew >= (p2_y - paddle) && yNew < (p2_y + paddle) )
		{
			xNew = PLAYER2_X;
			VxNew *= -1   * (1 + ACCEL*TIMESTEP*TIMESTEP * hits * 8 );
			VyNew *= -1.3 * (1 + ((float)rand() / ((float)RAND_MAX + 1) * ACCEL - ACCEL/2));		// Add a bit of random speed off the paddle; to counter bounce decay
			playFrequency( 440, 100 );
			hits++; 			//Used to increase the speed of the Ball each time we go around.
		}


		// Score at the ends.
		if (xNew < 0 )
		{
			serve = true;
			score_p2++;
			xNew =  -2;
			yNew =  -2;
			VxNew =  0;
			VyNew =  0;
		}

		if (xNew >= 24 )
		{
			serve = true;
			score_p1++;
			xNew =  -2;
			yNew =  -2;
			VxNew =  0;
			VyNew =  0;
		}

		if (score_p1 > 8 || score_p2 > 8)
			initialise = true;

		//   *** Draw ball ***

		// set to the previous brightness to erase the ball, before we draw a new ball.
		drawPixel( xp, yp, oldPixVal );

		if( serve==false || initialise==false) // if we're not going to serve or initialise the game.
		{

			// figure out which point we're going to draw for the ball.
			xp =      (uint8_t) round(xNew);
			yp = 24 - (uint8_t) round(yNew);

			// get the current value, so we don't blank a wall, paddle or score pixel.
			oldPixVal = getPixel( xp, yp );

			// Write the point to the buffer
			drawPixel( xp, yp, MAX_BRIGHTNESS );

			// This step is optional, but signals to WriteI2CVideo for the row to be sent first.
			// Send the newly set row number.  Wait for 0 ticks for space to become
			// available before proceeding.
			xQueueSendToBack( xI2CQueue, &yp, ( TickType_t ) 0 );

		}

		//Age variables for the next iteration
		VxOld = VxNew;
		VyOld = VyNew;

		xOld = xNew;
		yOld = yNew;

        vTaskDelayUntil( &xLastWakeTime, ( 5 / portTICK_PERIOD_MS ) );

        // check the amount of RAM consumed by the task, with respect to the amount allocated at creation.
//		xSerialPrintf_P(PSTR("TaskPong HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));

	}
}

/*-----------------------------------------------------------*/



static void TaskBall(void *pvParameters) // xxx Write to LED Buffer
{
    (void) pvParameters;

    TickType_t xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
 	xLastWakeTime = xTaskGetTickCount();

	float xOld = 0; // a few x & y position values
	float yOld = 0;

	float VxOld = 0; //  x & y velocity values
	float VyOld = 0;

	float xNew, yNew;
	float VxNew, VyNew;

	uint8_t NewBall = 101; // make a new ball straight away ( > 20)

//	uint8_t ballRadius = 2;

	// uint8_t for writing to the frame buffer
	uint8_t xp = 0;
	uint8_t yp = 0;

    while(1)
    {

    	if ( NewBall > 20 ) // If ball has run out of energy, make a new ball!
    	{

    		// clear frame
    		for (uint8_t y = 0; y < DISP_COLUMN_LENGTH; y++)
    		{
    			for (uint8_t x = 0; x < DISP_ROW_LENGTH; x++)
    				drawPixel( x, y, 0 );
    		}

			NewBall = 0;

			//Clear history:
			xOld = -2;
			yOld = -2;

			xOld = ((float)rand() / ((float)RAND_MAX + 1) * 25) ;   // Initial position: up to 24.
			yOld = (float) 25;

			//Random initial x-velocity:

			VxOld = ((float)rand() / ((float)RAND_MAX + 1) * 2*g) - g;   // Initial velocity: up to +/- g.

			//Zero initial y velocity:
			VyOld = 0;
    	}

		// Physics time!
		//x' = x + v*t + at*t/2
		//v' = v + a*t

		//Horizontal (X) axis: No acceleration; a = 0.
		//Vertical (Y) axis: a = -g
		//

		xNew = xOld + VxOld;
		yNew = yOld + VyOld - 0.5*g*TIMESTEP*TIMESTEP;

		VyNew = VyOld - g*TIMESTEP;
		VxNew = VxOld;

		// Bounce at walls

		if (xNew < 0)
		{
			VxNew *= -1;
			xNew = 0;
		}

		if (xNew >= 24)
		{
			VxNew *= -1;
			xNew = 24;
		}

		if (yNew <= 0)
		{
			yNew = 0;

			if (VyNew*VyNew < 0.1)
				NewBall++;

			if (VyNew < 0)
				VyNew *= -BOUNCE;

		}

		if (yNew >= 25)
		{
			yNew = 25;

			 if (VyNew > 0)
				 VyNew = 0; // open top box
//				 VyNew *= -BOUNCE; // closed box

		}


		//   *** Draw buffer ***

		// clear frame
		for (uint8_t y = 0; y < DISP_COLUMN_LENGTH; y++)
		{
			for (uint8_t x = 0; x < DISP_ROW_LENGTH; x++)
				drawPixel( x, y, 0 );
		}


		//Note: leaves a light trail after the ball!
		// set to 0 brightness to erase the trail.
//		drawPixel( xp, yp, 4 );


		drawLine( 0, 24-24,  0, 24- 0, 2); // draw an open box for the ball to bounce in.
		drawLine( 0, 24- 0, 24, 24- 0, 2);
		drawLine(24, 24- 0, 24, 24-24, 2);

		//Next, figure out which point we're going to draw.

/*		ballRadius = (values.adc2 >> 6); // reduce the 8 bit value to 2 bits (4 values)

		xp =      (uint8_t) round(xNew) - ballRadius;
		yp = 24 - (uint8_t) round(yNew) - ballRadius;

				// Write the point to the buffer

		drawCircle( xp, yp, ballRadius, FULLCIRCLE, MAX_BRIGHTNESS ); // */


		xp =      (uint8_t) round(xNew);
		yp = 24 - (uint8_t) round(yNew);

		// Write the point to the buffer

		drawPixel( xp, yp, MAX_BRIGHTNESS ); // */

		// This step is unnecessary, but signals for the new row to be sent first.
		// Send the newly set row number.  Wait for 0 ticks for space to become
        // available if necessary.

		xQueueSendToBack( xI2CQueue, &yp, ( TickType_t ) 0 );


		//Age variables for the next iteration
		VxOld = VxNew;
		VyOld = VyNew;

		xOld = xNew;
		yOld = yNew;

        vTaskDelayUntil( &xLastWakeTime, ( 5 / portTICK_PERIOD_MS ) );

        // check the amount of RAM consumed by the task, with respect to the amount allocated at creation.
//		xSerialPrintf_P(PSTR("TaskBall HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));

	}
}

/*-----------------------------------------------------------*/


static void TaskFace(void *pvParameters) // xxx Write to LED Buffer
{
    (void) pvParameters;

    TickType_t xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
 	xLastWakeTime = xTaskGetTickCount();


    while(1)
    {


		shiftOut(DATA, CLOCK, LATCH, MSBFIRST, (0b01111111)); // turn on 7 Segment decimal point

    	for ( uint8_t y = 0; y < DISP_ROW_LENGTH; y++)
    	{
    		for ( uint8_t x = 0; x < DISP_ROW_LENGTH; x++)
    			drawPixel( x, y, pgm_read_byte_near(&Picture[ y * DISP_ROW_LENGTH + x ] ));
    			// XXX uncomment pgm_read_byte line & uncomment picture in header file
    	}

     	vTaskDelayUntil( &xLastWakeTime, ( 1000 / portTICK_PERIOD_MS ) ); // 1 sec pause.

		shiftOut(DATA, CLOCK, LATCH, MSBFIRST, (0b11111111)); // turn off 7 Segment decimal point

    	for ( uint8_t y = 0; y < DISP_ROW_LENGTH; y++)
    	{
    		for ( uint8_t x = 0; x < DISP_ROW_LENGTH; x++)
    			drawPixel( x, y, 0 );
    	} // */

        // check the amount of RAM consumed by the task, with respect to the amount allocated at creation.
//		xSerialxPrintf_P(&xSerialPort, PSTR("TaskFace HighWater @ %u "), uxTaskGetStackHighWaterMark(NULL));

        vTaskDelayUntil( &xLastWakeTime, ( 1000 / portTICK_PERIOD_MS ) ); // 1 sec pause

	}
}


/*-----------------------------------------------------------*/

static void TaskSlowLoop(void *pvParameters) // xxx Write to 7 Segment display (via shift register)
    {
    (void) pvParameters;

    TickType_t xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
 	xLastWakeTime = xTaskGetTickCount();

    uint8_t character;

    while(1)
	{
    	ReadADCSensors(); // use this slow loop 20Hz task to read ADC and write global values.

		// Get the locations of the paddles
		p1_y =   (values.adc0 >> 3) - 3; // reduce the 8 bit paddle location value to 5 bits (32 values) and centre
		p2_y =   (values.adc2 >> 3) - 3;
		paddle = (values.adc1 >> 6) + 1; // reduce the 8 bit paddle length value to 2 bits (4 values)

		if (p1_y > 24) p1_y = 24; // bounding paddle position to 0 to 24
		if (p1_y < 0 ) p1_y =  0;

		if (p2_y > 24) p2_y = 24;
		if (p2_y < 0 ) p2_y =  0;

		//   *** Draw buffer ***

		// clear frame
		for (uint8_t y = 1; y < DISP_COLUMN_LENGTH -1; y++) // optimised this, by only clearing active frame.
		{
			for (uint8_t x = 0; x < DISP_ROW_LENGTH; x++)
				drawPixel( x, y, 0 );
		}

		drawLine( 0, 24-24, 25, 24-24, 1); // draw top and bottom walls for the ball to bounce off.
		drawLine( 0, 24- 0, 25, 24- 0, 1);

		// draw paddles
		drawLine( PLAYER1_X, 24 - p1_y + paddle, PLAYER1_X, 24 - p1_y - paddle, 8); // Player 1 Paddle
		drawLine( PLAYER2_X, 24 - p2_y + paddle, PLAYER2_X, 24 - p2_y - paddle, 8); // Player 2 Paddle

		// draw scores
		drawLine( PLAYER1_X + 1, 24 - 22, PLAYER1_X + 1 + score_p1, 24 - 22, 4); // Player 1 Paddle
		drawLine( PLAYER2_X - 1, 24 - 22, PLAYER2_X - 1 - score_p2, 24 - 22, 4); // Player 2 Paddle

		character = pgm_read_byte_near(&ledCharSet[(values.adc3 >> 3)]); // retrieve the character from PROGMEM; only do this once.
		// For Photo resistor (just as a test) reduce the 8 bit value to 5 bits (32 values, 7 segment) code

		shiftOut(DATA,CLOCK,LATCH,MSBFIRST,~(character | 0b10000000)); // turn on decimal point
		vTaskDelayUntil( &xLastWakeTime, ( 10 / portTICK_PERIOD_MS ) );

//		xSerialxPrintf_P(&xSerialPort, PSTR("A0: %3u, A1: %3u, A2: %3u\r\n"), values.adc0, values.adc1, values.adc2 );

        // check the amount of RAM consumed by the task, with respect to the amount allocated at creation.
//		xSerialxPrintf_P(&xSerialPort, PSTR("Write Scoring HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
//		xSerialxPrintf_P(&xSerialPort, PSTR("Minimum Free Heap Size: %u\r\n"), xPortGetMinimumEverFreeHeapSize() ); // needs heap_4 for this function to succeed.

		shiftOut(DATA,CLOCK,LATCH,MSBFIRST,~(character & 0b01111111)); // turn off decimal point
		vTaskDelayUntil( &xLastWakeTime, ( 40 / portTICK_PERIOD_MS ) );
	}
}



/*-----------------------------------------------------------*/

static void TaskWriteI2CVideo(void *pvParameters) // xxx Write i2c Bus for video frame Pong or whatever
{
    (void) pvParameters;;

    TickType_t xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
 	xLastWakeTime = xTaskGetTickCount();

	// I2C master interface initialisation, should need to do this once only.
	I2C_Master_Initialise( (PONGVIDEO<<I2C_ADR_BITS) | (true<<I2C_GEN_BIT) );

    while(1)
    {


    	int8_t hotRowHandlerLimit = 0;

    	for ( uint8_t j = 0; j < DISP_COLUMN_LENGTH; j++ )
    	{

    		uint8_t hotRow;

			xVideoRowArray xVideoRow;     // Holds transmission values for the I2C bus transfer.

			// Check on the queue to see if a row has been updated, and check we haven't done this too often.
    		if( xI2CQueue != 0 && hotRowHandlerLimit < HOTROWLIMIT )
    		{
     			if( xQueueReceive( xI2CQueue, &hotRow, 0) == pdPASS )
     			{
					// move the hot row number into the structure for transmission.
					xVideoRow.RowNumber = hotRow;

					// move the frame buffer into the into the structure for transmission.
					for ( uint8_t i = 0; i < DISP_BYTES_LENGTH; i++)
						 xVideoRow.DoublePixels[i] = frameBuffer[hotRow][i].both;

					hotRowHandlerLimit++; // increment the number of times we've handled a hot row, without normal scanning.
					j--; // go back and do the intended j scan row again, next time round.
     			}

    		}
    		else
    		{

				// move the (normal scanning) row number into into the structure for transmission.
				xVideoRow.RowNumber = j;

				// move the frame buffer into the into the structure for transmission.
				for ( uint8_t i = 0; i < DISP_BYTES_LENGTH; i++)
					 xVideoRow.DoublePixels[i] = frameBuffer[j][i].both;

    			hotRowHandlerLimit--; // decrement the number of times we've handled a hot row, without normal scanning.
    		}

   			xVideoRow.I2CAddress = (PEGGYVIDEO<<I2C_ADR_BITS) + I2C_WRITE;

			// calculate a CRC on the row, excluding the address byte, (and CRC byte)
			xVideoRow.CRC8 = crc8( (uint8_t *) &xVideoRow.RowNumber, sizeof(xVideoRow.RowNumber) + sizeof(xVideoRow.DoublePixels));

			if ( I2C_Check_Free_After_Stop() == true )
				// right address, right data, right CRC8; so hit the I2C bus...
				I2C_Master_Start_Transceiver_With_Data( (uint8_t *)&xVideoRow, sizeof(xVideoRow) );

		}
    	vTaskDelayUntil( &xLastWakeTime, ( 5 / portTICK_PERIOD_MS ) ); // 200Hz Frame refresh rate.

        // check the amount of RAM consumed by the task, with respect to the amount allocated at creation.
//		xSerialPrintf_P(PSTR("TaskWriteI2CVideo HighWater2 @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
    }
}


/*-----------------------------------------------------------*/


static void ReadADCSensors(void)  // Read ADC Sensors
{

	if( xADCSemaphore != NULL )
	{
		// See if we can obtain the semaphore.  If the semaphore is not available
		// wait 10 ticks to see if it becomes free.
		if( xSemaphoreTake( xADCSemaphore, ( TickType_t ) 10 ) == pdTRUE )
		{
		// We were able to obtain the semaphore and can now access the
		// shared resource.
		// We want to have the ADC for us alone, as it takes some time to sample,
		// so we don't want it getting stolen during the middle of a conversion.

			setAnalogMode(MODE_8_BIT);    // 8-bit analogue-to-digital conversions

			startAnalogConversion(0, 0);   // start next conversion
			while( analogIsConverting() )
				_delay_us(25);     // yield until conversion ready

			values.adc0 = analogConversionResult();

			startAnalogConversion(1, 0);   // start next conversion
			while( analogIsConverting() )
				 _delay_us(25);       // yield until conversion ready

			values.adc1 = analogConversionResult();

			startAnalogConversion(2, 0);   // start next conversion
			while( analogIsConverting() )
				 _delay_us(25);      // yield until conversion ready

			 values.adc2 = analogConversionResult();

			startAnalogConversion(3, 0);   // start next conversion
			while( analogIsConverting() )
				 _delay_us(25);      // yield until conversion ready

			 values.adc3 = analogConversionResult();

			xSemaphoreGive( xADCSemaphore );

			return;
		}
	}
}

/*-----------------------------------------------------------*/

//   uint8_t character;

//	character = pgm_read_byte(&ledCharSet[(values.adc1 >> 3)]); // retrieve the character from PROGMEM; only do this once.
// reduce the 8 bit value to 5 bits (32 values, 7 segment) code

//	shiftOut(DATA,CLOCK,LATCH,MSBFIRST,~(character | 0b10000000)); // turn on decimal point

//	shiftOut(DATA,CLOCK,LATCH,MSBFIRST,~(character & 0b01111111)); // turn off decimal point


static void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t latchPin, uint8_t bitOrder, uint8_t bitVal)
{
  uint8_t i;
  setDigitalOutput(latchPin,LOW); // prepare the shift register storage to receive.
  for (i = 0; i < 8; i++)
  {
	  setDigitalOutput(clockPin, LOW);
	  if (bitOrder == LSBFIRST)
		  setDigitalOutput(dataPin, bitVal & (1 << i));
	  else
		  setDigitalOutput(dataPin, bitVal & (1 << (7 - i)));
	  _delay_us( 0.1 );  // needs 100ns.
	  setDigitalOutput(clockPin, HIGH); // move data along shift registers on +ve edge.
	  _delay_us( 0.1 );   // needs 100ns.
  }
  setDigitalOutput(latchPin,HIGH); // move the shift register values into store on +ve edge.
}




/**************************************************************************************
 * Return the brightness value of the pixel at x, y.
 *************************************************************************************/
uint8_t getPixel(int8_t x,int8_t y)
{
	if (x & 0x01)
		return (frameBuffer[y][x/2].odd);
	else
		return (frameBuffer[y][x/2].even);
}

/**************************************************************************************
 * Set the brightness of the pixel at x, y to 'brightness' (0=off).
 *************************************************************************************/
void drawPixel(uint8_t x,uint8_t y, uint8_t brightness)
{
	if (x & 0x01)
		frameBuffer[y][x/2].odd = brightness;
	else
		frameBuffer[y][x/2].even = brightness;
}

/*************************************************************************************/
/**************************************************************************************
 * Extremely Fast Line Algorithm Var C (Addition)
 * Copyright 2001-2, By Po-Han Lin
 *
 * Freely useable in non-commercial applications as long as credits
 * to Po-Han Lin and link to http://www.edepot.com is provided in source
 * code and can been seen in compiled executable.  Commercial
 * applications please inquire about licensing the algorithms.
 *
 * Lastest version at http://www.edepot.com/phl.html
 *************************************************************************************/
// The following places a reference to the line drawing algorithm
// author in the binary, as required by the copyright.
// const uint8_t __attribute__((progmem))  * LineDrawingCopyright  = "Po-Han Lin www.edepot.com";

/*----------------------------------------------------------
 * Draw a line from x,y to x2, y2 at value in the
 * arg brightness
 *--------------------------------------------------------*/
void drawLine(int8_t x, int8_t y, int8_t x2, int8_t y2, uint8_t brightness)
{
	uint8_t yLonger=0;
	int8_t incrementVal, endVal;

	int8_t shortLen=y2-y;
	int8_t longLen=x2-x;
	if (abs(shortLen)>abs(longLen))
	{
		int8_t swap=shortLen;
		shortLen=longLen;
		longLen=swap;
		yLonger=1;
	}

	endVal=longLen;
	if (longLen<0)
	{
		incrementVal=-1;
		longLen=-longLen;
	}
	else
		incrementVal=1;

	double decInc;
	if (longLen==0) decInc=(double)shortLen;
	else decInc=((double)shortLen/(double)longLen);
	double j=0.0;
	if (yLonger) {
		for (int8_t i=0;i!=endVal;i+=incrementVal) {
			drawPixel(x+(int8_t)j,y+i, brightness);
			j+=decInc;
		}
	} else {
		for (int8_t i=0;i!=endVal;i+=incrementVal) {
			drawPixel(x+i,y+(int8_t)j, brightness);
			j+=decInc;
		}
	}
}


/*----------------------------------------------------------
 * Draw a square from x,y to x2, y2 at value in the
 * arg brightness
 *--------------------------------------------------------*/
void drawSquare(int8_t x, int8_t y, int8_t x2, int8_t y2, uint8_t brightness)
{
	drawLine(x,y,x2,y2,brightness);
	drawLine(x2,y2,x2+(y-y2),y2+(x2-x),brightness);
	drawLine(x,y,x+(y-y2),y+(x2-x),brightness);
	drawLine(x+(y-y2),y+(x2-x),x2+(y-y2),y2+(x2-x),brightness);
}

/*----------------------------------------------------------
 * Draw a rectangle from x,y to x2, y2 at value in the
 * arg brightness
 *--------------------------------------------------------*/
void drawRect(int8_t x, int8_t y, int8_t x2, int8_t y2, uint8_t brightness)
{
	drawLine(x,y,x2,y,brightness);
	drawLine(x2,y,x2,y2,brightness);
	drawLine(x2,y2,x,y2,brightness);
	drawLine(x,y2,x,y,brightness);
}


/**************************************************************************************
 * End of code  Copyright 2001-2, By Po-Han Lin
 *************************************************************************************/


// *************************************************************************************************
// drawCircle
//
// Draws a full circle, half a circle, or a quarter of a circle
// Inputs:
// xCenter the x for the center of the circle
// yCenter the y for the center of the circle
// radius the radius for the circle
// color the color of the circle
// circleType the type of circle see PeggyVideo.h for types of circles
//
//  This routine can be used to draw a complete circle, half circle or a quarter of a circle
//
// Returns: Nothing
//
// I found the original code at http://www.arduino.cc/playground/Code/GLCDks0108 and modified it.
//
// *************************************************************************************************

void drawCircle (int8_t xCenter, int8_t yCenter, int8_t radius, int8_t circleType, int8_t brightness)
{
	int16_t tSwitch, x1 = 0, y1 = radius;
	int16_t Width = 2*radius, Height=Width;
	tSwitch = 3 - 2 * radius;

	while (x1 <= y1)
	{
		if (circleType == FULLCIRCLE||circleType == OPENSOUTH||circleType == OPENEAST||circleType == OPENSOUTHEAST)
		{
			drawPixel(xCenter+radius - x1, yCenter+radius - y1, brightness);
			drawPixel(xCenter+radius - y1, yCenter+radius - x1, brightness);
		}
		if (circleType == FULLCIRCLE||circleType == OPENNORTH||circleType == OPENEAST||circleType == OPENNORTHEAST)
		{
			drawPixel(xCenter+Width-radius + x1, yCenter+radius - y1, brightness);
			drawPixel(xCenter+Width-radius + y1, yCenter+radius - x1, brightness);
		}
		if (circleType == FULLCIRCLE||circleType == OPENNORTH||circleType == OPENWEST||circleType == OPENNORTHWEST)
		{
			drawPixel(xCenter+Width-radius + x1, yCenter+Height-radius + y1, brightness);
			drawPixel(xCenter+Width-radius + y1, yCenter+Height-radius + x1, brightness);
		}
		if (circleType == FULLCIRCLE||circleType == OPENSOUTH||circleType == OPENWEST||circleType == OPENSOUTHWEST)
		{
			drawPixel(xCenter+radius - x1, yCenter+Height-radius + y1, brightness);
			drawPixel(xCenter+radius - y1, yCenter+Height-radius + x1, brightness);
		}

		if (tSwitch < 0) tSwitch += (4 * x1 + 6);
		else
		{
			tSwitch += (4 * (x1 - y1) + 10);
			y1--;
		}
		x1++;
	}
}

/*-----------------------------------------------------------*/


void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    portCHAR *pcTaskName )
{
	DDRB  |= _BV(DDB5);
	PORTB |= _BV(PORTB5);       // main (red PB5) LED on. Arduino LED on and die.
	while(1);
}

/*-----------------------------------------------------------*/
