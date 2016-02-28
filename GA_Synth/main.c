////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////    main.c
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* ring buffer include file. */
#include "ringBuffer.h"

/* serial interface include file. */
#include "serial.h"

/* debouncer include file, */
#include "buttonDebounce.h"

/* SPI interface include file. */
#include "spi.h"

/* time interface include file */
#include "time.h"

/* extended string to integer */
#include "xatoi.h"

/* Gameduino 2 include file. */
#include "FT_Platform.h"

/* Goldilocks Analogue and other DAC functions include file. */
#include "DAC.h"

/* local inclusions */
#include "GASynth.h"

/*--------------Global Variables--------------------*/

/* Create a handle for the serial port. */
extern xComPortHandle xSerialPort;

/* Create a Semaphore binary flag for the ADC for the microphone. To ensure only single access. */
SemaphoreHandle_t xADCSemaphore;

// EEPROM to hold the current Time Zone.
int8_t EEMEM eeSavedTZ;				// the time zone (in hours) saved through reboots.

/* This is the structure for holding all synthesiser control parameters */
synth_t synth;

/* This is a structure for holding the IIR filter coefficients and state variables */
filter_t filter;

// EEPROM to save the current synth settings.
synth_t EEMEM synth_store;

uint8_t * delayDataPtr;			// pointer to the delay buffer data location
ringBuffer_t delayBuffer;		// ring buffer control structure for delay buffer

uint16_t ch_A_out; // storage for the sample values to be written to MCP4822 DAC
uint16_t ch_B_out;

uint8_t * LineBuffer = (void *)0;	// put line buffer for monitor on heap later with pvPortMalloc.


/*--------- PROGMEM Wave Look Up Tables --------------*/

// create note lookup tables
// PROGMEM stores the values in the program memory
// notes are calculated based on a LUT_SIZE of 4096, and are scaled by 127 (<<7).
//
uint32_t const concertNoteTable[NOTES * STOPS] PROGMEM =
{
	// This is based on Concert Tuning with A = 440Hz, with Equal Temperament tuning.
	// this file is STOPS * NOTES sized note lookup table of unsigned 32bit integers
	#include "ConcertNoteLUT.inc"
};

uint32_t const verdiNoteTable[NOTES * STOPS] PROGMEM =
{
	// this is based on Scientific (Verdi) Tuning with A = 4330Hz, with Just Intonation tuning.
	// this file is STOPS * NOTES sized note lookup table of unsigned 32bit integers
	#include "VerdiNoteLUT.inc"
};

// create square wave lookup table
// PROGMEM stores the values in the program memory
int16_t const squareWave[LUT_SIZE] PROGMEM =
{
	// this file is a 4096 value square wave lookup table of signed 16bit integers
	#include "SquareLUT.inc"
};

// create saw tooth wave lookup table
// PROGMEM stores the values in the program memory
int16_t const sawWave[LUT_SIZE] PROGMEM =
{
	// this file is a 4096 value saw wave lookup table of signed 16bit integers
	#include "SawLUT.inc"
};

// create triangle wave lookup table
// PROGMEM stores the values in the program memory
int16_t const triangleWave[LUT_SIZE] PROGMEM =
{
	// this file is a 4096 value triangle wave lookup table of signed 16bit integers
	#include "TriangleLUT.inc"
};

// create sine wave lookup table
// PROGMEM stores the values in the program memory
int16_t const sineWave[LUT_SIZE] PROGMEM =
{
	// this file is a 4096 value sine wave lookup table of signed 16bit integers
	#include "SineLUT.inc"
};

// create exponential (1-e^-x) frequency lookup table
// use this to create a simple attack and release for notes.
// PROGMEM stores the values in the program memory
uint16_t const expTable[] PROGMEM = {
  // this file is a 2048 value exponential lookup table of unsigned 16bit integers
  // you can replace it with your own table if you like.
  #include "expTable.inc"
};

/*--------------Functions---------------------------*/

/* Main program loop */
int main(void)
{

    // turn on the serial port for setting or querying the time .
	xSerialPort = xSerialPortInitMinimal( USART0, 38400, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)

    // Semaphores are useful to stop a thread proceeding, where it should be stopped because it is using a resource.
    if( xADCSemaphore == (void *)0 ) 					// Check to see if the ADC semaphore has not been created.
    {
    	xADCSemaphore = xSemaphoreCreateBinary();	// binary semaphore for ADC for the microphone
		if( ( xADCSemaphore ) != (void *)0 )
			xSemaphoreGive( ( xADCSemaphore ) );	// make the ADC available by giving the semaphore
    }

	eeprom_busy_wait();
	set_zone( (int32_t)eeprom_read_byte((const uint8_t *)&eeSavedTZ) * ONE_HOUR ); // The Time Zone that we saved previously.

	avrSerialxPrint_P(&xSerialPort, PSTR("\r\nHello World!\r\n")); // Ok, so we're alive...

    xTaskCreate(
		TaskWriteLCD
		,  (const portCHAR *)"WriteLCD"
		,  400
		,  NULL
		,  3
		,  NULL ); // */

   xTaskCreate(
		TaskMonitor
		,  (const portCHAR *)"SerialMonitor"
		,  480
		,  NULL
		,  2
		,  NULL ); // */

   xTaskCreate(
		TaskAnalogue
		,  (const portCHAR *) "Analogue"
		,  96
		,  NULL
		,  1
		,  NULL ); // */

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

    TickType_t xLastWakeTime __attribute__ ((unused));
	/* The xLastWakeTime variable needs to be initialised with the current tick
	count.  Note that this is the only time we access this variable.  From this
	point on xLastWakeTime is managed automatically by the vTaskDelayUntil()
	API function. */
	xLastWakeTime = xTaskGetTickCount();

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
			xSerialPrint_P( PSTR(    "s  - show minimum ever heap size") );
			xSerialPrint_P( PSTR("\r\nh  - show this message") );
			xSerialPrint_P( PSTR("\r\nb  - reboot ft800 device") );
			xSerialPrint_P( PSTR("\r\nt  - set / show the time: t [<year yyyy> <month mm> <date dd> <hour hh> <minute mm> <second ss>]") );
			xSerialPrint_P( PSTR("\r\nz  - set the time zone +-hours (before you first set the time): z [<timezone zz>] \r\n") );
			break;

		case 's' : // reset
			xSerialPrintf_P(PSTR(    "Minimum Free Heap Size: %u"), xPortGetMinimumEverFreeHeapSize() ); // needs heap_1, heap_2 or heap_4 for this function to succeed.
			xSerialPrintf_P(PSTR("\r\nCurrent Free Heap Size: %u"), xPortGetFreeHeapSize() ); // needs heap_1, heap_2 or heap_4 for this function to succeed.
			xSerialPrintf_P(PSTR("\r\nSerial Monitor: Stack HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
			break;

		case 'b' : // reboot
			FT_API_Boot_Config();
			FT_API_Touch_Config();
			FT_touchTrackInit();
			FT_GUI();
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
				eeprom_update_byte( (uint8_t *)&eeSavedTZ, (int8_t)p1 );
				xSerialPrintf_P(PSTR("\r\nInput Time Zone %i"), (int8_t)p1 );
			}
			eeprom_busy_wait();
			xSerialPrintf_P(PSTR("\r\nSaved Time Zone %i"), (int8_t)eeprom_read_byte((uint8_t const *)&eeSavedTZ) );

			time(&timestamp);
			ctime_r( (time_t *)&timestamp, (char *)LineBuffer );
			xSerialPrintf_P(PSTR("\r\nLocal Time: %s - %u\r\n"), LineBuffer, timestamp );
			break;

		default :
			break;

		}
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

	// set up ADC sampling on the ADC0, ADC1 using MIDI Shield to control potentiometers.
	AudioCodec_ADC_init();

	// On Port D initialise the three buttons PD7, PD6, and PD5 on the MIDI Shield, pulled up.
	shieldDButtonInit( &portd, BUTTON_MASK, BUTTON_MASK );

	FT_API_Boot_Config();
	FT_API_Touch_Config();

	FT_touchTrackInit();
	FT_GUI();

    for(;;)
    {
    	uint8_t touched;
    	uint8_t physicalIO;

    	touched = FT_touch();
		physicalIO = shieldPhysicalIO(buttonCurrent( &portd, BUTTON_MASK ));

    	if( touched || physicalIO )
    		FT_GUI();

//		xSerialPrintf_P(PSTR("\r\nWriteLCD: Stack HighWater @ %u"), uxTaskGetStackHighWaterMark(NULL));
//		xSerialPrintf_P(PSTR("\r\nMinimum Ever Heap Free: %u\r\n"), xPortGetMinimumEverFreeHeapSize() ); // needs heap_1, heap_2 or heap_4 for this function to succeed.
		vTaskDelayUntil( &xLastWakeTime, 128 / portTICK_PERIOD_MS );
	}
}


static void TaskAnalogue(void *pvParameters) // Prepare the DAC
{
	(void) pvParameters;

	/* Create the ring-buffers used by audio delay loop, and initialise the control structure. */
	if(delayDataPtr == (void *)0) // if there is no delay buffer allocated (pointer is NULL), then allocate buffer.
	{
		if( (delayDataPtr = (uint8_t *)pvPortMalloc( sizeof(int16_t) * DELAY_BUFFER)))
			ringBuffer_InitBuffer( &delayBuffer, delayDataPtr, sizeof(int16_t) * DELAY_BUFFER);
		else
			xSerialPrint_P(PSTR("pvPortMalloc for *delayDataPtr fail..!\r\n"));
	}

//	xSerialPrintf_P(PSTR("\r\nDAC_Codec_init:"));
	DAC_init();
//	xSerialPrintf_P(PSTR(" will soon"));

	/* Initialise the sample interrupt timer. Exact multiples of 2000Hz are ok with 8 bit Timer0, otherwise use 16 bit Timer1 */
	AudioCodec_Timer0_init(SAMPLE_RATE);	// xxx set up the sampling Timer0 to 48000Hz (or lower), runs at audio sampling rate in Hz.
//	AudioCodec_Timer1_init(SAMPLE_RATE);	// xxx set up the sampling Timer1 to 44100Hz (or odd rates), runs at audio sampling rate in Hz.
//	xSerialPrintf_P(PSTR(" be"));

	AudioCodec_setHandler( synthesizer, &ch_A_out, &ch_B_out );		// Set the call back function to do the audio processing.
																	//	Done this way so that we can change the audio handling depending on what we want to achieve.

//	xSerialPrintf_P(PSTR(" done."));

//	xSerialPrintf_P(PSTR("\r\nFree Heap Size: %u"),xPortGetMinimumEverFreeHeapSize() ); // needs heap_1.c, heap_2.c or heap_4.c
//	xSerialPrintf_P(PSTR("\r\nAudio HighWater: %u\r\n"), uxTaskGetStackHighWaterMark(NULL));

	vTaskSuspend(NULL);						// Well, we're pretty much done here.
//	vTaskEndScheduler();					// Rely on Timer0/1 Interrupt for regular output.

	for(;;);
}

/*-----------------------------------------------------------*/
/* static functions */
/*-----------------------------------------------------------*/

void synthesizer( uint16_t * ch_A,  uint16_t * ch_B) // Voltage controlled oscillator
{
	// create some temporary variables
	uint16_t currentPhase;
	uint8_t frac;

	DAC_value_t temp0; // this is a int16_t that can be called as either byte.

	int16_t temp1 = 0;
	int16_t temp2 = 0;
	int16_t temp3 = 0;

	uint16_t buffCount;

	int16_t outXMOD;
	int16_t outVCO1;
	int16_t outVCO2;
	int16_t outLFO;

	// create a variable frequency and amplitude wave of size.
	// since we will be moving through the lookup table with 4096 values
	// at a variable frequency, we won't always land directly
	// on a single sample.  so we will average between the
	// two samples closest to us. This is called interpolation.
	// step through the table at rate determined by phase_increment
	// use upper byte of phase_increment value to set the rate

	// This is built to use 4096 sample LUTs.

	// The phase and the phase increment are in 24.8 fixed format.
	// That is, the lower 8 bits are assumed to be fractional.
	// They are used for the interpolation process, and to ensure accuracy.

	// Remember our DAC only has 12 bits, so we have 4 LSB spare the low end too.

	// only play if we're in the adsr envelope.
	if(synth.adsr != off)
	{
		////////////// First do the LFO ///////////////

		// This will later modulate by the VCO1 and VCO2 phase,
		// so we need it first.
		if( synth.lfo.toggle )
		{
			// increment the phase (index into LUT) by the calculated phase increment.
			synth.lfo.phase += synth.lfo.phase_increment;

			// if we've gone over the LUT boundary -> loop back
			synth.lfo.phase &= 0x000fffff; // this is a faster way doing the table
											// wrap around, which is possible
											// because our table is a multiple of 2^n.
											// Remember the lowest 0xff are fractions of LUT steps.

			currentPhase = (uint16_t)(synth.lfo.phase >> 8);

			// get first sample from the LUT and store it in temp1
			temp1 = pgm_read_word(synth.lfo.wave_table_ptr + currentPhase);

			++currentPhase; // go to next sample
			currentPhase &= 0x0fff;	// check if we've gone over the boundary.
									// we can do this because it is a multiple of 2^n.

			// get second sample from the LUT and put it in temp2
			temp2 = pgm_read_word(synth.lfo.wave_table_ptr + currentPhase);

			// interpolate between samples
			// multiply each sample by the fractional distance
			// to the actual location value
			frac = (uint8_t)(synth.lfo.phase & 0x000000ff); // fetch the lower 8b
			MultiSU16X8toH16Round(temp3, temp2, frac);
			// scaled sample 2 is now in temp3, and since we are done with
			// temp2, we can reuse it for the next result
			MultiSU16X8toH16Round(temp2, temp1, 0xff - frac);
			// temp2 now has the scaled sample 1
			temp2 += temp3; // add samples together to get an average
			// our resultant wave is now in temp2

			// set amplitude with volume
			// multiply our wave by the volume value
			MultiSU16X16toH16Round(outLFO, temp2, synth.lfo.volume);
			// our LFO wave is now in outLFO
		}
		else // LFO is turned off.
			outLFO = 0;

		////////////// Now do the VCO2 ///////////////

		// This will later modulate the VCO1 phase (depending on the XMOD intensity),
		// so we need it first.
		if( synth.vco2.toggle )
		{
			// increment the phase (index into LUT) by the calculated phase increment.
			synth.vco2.phase += synth.vco2.phase_increment;

			// calculate how much the LFO affects the VCO2 phase increment
			if (synth.lfo.toggle)
			{
				// increment the phase (index into LUT) by the calculated phase increment including the LFO output.
				synth.vco2.phase += outLFO; // increment on the fractional component 8.8.
			}

			// if we've gone over the LUT boundary -> loop back
			synth.vco2.phase &= 0x000fffff; // this is a faster way doing the table
											// wrap around, which is possible
											// because our table is a multiple of 2^n.
											// Remember the lowest 0xff are fractions of LUT steps.

			currentPhase = (uint16_t)(synth.vco2.phase >> 8);

			// get first sample from the LUT and store it in temp1
			temp1 = pgm_read_word(synth.vco2.wave_table_ptr + currentPhase);

			++currentPhase; // go to next sample
			currentPhase &= 0x0fff;	// check if we've gone over the boundary.
									// we can do this because it is a multiple of 2^n.

			// get second sample from the LUT and put it in temp2
			temp2 = pgm_read_word(synth.vco2.wave_table_ptr + currentPhase);

			// interpolate between samples
			// multiply each sample by the fractional distance
			// to the actual location value
			frac = (uint8_t)(synth.vco2.phase & 0x000000ff); // fetch the lower 8b
			MultiSU16X8toH16Round(temp3, temp2, frac);
			// scaled sample 2 is now in temp3, and since we are done with
			// temp2, we can reuse it for the next result
			MultiSU16X8toH16Round(temp2, temp1, 0xff - frac);
			// temp2 now has the scaled sample 1
			temp2 += temp3; // add samples together to get an average
			// our resultant wave is now in temp2

			// set amplitude with volume
			// multiply our wave by the volume value
			MultiSU16X16toH16Round(outVCO2, temp2, synth.vco2.volume);
			// our VCO2 wave is now in outVCO2

			// And now calculate the XMOD intensity to apply to the VCO1
			MultiSU16X16toH16Round(outXMOD, temp2, synth.xmod);
		}
		else // VCO2 is turned off.
			outXMOD = outVCO2 = 0;

		///////////// Now do the VCO1 ////////////////////

		// This will be modulated by the VCO2 value (depending on the XMOD intensity).
		if( synth.vco1.toggle )
		{

			// increment the phase (index into LUT) by the calculated phase increment.
			synth.vco1.phase += synth.vco1.phase_increment;

			// calculate how much the LFO affects the VCO1 phase increment
			if (synth.lfo.toggle)
			{
				// increment the phase (index into LUT) by the calculated phase increment including the LFO output.
				synth.vco1.phase += (uint32_t)outLFO; // increment on the fractional component 8.8.
			}

			// calculate how much the VCO2 XMOD affects the VCO1 phase increment
			if (synth.vco2.toggle)
			{
				// increment the phase (index into LUT) by the calculated phase increment including the LFO output.
				synth.vco1.phase += (uint32_t)outXMOD; // increment on the fractional component 8.8.
			}

			// if we've gone over the LUT boundary -> loop back
			synth.vco1.phase &= 0x000fffff; // this is a faster way doing the table
											// wrap around, which is possible
											// because our table is a multiple of 2^n.
											// Remember the lowest 0xff are fractions of LUT steps.

			currentPhase = (uint16_t)(synth.vco1.phase >> 8);

			// get first sample from the LUT and store it in temp1
			temp1 = pgm_read_word(synth.vco1.wave_table_ptr + currentPhase);

			++currentPhase; // go to next sample
			currentPhase &= 0x0fff;	// check if we've gone over the boundary.
									// we can do this because it is a multiple of 2^n.

			// get second sample from the LUT and put it in temp2
			temp2 = pgm_read_word(synth.vco1.wave_table_ptr + currentPhase);

			// interpolate between samples
			// multiply each sample by the fractional distance
			// to the actual location value
			frac = (uint8_t)(synth.vco1.phase & 0x000000ff); // fetch the lower 8b
			MultiSU16X8toH16Round(temp3, temp2, frac);
			// scaled sample 2 is now in temp3, and since we are done with
			// temp2, we can reuse it for the next result
			MultiSU16X8toH16Round(temp2, temp1, 0xff - frac);
			// temp2 now has the scaled sample 1
			temp2 += temp3; // add samples together to get an average
			// our resultant wave is now in temp2

			// set amplitude with volume
			// multiply our wave by the volume value
			MultiSU16X16toH16Round(outVCO1, temp2, synth.vco1.volume);
			// our VCO1 wave is now in outVCO1
		}
		else // VCO1 is turned off;
			outVCO1 = 0;
	}
	else // we're in off state, and no notes are playing.
		outVCO1 = outVCO2 = 0;


	////////////// mix the two oscillators //////////////////

	// irrespective of whether a note is playing or not.
	// combine the outputs
	temp2 = (outVCO1 >> 1) + (outVCO2 >>1);


	///////////////// calculate the adsr /////////////////////

	switch (synth.adsr)
	{
	case off: // wait for a note to be played
		if ( synth.note == FT_FALSE )
		{	// if there is no note being played, then reset the VCO increments.
			synth.vco1.phase = \
			synth.vco2.phase = \
			synth.lfo.phase  = \
			temp1 			 = 0x00;
		}
		else
		{	// set the adsr to attack and start producing sounds
			synth.adsr = attack;
			synth.adsr_phase = 0x00;
		}
		break;

	case attack:
		currentPhase = pgm_read_word(synth.adsr_table_ptr + synth.adsr_phase);
		MultiSU16X16toH16Round( temp1, temp2, currentPhase );

		if ( ++synth.adsr_phase > 0x07ff ) // attack state is for 2047 samples.
		{
			synth.adsr = decay;
			synth.adsr_phase = 0x0000;
		}
		break;

	case decay:
		temp1 = temp2;
		if (synth.note == FT_FALSE)
		{
			synth.adsr = release;
			synth.adsr_phase = 0x0000;
		}
		else
		{
			synth.adsr = sustain;
			synth.adsr_phase = 0x0000;
		}
		break;

	case sustain:
		temp1 = temp2;
		if ( synth.note == FT_FALSE )
		{
			synth.adsr = release;
			synth.adsr_phase = 0x0000;
		}
		break;

	case release:
		currentPhase = pgm_read_word(synth.adsr_table_ptr + synth.adsr_phase);
		MultiSU16X16toH16Round( temp1, temp2, UINT16_MAX - currentPhase );

		if ( ++synth.adsr_phase > 0x07ff) // release state is for 2047 samples.
		{
			synth.adsr = off;
			synth.adsr_phase = 0x0000;
		}
		else if ( synth.note != FT_FALSE )
		{
			synth.adsr = attack;
			synth.adsr_phase = 0x0000;
		}
		break;
	}


	////////////////// do the IIR LPF ///////////////////////

	IIRFilter( &filter, &temp1 );

	/////////// now do the space delay function /////////////

	// Get the number of buffer items we have, which is the delay.
	MultiU16X16toH16Round( buffCount, (uint16_t)(sizeof(int16_t) * DELAY_BUFFER), synth.delay_time);

	// Get a sample back from the delay buffer, some time later,
	if( ringBuffer_GetCount(&delayBuffer) >= buffCount )
	{
		temp0.u8[1] = ringBuffer_Pop(&delayBuffer);
		temp0.u8[0] = ringBuffer_Pop(&delayBuffer);
	}
	else // or else wait until we have samples available.
	{
		temp0.i16 = 0;
	}

	if (synth.delay_time) // If the delay time is set to be non zero,
	{
		// do the space delay function, irrespective of whether a note is playing or not,
		// and combine the output sample with the delayed sample.
		temp1 += temp0.i16;

		// multiply our sample by the feedback value
		MultiSU16X16toH16Round(temp0.i16, temp1, synth.delay_feedback);
	}
	else
		ringBuffer_Flush(&delayBuffer);	// otherwise flush the buffer if the delay is set to zero.

	// and push it into the delay buffer if buffer space is available
	if( ringBuffer_GetCount(&delayBuffer) <= buffCount )
	{
		ringBuffer_Poke(&delayBuffer, temp0.u8[1]);
		ringBuffer_Poke(&delayBuffer, temp0.u8[0]);
	}
	// else drop the space delay sample (probably because the delay has been reduced).

	////////////// Finally, set the output volume //////////////////

	// multiply our wave by the volume value
	MultiSU16X16toH16Round(temp2, temp1, synth.master);

	// and output wave on both A & B channel, shifted to (+)ve values only because this is what the DAC needs.
	*ch_A = *ch_B = temp2 + 0x7fff;

	// sample the MIDI Shield potentiometers to provide a physical control interface.
	AudioCodec_ADC(&mod0Value, &mod1Value);

	// check to see if any MIDI Shield buttons have been pushed, and fill the debounce structure, for a physical control interface.
	buttonProcess( &portd, (PIND & BUTTON_MASK) );
}

void FT_GUI()
{

	FT_GPU_CoCmd_Dlstart(phost);
	FT_API_Write_CoCmd(CLEAR_COLOR_RGB(0,0,0));
	FT_API_Write_CoCmd(CLEAR(1,1,1));

	FT_API_Write_CoCmd(SAVE_CONTEXT());
	FT_API_Write_CoCmd(COLOR_RGB(255,255,255));
	FT_GPU_CoCmd_Text_P(phost,   4,  8, 27, OPT_CENTERY, PSTR("VCO 1"));
	FT_GPU_CoCmd_Text_P(phost,   4,100, 27, OPT_CENTERY, PSTR("VCO 2"));
	FT_GPU_CoCmd_Text_P(phost,   4,194, 27, OPT_CENTERY, PSTR("LFO"));
	FT_GPU_CoCmd_Text_P(phost, 103, 18, 26, OPT_CENTER, PSTR("OCTAVE"));
	FT_GPU_CoCmd_Text_P(phost, 103,111, 26, OPT_CENTER, PSTR("PITCH"));
	FT_GPU_CoCmd_Text_P(phost, 103,204, 26, OPT_CENTER, PSTR("PITCH"));

	FT_GPU_CoCmd_Text_P(phost, 203,  8, 27, OPT_CENTER, PSTR("MIXER"));
	FT_GPU_CoCmd_Text_P(phost, 170, 25, 26, OPT_CENTER, PSTR("VCO 1"));
	FT_GPU_CoCmd_Text_P(phost, 235, 25, 26, OPT_CENTER, PSTR("VCO 2"));
	FT_GPU_CoCmd_Text_P(phost, 170, 95, 26, OPT_CENTER, PSTR("LFO"));
	FT_GPU_CoCmd_Text_P(phost, 235, 95, 26, OPT_CENTER, PSTR("X MOD"));

	FT_GPU_CoCmd_Text_P(phost, 300,  8, 27, OPT_CENTER, PSTR("VCF"));
	FT_GPU_CoCmd_Text_P(phost, 300, 25, 26, OPT_CENTER, PSTR("CUTOFF"));
	FT_GPU_CoCmd_Text_P(phost, 300, 95, 26, OPT_CENTER, PSTR("PEAK"));

	FT_GPU_CoCmd_Text_P(phost, 365,  8, 27, OPT_CENTER, PSTR("DELAY"));
	FT_GPU_CoCmd_Text_P(phost, 365, 25, 26, OPT_CENTER, PSTR("TIME"));
	FT_GPU_CoCmd_Text_P(phost, 365, 95, 26, OPT_CENTER, PSTR("FEEDBACK"));

	FT_GPU_CoCmd_Text_P(phost, 440,  8, 27, OPT_CENTER, PSTR("MASTER"));

	/* Now we have active widgets, so turn on the touch mask */
	FT_API_Write_CoCmd(TAG_MASK(FT_TRUE));		// turn on the TAG_MASK Because these things have touch

	/* Display the Toggles */

	FT_API_Write_CoCmd(COLOR_RGB(255,255,255));
	FT_GPU_CoCmd_FgColor(phost, 0xff0000);
	FT_GPU_CoCmd_BgColor(phost, 0x1a1a1a);

	FT_API_Write_CoCmd(TAG(VCO1_TOGGLE));
	FT_GPU_CoCmd_Toggle_P(phost, 13,26,46,18, OPT_3D, synth.vco1.toggle, PSTR("OFF" "\xFF" "VCO 1"));

	FT_GPU_CoCmd_FgColor(phost, 0x0000ff);

	FT_API_Write_CoCmd(TAG(VCO2_TOGGLE));
	FT_GPU_CoCmd_Toggle_P(phost, 13,119,46,18, OPT_3D, synth.vco2.toggle, PSTR("OFF" "\xFF" "VCO 2"));

	FT_GPU_CoCmd_FgColor(phost, 0x00ff00);

	FT_API_Write_CoCmd(TAG(LFO_TOGGLE));
	FT_GPU_CoCmd_Toggle_P(phost, 13,212,46,18, OPT_3D, synth.lfo.toggle, PSTR( "OFF" "\xFF" "LFO"));

	FT_GPU_CoCmd_FgColor(phost, 0xfffae0);

	FT_API_Write_CoCmd(TAG(VCO1_WAVE));
	FT_GPU_CoCmd_Toggle_P(phost, 13,56,46,18, OPT_3D, synth.vco1.wave, PSTR("SQR" "\xFF" "SIN"));

	FT_API_Write_CoCmd(TAG(VCO2_WAVE));
	FT_GPU_CoCmd_Toggle_P(phost, 13,150,46,18, OPT_3D, synth.vco2.wave, PSTR("TRI" "\xFF" "SAW"));

	FT_API_Write_CoCmd(TAG(LFO_WAVE));
	FT_GPU_CoCmd_Toggle_P(phost, 13,242,46,18, OPT_3D, synth.lfo.wave, PSTR("SIN" "\xFF" "TRI"));

	/* and the final toggle to get to the alternate tuning range (concert vs verdi) */

	FT_API_Write_CoCmd(TAG(KBD_TOGGLE));
	FT_GPU_CoCmd_Toggle_P(phost, 405,130,60,26, OPT_3D, synth.kbd_toggle, PSTR("CONCRT" "\xFF" "VERDI"));


	/* Display the Dials */

	FT_GPU_CoCmd_FgColor(phost, 0x1a1a1a);

	FT_API_Write_CoCmd(COLOR_RGB(255,0,0));

	FT_API_Write_CoCmd(TAG(VCO1_PITCH));
	FT_GPU_CoCmd_Dial(phost, 103,50,22, OPT_3D, synth.vco1.pitch); // VCO 1 Pitch

	FT_API_Write_CoCmd(COLOR_RGB(0,0,255));

	FT_API_Write_CoCmd(TAG(VCO2_PITCH));
	FT_GPU_CoCmd_Dial(phost, 103,145,22, OPT_3D, synth.vco2.pitch); // VCO 2 Pitch

	FT_API_Write_CoCmd(COLOR_RGB(0,255,0));

	FT_API_Write_CoCmd(TAG(LFO_PITCH));
	FT_GPU_CoCmd_Dial(phost, 103,235,22, OPT_3D, synth.lfo.pitch); // LFO Pitch

	FT_API_Write_CoCmd(COLOR_RGB(255,0,0));

	FT_API_Write_CoCmd(TAG(MIXER_VCO1));
	FT_GPU_CoCmd_Dial(phost, 170,55,20, OPT_3D, synth.vco1.volume); // MIXER VCO 1

	FT_API_Write_CoCmd(COLOR_RGB(0,0,255));

	FT_API_Write_CoCmd(TAG(MIXER_VCO2));
	FT_GPU_CoCmd_Dial(phost, 235,55,20, OPT_3D, synth.vco2.volume); // MIXER VCO 2

	FT_API_Write_CoCmd(COLOR_RGB(0,255,0));

	FT_API_Write_CoCmd(TAG(MIXER_LFO));
	FT_GPU_CoCmd_Dial(phost, 170,125,20, OPT_3D, synth.lfo.volume); // MIXER LFO

	FT_API_Write_CoCmd(COLOR_RGB(255,0,255));

	FT_API_Write_CoCmd(TAG(MIXER_XMOD));
	FT_GPU_CoCmd_Dial(phost, 235,125,20, OPT_3D, synth.xmod); // MIXER XMOD

	FT_API_Write_CoCmd(COLOR_RGB(255,250,224));
	FT_GPU_CoCmd_FgColor(phost, 0x1a1a1a);

	FT_API_Write_CoCmd(TAG(VCF_CUTOFF));
	FT_GPU_CoCmd_Dial(phost, 300,55,20, OPT_3D, synth.vcf_cutoff); // VCF CUTOFF

	FT_API_Write_CoCmd(TAG(VCF_PEAK));
	FT_GPU_CoCmd_Dial(phost, 300,125,20, OPT_3D, synth.vcf_peak); // VCF PEAK

	FT_API_Write_CoCmd(TAG(DELAY_TIME));
	FT_GPU_CoCmd_Dial(phost, 365,55,20, OPT_3D, synth.delay_time); // DELAY TIME

	FT_API_Write_CoCmd(TAG(DELAY_FEEDBACK));
	FT_GPU_CoCmd_Dial(phost, 365,125,20, OPT_3D, synth.delay_feedback); // DELAY FEEDBACK

	FT_API_Write_CoCmd(TAG(MASTER));
	FT_GPU_CoCmd_Dial(phost, 440,55,26, OPT_3D, synth.master); // MASTER


	/* Display the Keyboard */

	FT_API_Write_CoCmd(COLOR_RGB(0xff,0xfa,0xe0));
	FT_GPU_CoCmd_FgColor(phost, 0xfffae0);
	FT_GPU_CoCmd_GradColor(phost, 0x1a1a1a);

	// no need to write touch tags for keys, because the TAG is set to the ASCII code for the key.
	FT_GPU_CoCmd_Keys_P(phost, 137,160,340,110, 27, synth.note | OPT_3D, PSTR("CDEFGAB"));

	FT_API_Write_CoCmd(COLOR_RGB(0x1f,0x1f,0x1f));
	FT_GPU_CoCmd_FgColor(phost, 0x1a1a1a);
	FT_GPU_CoCmd_GradColor(phost, 0xffffff);

	// no need to write touch tags for keys, because the TAG is set to the ASCII code for the key.
	FT_GPU_CoCmd_Keys_P(phost, 169,160,30,45, 27, synth.note | OPT_3D, PSTR("c"));
	FT_GPU_CoCmd_Keys_P(phost, 219,160,30,45, 27, synth.note | OPT_3D, PSTR("d"));
	FT_GPU_CoCmd_Keys_P(phost, 316,160,30,45, 27, synth.note | OPT_3D, PSTR("f"));
	FT_GPU_CoCmd_Keys_P(phost, 365,160,30,45, 27, synth.note | OPT_3D, PSTR("g"));
	FT_GPU_CoCmd_Keys_P(phost, 414,160,30,45, 27, synth.note | OPT_3D, PSTR("a"));

	/* Display a Button */

	FT_API_Write_CoCmd(COLOR_RGB(0x1a,0x1a,0x1a));
	FT_API_Write_CoCmd(TAG(SETTINGS));
	if (synth.settings_loaded)
		FT_GPU_CoCmd_Button_P(phost, 415,95, 50,20, 26, OPT_3D, PSTR("STO"));
	else
		FT_GPU_CoCmd_Button_P(phost, 415,95, 50,20, 26, OPT_3D, PSTR("RCL"));

	FT_API_Write_CoCmd(RESTORE_CONTEXT());

	FT_API_Write_CoCmd(DISPLAY());
	FT_GPU_CoCmd_Swap(phost);

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);
}


void FT_touchTrackInit(void)
{
	FT_GPU_CoCmd_Track(phost, 103, 50, 1, 1, VCO1_PITCH);
	FT_GPU_CoCmd_Track(phost, 103,145, 1, 1, VCO2_PITCH);
	FT_GPU_CoCmd_Track(phost, 103,235, 1, 1, LFO_PITCH);

	FT_GPU_CoCmd_Track(phost, 170, 55, 1, 1, MIXER_VCO1);
	FT_GPU_CoCmd_Track(phost, 235, 55, 1, 1, MIXER_VCO2);
	FT_GPU_CoCmd_Track(phost, 170,125, 1, 1, MIXER_LFO);
	FT_GPU_CoCmd_Track(phost, 235,125, 1, 1, MIXER_XMOD);

	FT_GPU_CoCmd_Track(phost, 300, 55, 1, 1, VCF_CUTOFF);
	FT_GPU_CoCmd_Track(phost, 300,125, 1, 1, VCF_PEAK);
	FT_GPU_CoCmd_Track(phost, 365, 55, 1, 1, DELAY_TIME);
	FT_GPU_CoCmd_Track(phost, 365,125, 1, 1, DELAY_FEEDBACK);

	FT_GPU_CoCmd_Track(phost, 440, 55, 1, 1, MASTER);

	/* Wait till coprocessor completes the operation */
	FT_GPU_HAL_WaitCmdfifo_empty(phost);

	// initialise the LUT tables

	synth.vco1.wave_table_ptr = squareWave;
	synth.vco2.wave_table_ptr = triangleWave;
	synth.lfo.wave_table_ptr = sineWave;
	synth.note_table_ptr = concertNoteTable;
	synth.adsr_table_ptr = expTable;	// adsr envelope exponential table

	// initialise the IIR filter
	setIIRFilterLPF( &filter );			// initialise the filter and coefficients with the default values.

	// these are the default values, write them to the synth, so they can be correctly represented on the dials.
	synth.vcf_cutoff = filter.cutoff;	// normalised frequency. Half the maximum frequency = (SAMPLE_RATE>>1 / 2)
	synth.vcf_peak = filter.peak;		// normalised Q (resonance). 1/sqrt(2) = M_SQRT1_2
}

uint8_t FT_touch(void)
{
	uint8_t static oldReadTag = 0;
	uint8_t readTag;
	touch_t TrackRegisterVal;
	uint8_t touched;

	uint16_t stop;
	uint16_t note;


	readTag = FT_GPU_HAL_Rd8(phost, REG_TOUCH_TAG);

	if (readTag && (readTag == oldReadTag) )
	{
		touched = FT_FALSE;		// No new touch. So still the old touch and don't update GUI.
	}
	else
	{
		if ( ((FT_GPU_HAL_Rd32(phost, REG_TOUCH_DIRECT_XY) ) & 0x8000) && (readTag == 0x00) ) // pen is up, no active touch
		{
			if( synth.note )
			{
				touched = FT_TRUE;
				synth.note = FT_FALSE;	// turn off the note.
			}
			else
			{
				touched = FT_FALSE;
			}
		}
		else
		{
			touched = FT_TRUE;

			if(readTag < 0x40) // tags are a toggle
			{
				switch (readTag)
				{
					case (VCO1_TOGGLE):
						synth.vco1.toggle ^= WAVE_ON;
						break;

					case (VCO2_TOGGLE):
						synth.vco2.toggle ^= WAVE_ON;
						break;

					case (LFO_TOGGLE):
						synth.lfo.toggle ^= WAVE_ON;
						break;

					case (VCO1_WAVE):
						synth.vco1.wave ^= WAVE_SAW;
						if (synth.vco1.wave == WAVE_SAW)
							synth.vco1.wave_table_ptr = sineWave;
						else
							synth.vco1.wave_table_ptr = squareWave;
						break;

					case (VCO2_WAVE):
						synth.vco2.wave ^= WAVE_SAW;
						if (synth.vco2.wave == WAVE_SAW)
							synth.vco2.wave_table_ptr = sawWave;
						else
							synth.vco2.wave_table_ptr = triangleWave;
						break;

					case (LFO_WAVE):
						synth.lfo.wave ^= WAVE_TRI;
						if (synth.lfo.wave == WAVE_TRI)
							synth.lfo.wave_table_ptr = triangleWave;
						else
							synth.lfo.wave_table_ptr = sineWave;
						break;

					case (KBD_TOGGLE):
						synth.kbd_toggle ^= KBD_VERDI;
						if (synth.kbd_toggle == KBD_VERDI)
							synth.note_table_ptr = verdiNoteTable;
						else
							synth.note_table_ptr = concertNoteTable;
						break;

					case (SETTINGS):
						eeprom_busy_wait();
						if (synth.settings_loaded)
						{
							eeprom_update_block (&synth, &synth_store, sizeof(synth_t)); // write the settings into EEPROM.
						}
						else
						{
							eeprom_read_block (&synth, &synth_store, sizeof(synth_t)); // read any stored settings into RAM.
							synth.settings_loaded = FT_TRUE;

							// set the pointers to LUTs, which are incorrectly stored in the first RCL following programming.

							synth.note_table_ptr = concertNoteTable;
							synth.adsr_table_ptr = expTable;	// adsr envelope exponential table

							synth.note = FT_FALSE;
							synth.adsr = off;

							if (synth.vco1.wave == WAVE_SAW)
								synth.vco1.wave_table_ptr = sineWave;
							else
								synth.vco1.wave_table_ptr = squareWave;

							if (synth.vco2.wave == WAVE_SAW)
								synth.vco2.wave_table_ptr = sawWave;
							else
								synth.vco2.wave_table_ptr = triangleWave;

							if (synth.lfo.wave == WAVE_TRI)
								synth.lfo.wave_table_ptr = triangleWave;
							else
								synth.lfo.wave_table_ptr = sineWave;
						}
						break;

					default:
						break;
				}

				vTaskDelay( 128 / portTICK_PERIOD_MS ); // debounce the toggles.
			}
			else if (readTag > 0x80)// tag is greater than 0x80 and therefore is a dial.
			{
				TrackRegisterVal.u32 = FT_GPU_HAL_Rd32(phost, REG_TRACKER);

				switch (TrackRegisterVal.touch.tag)
				{
					case (VCO1_PITCH):
						synth.vco1.pitch = TrackRegisterVal.touch.value & 0xe000;
						break;

					case (VCO2_PITCH):
						synth.vco2.pitch = TrackRegisterVal.touch.value;
						break;

					case (LFO_PITCH):
						synth.lfo.pitch = TrackRegisterVal.touch.value;
						break;

					case (MIXER_VCO1):
						synth.vco1.volume = TrackRegisterVal.touch.value;
						break;

					case (MIXER_VCO2):
						synth.vco2.volume = TrackRegisterVal.touch.value;
						break;

					case (MIXER_LFO):
						synth.lfo.volume = TrackRegisterVal.touch.value;
						break;

					case (MIXER_XMOD):
						synth.xmod = TrackRegisterVal.touch.value;
						break;

					case (VCF_CUTOFF):
						synth.vcf_cutoff = TrackRegisterVal.touch.value;
						// set the VCF
						filter.cutoff = synth.vcf_cutoff;
						setIIRFilterLPF( &filter );
						break;

					case (VCF_PEAK):
						synth.vcf_peak = TrackRegisterVal.touch.value;
						// set the VCF
						filter.peak = synth.vcf_peak;
						setIIRFilterLPF( &filter );
						break;

					case (DELAY_TIME):
						synth.delay_time = TrackRegisterVal.touch.value;
						break;

					case (DELAY_FEEDBACK):
						synth.delay_feedback = TrackRegisterVal.touch.value;
						break;

					case (MASTER):
						synth.master = TrackRegisterVal.touch.value;
						break;

					default:
						break;
				}
			}
			else
			{
				synth.note = readTag; // an ASCII key was pressed, set the note to be the read tag.

				// now to calculate the phase_increment (which sets the tone) for each of the three oscillators.

				//  for VCO1 - we'll have 8 registers.
				stop = 0;
				switch (synth.vco1.pitch)
				{
				case STOP_C8:
					++stop;
				case STOP_C7:
					++stop;
				case STOP_C6:
					++stop;
				case STOP_C5:
					++stop;
				case STOP_C4:
					++stop;
				case STOP_C3:
					++stop;
				case STOP_C2:
					++stop;
				case STOP_C1:
				default:
					break;
				}

				// now to set the note within the register.
				note= 0;
				switch (synth.note)
				{
				case 'a':
					++note;
				case 'g':
					++note;
				case 'f':
					++note;
				case 'd':
					++note;
				case 'c':
					++note;
				case 'B':
					++note;
				case 'A':
					++note;
				case 'G':
					++note;
				case 'F':
					++note;
				case 'E':
					++note;
				case 'D':
					++note;
				case 'C':
				default:
					break;
				}

				//  setting the phase increment for VCO1 is frequency * LUT size / sample rate.
				synth.vco1.phase_increment = (uint32_t)pgm_read_dword(synth.note_table_ptr + (stop * NOTES) + note) / (SAMPLE_RATE >> 1) ; //  << 1 is scale to 24.8 fixed point.

				// set the VCO2 phase increment to be -1 octave to +1 octave from VCO1, with centre dial frequency identical.
				if (synth.vco2.pitch & 0x8000) // upper half dial
//					synth.vco2.phase_increment = (uint32_t)(((uint64_t)synth.vco1.phase_increment * ((uint32_t)synth.vco2.pitch << 1)) >> 16);
					synth.vco2.phase_increment = ((synth.vco1.phase_increment >> 4) * synth.vco2.pitch ) >> 11;
				else // lower half dial
//					synth.vco2.phase_increment = (synth.vco1.phase_increment >> 1) + (uint32_t)(( (synth.vco1.phase_increment >> 1) * ((uint32_t)synth.vco2.pitch << 1) ) >> 16 );
					synth.vco2.phase_increment = (synth.vco1.phase_increment >> 1) + (((synth.vco1.phase_increment >> 4) * synth.vco2.pitch) >> 12);

				// set the LFO phase increment to be from 0 Hz to 32 Hz.
				synth.lfo.phase_increment = ((uint32_t)synth.lfo.pitch * LUT_SIZE / ((uint32_t)SAMPLE_RATE << 4) );
			}
		}
		oldReadTag = readTag;
	}
	return touched;
}


static uint8_t shieldPhysicalIO(uint8_t button)
{
	uint8_t static buttonState;

	if (button != 0)
		buttonState = button;

	if (buttonState != 0)
	{
		switch (buttonState)
		{
			case (VCO1_BUTTON):
				synth.vco1.pitch = (UINT16_MAX - (mod0Value << (6 - DECIMATE))) & 0xe000;
				synth.vco1.volume = UINT16_MAX - (mod1Value << (6 - DECIMATE));
				break;

			case (VCO2_BUTTON):
				synth.vco2.pitch = UINT16_MAX - (mod0Value << (6 - DECIMATE));
				synth.vco2.volume = UINT16_MAX - (mod1Value << (6 - DECIMATE));

				// set the VCO2 phase increment to be -1 octave to +1 octave from VCO1, with centre dial frequency identical.
				if (synth.vco2.pitch & 0x8000) // upper half dial
					synth.vco2.phase_increment = ((synth.vco1.phase_increment >> 4) * synth.vco2.pitch ) >> 11;
				else // lower half dial
					synth.vco2.phase_increment = (synth.vco1.phase_increment >> 1) + (((synth.vco1.phase_increment >> 4) * synth.vco2.pitch) >> 12);

				break;

			case (LFO_BUTTON):
				synth.lfo.pitch = UINT16_MAX - (mod0Value << (6 - DECIMATE));
				synth.lfo.volume = UINT16_MAX - (mod1Value << (6 - DECIMATE));

				// set the LFO phase increment to be from 0 Hz to 32 Hz.
				synth.lfo.phase_increment = ((uint32_t)synth.lfo.pitch * LUT_SIZE / ((uint32_t)SAMPLE_RATE << 4) );
				break;

			case (VCF_BUTTON):
				filter.cutoff = synth.vcf_cutoff = UINT16_MAX - (mod0Value << (6 - DECIMATE));
				filter.peak = synth.vcf_peak = UINT16_MAX - (mod1Value << (6 - DECIMATE));
				// set the VCF
				setIIRFilterLPF( &filter );
				break;

			case (DELAY_BUTTON):
				synth.delay_time = UINT16_MAX - (mod0Value << (6 - DECIMATE));
				synth.delay_feedback = UINT16_MAX - (mod1Value << (6 - DECIMATE));
				break;

			case (CANCEL_BUTTON): // all buttons pressed
				buttonState = 0;	// reset buttonState to 0 (none pressed, physical IO turned off).
				break;

			default:
				break;
		}

		return FT_TRUE;
	}
	return FT_FALSE;
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
/* Interrupts */
/*-----------------------------------------------------------*/




/*-----------------------------------------------------------*/



